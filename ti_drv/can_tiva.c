#include <drivers/can.h>
#include <kernel.h>

#include "cfg_tiva.h"
#include "pinmux_tiva.h"
#include "sysctl_tiva.h"

#include <logging/log.h>
// LOG_MODULE_DECLARE(can_driver, CONFIG_CAN_LOG_LEVEL);
LOG_MODULE_REGISTER(can_driver, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT ti_tiva_can

/*
 * Each CAN peripheral has 32 message objects capable of TX/RX.
 * We dedicate some of them for TX slots and rest is used for RX slots.
 */
#define MAX_CAN_SLOTS 32
#define CAN_TX_SLOTS 2
#define CAN_RX_SLOTS (MAX_CAN_SLOTS - CAN_TX_SLOTS)


/*
 * Macro to exclude the sample point algorithm from compilation if not used
 * Without the macro, the algorithm would always waste ROM.
 */
#define _SP_IS_SET(inst) DT_INST_NODE_HAS_PROP(inst, sample_point) ||
#define USE_SP_ALGO (DT_INST_FOREACH_STATUS_OKAY(_SP_IS_SET) 0)

// Device configuration.
struct can_tiva_config {
	uint32_t base;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	uint32_t bus_speed;
	uint16_t sample_point;
	uint8_t sjw;
	uint8_t prop_ts1;
	uint8_t ts2;
};

// RX slot.
struct can_rx_slot {
	volatile bool free;
	can_rx_callback_t callback;
	void * callback_arg;
	volatile uint32_t error;
	tCANMsgObject msg_object;
	uint8_t raw_data[8];
};

// TX slot.
struct can_tx_slot {
	volatile bool free;
	struct k_sem io_sem;
	can_tx_callback_t callback;
	void * callback_arg;
	volatile uint32_t error;
};

// TX slots and semaphore.
struct can_tiva_tx_data {
	struct can_tx_slot slots[CAN_TX_SLOTS];
	struct k_sem free_sem;
};

// RX slots and mutex.
struct can_tiva_rx_data {
	struct can_rx_slot slots[CAN_RX_SLOTS];
	struct k_mutex slots_lock;
};

// Device data.
struct can_tiva_data {
	struct can_tiva_tx_data tx;
	struct can_tiva_rx_data rx;
	struct k_mutex api_lock;
	can_state_change_callback_t state_change_isr;
	void * user_data;
};

// Get device config
#define DEV_CFG(dev)						\
	((const struct can_tiva_config *const)	\
	(dev)->config)

// Get device data
#define DEV_DATA(dev)						\
	((struct can_tiva_data *const)			\
	(dev)->data)

// TI HAL uses linear message space, so we put TX slots after RX slots.
#define RX_SLOT_MASK(slot_idx) (1 << slot_idx) // Bit mask for given RX slot.
#define RX_SLOTS_MASK ((uint32_t)((1ULL << CAN_RX_SLOTS) - 1)) // Bit mask for all RX slots.
#define TX_SLOT_MASK(slot_idx) (1 << (slot_idx + CAN_RX_SLOTS)) // Bit mask for given TX slot.
#define TX_SLOTS_MASK ((uint32_t)(((1ULL << MAX_CAN_SLOTS) - 1) & ~RX_SLOTS_MASK)) // Bit mask for all TX slots.

// TI HAL uses linear message object space with index starting at 1, so we put TX slots after RX slots.
#define RX_SLOT_OBJ_ID(slot_idx) (slot_idx + 1)
#define TX_SLOT_OBJ_ID(slot_idx) (slot_idx + CAN_RX_SLOTS + 1)

// CAN interrupt mask.
#define INT_MASK (CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS)

// Copied from can_common.c as it's not present in any header file.
#define CAN_SYNC_SEG 1

// Lock API mutex and CAN IRQs.
static void can_tiva_api_lock(const struct device *dev)
{
	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	k_mutex_lock(&dev_data->api_lock, K_FOREVER);
	CANIntDisable(dev_cfg->base, INT_MASK);
}

// Unlock API mutex and CAN IRQs.
static void can_tiva_api_unlock(const struct device *dev)
{
	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	CANIntEnable(dev_cfg->base, INT_MASK);
	k_mutex_unlock(&dev_data->api_lock);
}

// Convert status from TI to Zephyr.
static enum can_state can_tiva_status_convert(uint32_t status)
{
	if ((status & CAN_STATUS_BUS_OFF) == CAN_STATUS_BUS_OFF)
	{
		return CAN_BUS_OFF;
	}
	else if ((status & CAN_STATUS_EPASS) == CAN_STATUS_EPASS)
	{
		return CAN_ERROR_PASSIVE;
	}

	return CAN_ERROR_ACTIVE;
}

// Called from ISR to indicate TX end for given slot.
static void can_tiva_tx_done_from_isr(const struct device *dev, size_t tx_slot_idx)
{
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);
	struct can_tiva_data * const dev_data = DEV_DATA(dev);

	struct can_tx_slot * slot = &dev_data->tx.slots[tx_slot_idx];

	if (slot->free == false)
	{
		/*
		 * If callback is set, use callback to notify TX status,
		 * Otherwise give semaphore as send function is blocked on it.
		 */
		if (slot->callback != NULL)
		{
			slot->callback(dev, slot->error, slot->callback_arg);
			slot->callback = NULL;
		}
		else
		{
			k_sem_give(&slot->io_sem);
		}

		CANMessageClear(dev_cfg->base, TX_SLOT_OBJ_ID(tx_slot_idx));

		// Free the TX slot.
		slot->free = true;
		k_sem_give(&dev_data->tx.free_sem);
	}

}

int can_tiva_set_mode(const struct device *dev, can_mode_t mode)
{
	// Only normal mode is supported.
	if (mode != CAN_MODE_NORMAL)
	{
		return -ENOTSUP;
	}

	return 0;
}

int can_tiva_set_timing(const struct device *dev, const struct can_timing *timing) {
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	tCANBitClkParms clk_params;

	// Sync, propagation and phase 1 is combined.
	clk_params.ui32SyncPropPhase1Seg = CAN_SYNC_SEG + timing->prop_seg + timing->phase_seg1;
	clk_params.ui32Phase2Seg = timing->phase_seg2;
	clk_params.ui32SJW = timing->sjw;
	clk_params.ui32QuantumPrescaler = timing->prescaler - 1;

	can_tiva_api_lock(dev);

	CANBitTimingSet(dev_cfg->base, &clk_params);

	can_tiva_api_unlock(dev);

	return 0;
}

int can_tiva_send(const struct device *dev, const struct zcan_frame *msg,
		   k_timeout_t timeout, can_tx_callback_t callback,
		   void *callback_arg)
{
	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);
	int err;

	// Only dataframe is supported.
	if (msg->rtr != CAN_DATAFRAME)
	{
		LOG_ERR("Remote requests are not supported");
		return -ENOTSUP;
	}

	// CAN FD is not supported.
	if (msg->fd != 0)
	{
		LOG_ERR("CAN-FD is not supported");
		return -ENOTSUP;
	}

	/*
	 * Wait for free TX slot,
	 * free_sem tracks number of free slots allowing to block waiting for one.
	 */
	err = k_sem_take(&dev_data->tx.free_sem, timeout);
	if (err != 0)
	{
		LOG_WRN("Failed waiting for free TX slot");
		return err;
	}

	/*
	 * We can't lock earlier as it will block ISR thus not allowing to free up any TX slot.
	 * We have to lock here as TX ISR modifies this data.
	 */
	can_tiva_api_lock(dev);

	struct can_tx_slot * slot = NULL;
	uint32_t slot_obj_id = 0;

	for (size_t i = 0; i < CAN_TX_SLOTS; i++)
	{
		if (dev_data->tx.slots[i].free == true)
		{
			slot = &dev_data->tx.slots[i];
			slot_obj_id = TX_SLOT_OBJ_ID(i);

			break;
		}
	}

	// TX free_sem track number of free TX slots so this should never happen.
	__ASSERT(slot != NULL, "No free slot found, despite semaphore being taken\n");

	// Mark the slot as used
	slot->free = false;

	slot->callback = callback;
	slot->callback_arg = callback_arg;

	const uint32_t extended_id_flag = msg->id_type == CAN_EXTENDED_IDENTIFIER ? MSG_OBJ_EXTENDED_ID : 0;
	tCANMsgObject msg_object = {
		.ui32MsgID = msg->id,
		.ui32MsgIDMask = 0,
		.ui32Flags = MSG_OBJ_TX_INT_ENABLE | extended_id_flag,
		.ui32MsgLen = msg->dlc,
		.pui8MsgData = (uint8_t *) msg->data,
	};

	CANIntClear(dev_cfg->base, slot_obj_id);
	k_sem_reset(&slot->io_sem);
	CANMessageSet(dev_cfg->base, slot_obj_id, &msg_object, MSG_OBJ_TYPE_TX);

	can_tiva_api_unlock(dev);

	// If callback is not provided this is a blocking call so we wait for io_sem.
	if (callback == NULL)
	{
		k_sem_take(&slot->io_sem, K_FOREVER);
		return slot->error;
	}

	return 0;
}

int can_tiva_add_rx_filter (const struct device *dev,
	can_rx_callback_t callback,
	void *user_data,
	const struct zcan_filter *filter) {

	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	k_mutex_lock(&dev_data->rx.slots_lock, K_FOREVER);

	const uint32_t extended_id_flag = filter->id_type == CAN_EXTENDED_IDENTIFIER ? MSG_OBJ_EXTENDED_ID : 0;

	struct can_rx_slot * slot = NULL;
	uint32_t slot_obj_id = 0;
	size_t slot_idx = 0;

	// Search for first free RX slot.
	for (size_t i = 0; i < CAN_RX_SLOTS; i++)
	{
		if (dev_data->rx.slots[i].free == true)
		{
			slot = &dev_data->rx.slots[i];
			slot_obj_id = RX_SLOT_OBJ_ID(i);
			slot_idx = i;

			slot->free = false;
			slot->callback = callback;
			slot->callback_arg = user_data;

			tCANMsgObject *msg_object = &slot->msg_object;

			msg_object->ui32MsgID = filter->id;
			msg_object->ui32MsgIDMask = filter->id_mask;
			msg_object->ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | extended_id_flag;
			msg_object->ui32MsgLen = sizeof(slot->raw_data);
			msg_object->pui8MsgData = slot->raw_data;

			break;
		}
	}

	k_mutex_unlock(&dev_data->rx.slots_lock);

	// No free slot found.
	if (slot == NULL) {
		LOG_WRN("All RX slots are occupied");
		return -ENOSPC;
	}

	can_tiva_api_lock(dev);

	CANIntClear(dev_cfg->base, slot_obj_id);
	CANMessageSet(dev_cfg->base, slot_obj_id, &slot->msg_object, MSG_OBJ_TYPE_RX);

	can_tiva_api_unlock(dev);

	return slot_idx;
}

void can_tiva_remove_rx_filter(const struct device *dev, int filter_id){

	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	uint32_t slot_obj_id = RX_SLOT_OBJ_ID(filter_id);

	can_tiva_api_lock(dev);

	CANMessageClear(dev_cfg->base, slot_obj_id);
	CANIntClear(dev_cfg->base, slot_obj_id);

	can_tiva_api_unlock(dev);

	k_mutex_lock(&dev_data->rx.slots_lock, K_FOREVER);

	// Mark the slot as free.
	dev_data->rx.slots[filter_id].free = true;

	k_mutex_unlock(&dev_data->rx.slots_lock);
}

int can_tiva_get_state(const struct device *dev, enum can_state *state, struct can_bus_err_cnt *err_cnt) {
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	can_tiva_api_lock(dev);
	if (NULL != state) {
		*state = can_tiva_status_convert(CANStatusGet(dev_cfg->base, CAN_STS_CONTROL));
	}
	if (NULL != err_cnt) {
		err_cnt->rx_err_cnt = 0;
		err_cnt->tx_err_cnt = 0;
	}
	can_tiva_api_unlock(dev);
	return 0;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
#error "Manual bus-off recovery is not supported yet"
int can_tiva_recover(const struct device *dev, k_timeout_t timeout)
{
	/// TODO: Not implemented
	return -ENOTSUP;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

void can_tiva_set_state_change_callback(const struct device *dev, can_state_change_callback_t callback, void *user_data) {
	struct can_tiva_data * const dev_data = DEV_DATA(dev);

	dev_data->state_change_isr = callback;
	dev_data->user_data = user_data;
}

int can_tiva_get_core_clock(const struct device *dev, uint32_t *rate)
{
	*rate = SysCtlClockGet();

	return 0;
}

int can_tiva_get_max_filters (const struct device *dev, enum can_ide id_type) {
	return CAN_RX_SLOTS;
}

int can_tiva_get_max_bitrate (const struct device *dev, uint32_t *max_bitrate) {
	*max_bitrate = 1000000; // 1Mbit from datasheet
	return 0;
}

int can_tiva_init(const struct device *dev)
{
	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	struct can_timing timing = {};
    int ret;

	// Configure pinmux
	pinmux_tiva_arrayCfg(DEV_CFG(dev)->pinctrl_list, DEV_CFG(dev)->pinctrl_list_size);

	// Enable peripheral clock
	sysctl_activatePeripheral(dev_cfg->base);

	// Init RX chain
    for (size_t i = 0; i < CAN_RX_SLOTS; i++)
    {
		CANMessageClear(dev_cfg->base, RX_SLOT_OBJ_ID(i));

		dev_data->rx.slots[i].free = true;
    }

	ret = k_mutex_init(&dev_data->rx.slots_lock);
	if (ret != 0)
	{
		LOG_ERR("Failed to init mutex: %d", ret);
		return ret;
	}

	// Init TX chain
	ret = k_sem_init(&dev_data->tx.free_sem, CAN_TX_SLOTS, CAN_TX_SLOTS);
	if (ret != 0)
	{
		LOG_ERR("Failed to init sem: %d", ret);
		return ret;
	}

	for (size_t i = 0; i < CAN_TX_SLOTS; i++)
	{
		ret = k_sem_init(&dev_data->tx.slots[i].io_sem, 0, 1);
		if (ret != 0)
		{
			LOG_ERR("Failed to init sem: %d", ret);
			return ret;
		}

		CANMessageClear(dev_cfg->base, TX_SLOT_OBJ_ID(i));

		dev_data->tx.slots[i].free = true;
	}

	// Common init
	ret = k_mutex_init(&dev_data->api_lock);
	if (ret!= 0)
	{
		LOG_ERR("Failed to init mutex: %d", ret);
		return ret;
	}

	dev_data->state_change_isr = NULL;
	dev_data->user_data = NULL;

	// Init peripheral
	CANInit(dev_cfg->base);


	// Setup timings
	timing.sjw = dev_cfg->sjw;

	if ((dev_cfg->sample_point != 0) && USE_SP_ALGO) {
		ret = can_calc_timing(dev, &timing, dev_cfg->bus_speed,
				      dev_cfg->sample_point);
		if (ret == -EINVAL) {
			LOG_ERR("Can't find timing for given param");
			return -EIO;
		}
		LOG_DBG("Presc: %d, TS1: %d, TS2: %d",
			timing.prescaler, timing.phase_seg1, timing.phase_seg2);
		LOG_DBG("Sample-point err : %d", ret);
	} else {
		timing.prop_seg = 0;
		timing.phase_seg1 = dev_cfg->prop_ts1;
		timing.phase_seg2 = dev_cfg->ts2;
		ret = can_calc_prescaler(dev, &timing, dev_cfg->bus_speed);
		if (ret != 0) {
			LOG_WRN("Bitrate error: %d", ret);
		}
	}

	ret = can_tiva_set_timing(dev, &timing);
	if (ret != 0)
	{
		LOG_ERR("Failed to set timing: %d", ret);
		return ret;
	}

	// Enable interrupts
	CANIntEnable(dev_cfg->base, INT_MASK);

	// Enable peripheral
	CANEnable(dev_cfg->base);

	return 0;
}

// Called for every successful TX operation.
static inline void can_tiva_tx_isr(const struct device *dev, size_t tx_slot_idx)
{
	struct can_tiva_data * const dev_data = DEV_DATA(dev);

	struct can_tx_slot *slot = &dev_data->tx.slots[tx_slot_idx];

	// ToDo: tx isr error
	//slot->error = CAN_TX_OK;
	slot->error = 0;
	can_tiva_tx_done_from_isr(dev, tx_slot_idx);
}

// Called for every successful RX operation.
static inline void can_tiva_rx_isr(const struct device *dev, size_t rx_slot_idx)
{
	struct can_tiva_data * const dev_data = DEV_DATA(dev);
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	struct can_rx_slot *slot = &dev_data->rx.slots[rx_slot_idx];

	if ((slot->free == false) && (slot->callback != NULL))
	{
		CANMessageGet(dev_cfg->base, RX_SLOT_OBJ_ID(rx_slot_idx), &slot->msg_object, true);

		struct zcan_frame msg = {
			.id = slot->msg_object.ui32MsgID,
			.fd = 0,
			.rtr = CAN_DATAFRAME,
			.id_type = CAN_STANDARD_IDENTIFIER,
			.dlc = (uint8_t) slot->msg_object.ui32MsgLen,
			.brs = 0,
			.data = {}
		};

		memcpy(msg.data, slot->raw_data, sizeof(msg.data));

		slot->callback(dev, &msg, slot->callback_arg);
	}
}

// Called for every CAN status update.
static inline void can_tiva_state_change_isr(const struct device *dev)
{
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);
	struct can_tiva_data * const dev_data = DEV_DATA(dev);

	// Reading the status clears the IRQ.
	const uint32_t status = CANStatusGet(dev_cfg->base, CAN_STS_CONTROL);
	const uint32_t pending_tx = CANStatusGet(dev_cfg->base, CAN_STS_TXREQUEST);

	// Notify about state change if handler is registered
	if (dev_data->state_change_isr != NULL)
	{
		uint32_t rx_err_cnt = 0;
		uint32_t tx_err_cnt = 0;

		CANErrCntrGet(dev_cfg->base, &rx_err_cnt, &tx_err_cnt);

		struct can_bus_err_cnt err_cnt = {
			.rx_err_cnt = rx_err_cnt > UINT8_MAX ? UINT8_MAX : (uint8_t)rx_err_cnt,
			.tx_err_cnt = tx_err_cnt > UINT8_MAX ? UINT8_MAX : (uint8_t)tx_err_cnt,
		};

		dev_data->state_change_isr(dev, can_tiva_status_convert(status), err_cnt, dev_data->user_data);
	}

	// ToDo:
	//uint32_t tx_error = CAN_TX_UNKNOWN;
	uint32_t tx_error = 0;

	// Look for any transmission errors.
	if ((status & CAN_STATUS_LEC_MSK) == CAN_STATUS_LEC_ACK)
	{
		//tx_error = CAN_TX_ERR;
		tx_error = -EIO;
	}
	else if ((status & CAN_STATUS_LEC_MSK) == CAN_STATUS_LEC_BIT1)
	{
		//tx_error = CAN_TX_ARB_LOST;
		tx_error = -EIO;
	}
	else if ((status & CAN_STATUS_LEC_MSK) == CAN_STATUS_LEC_BIT0)
	{
		//tx_error = CAN_TX_ARB_LOST;
		tx_error = -EIO;
	}
	else if ((status & CAN_STATUS_BUS_OFF) != 0)
	{
		//tx_error = CAN_TX_BUS_OFF;
		tx_error = -EIO;
	}

	// TX is pending and TX tx_error is set so error is connected with highest priority pending TX.
	//if ((tx_error != CAN_TX_UNKNOWN) && pending_tx)
	if ((tx_error == -EIO) && pending_tx)
	{
		for (size_t i = 0; i < CAN_TX_SLOTS; i++)
		{
			if ((pending_tx & TX_SLOT_MASK(i)) != 0)
			{
				CANIntClear(dev_cfg->base, TX_SLOT_OBJ_ID(i));

				dev_data->tx.slots[i].error = tx_error;
				can_tiva_tx_done_from_isr(dev, i);

				// Lower number slot has higher priority so reported status is connected with it.
				break;
			}
		}
	}
}

static void can_tiva_isr(const struct device *dev)
{
	const struct can_tiva_config * const dev_cfg = DEV_CFG(dev);

	const uint32_t int_cause = CANIntStatus(dev_cfg->base, CAN_INT_STS_CAUSE);

	/*
	 * Currently, as side efect, we call can_tiva_state_change_isr for successful TX operation,
	 * it wastes some time but is harmful. We can fix this in the future if needed.
	 */
	if (int_cause == CAN_INT_INTID_STATUS)
	{
		can_tiva_state_change_isr(dev);
	}

	const uint32_t irq_mask = CANIntStatus(dev_cfg->base, CAN_INT_STS_OBJECT);

	// CHeck TX slots.
	if ((irq_mask & TX_SLOTS_MASK) != 0)
	{
		for (size_t i = 0; i < CAN_TX_SLOTS; i++)
		{
			const uint32_t slot_mask = TX_SLOT_MASK(i);

			if ((irq_mask & slot_mask) != 0)
			{
				CANIntClear(dev_cfg->base, TX_SLOT_OBJ_ID(i));
				can_tiva_tx_isr(dev, i);
			}
		}
	}

	// Check RX slots.
	if ((irq_mask & RX_SLOTS_MASK) != 0)
	{
		for (size_t i = 0; i < CAN_RX_SLOTS; i++)
		{
			const uint32_t slot_mask = RX_SLOT_MASK(i);

			if ((irq_mask & slot_mask) != 0)
			{
				CANIntClear(dev_cfg->base, RX_SLOT_OBJ_ID(i));
				can_tiva_rx_isr(dev, i);
			}
		}
	}
}

static const struct can_driver_api can_api_funcs = {
	.set_mode = can_tiva_set_mode,
	.set_timing = can_tiva_set_timing,
	.send = can_tiva_send,
	.add_rx_filter = can_tiva_add_rx_filter,
	.remove_rx_filter = can_tiva_remove_rx_filter,
/*
	.attach_isr = can_tiva_attach_isr,
	.detach = can_tiva_detach,
*/
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_tiva_recover,
#endif
	.get_state = can_tiva_get_state,
	.set_state_change_callback = can_tiva_set_state_change_callback,
	.get_core_clock = can_tiva_get_core_clock,
	.get_max_filters = can_tiva_get_max_filters,
	.get_max_bitrate = can_tiva_get_max_bitrate,
	.timing_min = {
		.sjw = 0x1,
		.prop_seg = 0x00,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x04,
		.prop_seg = 0x00,
		.phase_seg1 = 0x0F,
		.phase_seg2 = 0x08,
		.prescaler = 0x400
	}
#if defined(CONFIG_CAN_FD_MODE) || defined(__DOXYGEN__)
	.set_timing_data = NULL,
	.timing_data_min = {0,},
	.timing_data_max = {0,},
#endif /* CONFIG_CAN_FD_MODE */

};

#define TIVA_CAN_DEVICE(n)																\
	static const ti_tiva_gpio_pinctrl_t can_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct can_tiva_config can_## n ##_tiva_cfg={							\
			.base = DT_INST_REG_ADDR(n),												\
			.pinctrl_list = can_pins_##n,												\
			.pinctrl_list_size = ARRAY_SIZE(can_pins_##n),								\
			.bus_speed = DT_INST_PROP(n, bus_speed), 									\
			.sample_point = DT_INST_PROP_OR(n, sample_point, 0),						\
			.sjw = DT_INST_PROP_OR(n, sjw, 1),											\
			.prop_ts1 = DT_INST_PROP_OR(n, prop_seg, 0) +								\
					DT_INST_PROP_OR(n, phase_seg1, 0),									\
			.ts2 = DT_INST_PROP_OR(n, phase_seg2, 0),									\
	};																					\
	static struct can_tiva_data can_## n ##_tiva_runtime;					        	\
	static int can_## n ##_tiva_init(const struct device *dev) {						\
		IRQ_CONNECT(DT_INST_IRQN(n),													\
			    DT_INST_IRQ(n, priority),												\
			    can_tiva_isr, DEVICE_DT_INST_GET(n), 0);								\
		irq_enable(DT_INST_IRQN(n));													\
		return can_tiva_init(dev);														\
	}																					\
	DEVICE_DT_INST_DEFINE(n,															\
		can_## n ##_tiva_init,															\
		NULL,																			\
		&can_## n ##_tiva_runtime,														\
		&can_## n ##_tiva_cfg,															\
		POST_KERNEL, 50,																\
		&can_api_funcs);								    							\

DT_INST_FOREACH_STATUS_OKAY(TIVA_CAN_DEVICE)
