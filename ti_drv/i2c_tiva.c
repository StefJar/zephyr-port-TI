/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_i2c

#include <kernel.h>
#include <drivers/i2c.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"
#include "pinmux_tiva.h"

#include "i2c-priv.h"

struct i2c_tiva_config {
	uint32_t base;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
};

struct i2c_tiva_runtime {
	struct k_mutex lock;
	struct k_sem complete;
	volatile uint32_t error;
};

#define DEV_CFG(dev) ((const struct i2c_tiva_config *const)(dev)->config)

#define DEV_DATA(dev) ((struct i2c_tiva_runtime *const)(dev)->data)

static int i2c_tiva_transmit(const struct device *dev, const uint8_t *buf, uint32_t len, uint16_t addr){
	const uint32_t base = DEV_CFG(dev)->base;
	struct i2c_tiva_runtime *data = DEV_DATA(dev);

	/* Sending address without data is not supported */
	if (len == 0) {
		return -EIO;
	}

	I2CMasterSlaveAddrSet(base, addr, false);

	/* The following assumes a single master. Use I2CMasterBusBusy() if
	 * wanting to implement multiple master support.
	 */

	/* Single transmission */
	if (len == 1) {
		I2CMasterDataPut(base, *buf);

		I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);

		k_sem_take(&data->complete, K_FOREVER);

		return data->error == I2C_MASTER_ERR_NONE ? 0 : -EIO;
	}

	/* Burst transmission */
	I2CMasterDataPut(base, buf[0]);

	I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);

	k_sem_take(&data->complete, K_FOREVER);

	if (data->error != I2C_MASTER_ERR_NONE) {
		goto send_error_stop;
	}

	for (int i = 1; i < len - 1; i++) {
		I2CMasterDataPut(base, buf[i]);

		I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);

		k_sem_take(&data->complete, K_FOREVER);

		if (data->error != I2C_MASTER_ERR_NONE) {
			goto send_error_stop;
		}
	}

	I2CMasterDataPut(base, buf[len - 1]);

	I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);

	k_sem_take(&data->complete, K_FOREVER);

	if (data->error != I2C_MASTER_ERR_NONE) {
		return -EIO;
	}

	return 0;

send_error_stop:
	I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
	return -EIO;
}

static int i2c_tiva_receive(const struct device *dev, uint8_t *buf, uint32_t len, uint16_t addr)
{
	const uint32_t base = DEV_CFG(dev)->base;
	struct i2c_tiva_runtime *data = DEV_DATA(dev);

	/* Sending address without data is not supported */
	if (len == 0) {
		return -EIO;
	}

	I2CMasterSlaveAddrSet(base, addr, true);

	/* The following assumes a single master. Use I2CMasterBusBusy() if
	 * wanting to implement multiple master support.
	 */

	/* Single receive */
	if (len == 1) {
		I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);

		k_sem_take(&data->complete, K_FOREVER);

		if (data->error != I2C_MASTER_ERR_NONE) {
			return -EIO;
		}

		*buf = I2CMasterDataGet(base);

		return 0;
	}

	/* Burst receive */
	I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);

	k_sem_take(&data->complete, K_FOREVER);

	if (data->error != I2C_MASTER_ERR_NONE) {
		goto recv_error_stop;
	}

	buf[0] = I2CMasterDataGet(base);

	for (int i = 1; i < len - 1; i++) {
		I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

		k_sem_take(&data->complete, K_FOREVER);

		if (data->error != I2C_MASTER_ERR_NONE) {
			goto recv_error_stop;
		}

		buf[i] = I2CMasterDataGet(base);
	}

	I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	k_sem_take(&data->complete, K_FOREVER);

	if (data->error != I2C_MASTER_ERR_NONE) {
		return -EIO;
	}

	buf[len - 1] = I2CMasterDataGet(base);

	return 0;

recv_error_stop:
	I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
	return -EIO;
}

static int i2c_tiva_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
	int ret = 0;

	if (num_msgs == 0) {
		return 0;
	}

	k_mutex_lock(&DEV_DATA(dev)->lock, K_FOREVER);

	for (int i = 0; i < num_msgs; i++) {
		/* Not supported by hardware */
		if (msgs[i].flags & I2C_MSG_ADDR_10_BITS) {
			ret = -EIO;
			break;
		}

		if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = i2c_tiva_transmit(dev, msgs[i].buf, msgs[i].len, addr);
		} else {
			ret = i2c_tiva_receive(dev, msgs[i].buf, msgs[i].len, addr);
		}

		if (ret) {
			break;
		}
	}

	k_mutex_unlock(&DEV_DATA(dev)->lock);

	return ret;
}

static int i2c_tiva_configure(const struct device *dev, uint32_t dev_config)
{
	bool fast;

	switch (I2C_SPEED_GET(dev_config)) {
		case I2C_SPEED_STANDARD:
			fast = false;
			break;
		case I2C_SPEED_FAST:
			fast = true;
			break;
		default:
			return -EIO;
	}

	/* Support for slave mode has not been implemented */
	if (!(dev_config & I2C_MODE_MASTER)) {
		return -EIO;
	}

	/* This is deprecated and could be ignored in the future */
	if (dev_config & I2C_ADDR_10_BITS) {
		return -EIO;
	}

	/* Enables and configures I2C master */
	I2CMasterInitExpClk(DEV_CFG(dev)->base, SysCtlClockGet(), fast);

	I2CMasterTimeoutSet(DEV_CFG(dev)->base, 0xFF);
	return 0;
}

static void i2c_tiva_isr(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct i2c_tiva_runtime *data = DEV_DATA(dev);

	if (I2CMasterIntStatus(base, true)) {
		I2CMasterIntClear(base);
		data->error = I2CMasterErr(base);
		k_sem_give(&data->complete);
	}
}

static const struct i2c_driver_api i2c_tiva_driver_api = {
	.configure = i2c_tiva_configure,
	.transfer = i2c_tiva_transfer,
	.slave_register = NULL,
	.slave_unregister = NULL,
	.recover_bus = NULL,
};

#define TIVA_I2C_DEVICE(n)																\
	static const ti_tiva_gpio_pinctrl_t i2c_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct i2c_tiva_config i2c_## n ##_tiva_cfg={							\
			.base = DT_INST_REG_ADDR(n),												\
			.pinctrl_list = i2c_pins_##n,												\
			.pinctrl_list_size = ARRAY_SIZE(i2c_pins_##n),								\
	};																					\
	static struct i2c_tiva_runtime i2c_## n ##_tiva_runtime;							\
	static int i2c_## n ##_tiva_init(const struct device *dev) {						\
		uint32_t cfg; 																	\
		int err; 																		\
		IRQ_CONNECT(DT_INST_IRQN(n),													\
			    DT_INST_IRQ(n, priority),												\
			    i2c_tiva_isr, DEVICE_DT_INST_GET(n), 0);								\
		irq_enable(DT_INST_IRQN(n));													\
		k_sem_init(&(DEV_DATA(dev)->complete), 0, 1); 									\
		k_mutex_init(&(DEV_DATA(dev)->lock));											\
		DEV_DATA(dev)->error = I2C_MASTER_ERR_NONE;										\
		sysctl_activatePeripheral(DEV_CFG(dev)->base); 									\
		pinmux_tiva_arrayCfg(DEV_CFG(dev)->pinctrl_list, DEV_CFG(dev)->pinctrl_list_size);\
		cfg = i2c_map_dt_bitrate(DT_INST_PROP(n, clock_frequency)); 					\
		err = i2c_tiva_configure(dev, cfg | I2C_MODE_MASTER);							\
		if (err) return err;															\
		I2CMasterIntEnable(DEV_CFG(dev)->base);											\
		I2CMasterEnable(DEV_CFG(dev)->base);											\
		return 0;																		\
	};																					\
	DEVICE_DT_INST_DEFINE(n,															\
		i2c_## n ##_tiva_init,															\
		NULL,																			\
		&i2c_## n ##_tiva_runtime,														\
		&i2c_## n ##_tiva_cfg,															\
		POST_KERNEL, 50,																\
		&i2c_tiva_driver_api);															\


DT_INST_FOREACH_STATUS_OKAY(TIVA_I2C_DEVICE)
