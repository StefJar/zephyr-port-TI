/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_uart

#include <kernel.h>
#include <arch/cpu.h>
#include <sys/__assert.h>
#include <soc.h>
#include <init.h>
#include <drivers/uart.h>
#include <linker/sections.h>
#include <devicetree.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"
#include "pinmux_tiva.h"
#include "udma_tiva.h"

struct uart_tiva_config {
	const uint32_t base;
	const uint32_t baudrate;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	uart_irq_config_func_t	irq_config_func;
	const uint32_t dma_tx_channel;
	const uint32_t dma_tx_assignCfg;
};

struct uart_tiva_runtime {
	struct k_mutex gm;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;	/**< Callback function pointer */
	void *cb_data;	/**< Callback function arg */
#endif
	struct {
		struct k_sem done;
	} tx;
};

#define DEV_CFG(dev) ((const struct uart_tiva_config *const)(dev)->config)

#define DEV_DATA(dev) ((struct uart_tiva_runtime *const)(dev)->data)

static void uart_tiva_isr(const struct device *dev) {
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	struct uart_tiva_runtime *const dev_data = DEV_DATA(dev);
	const uint32_t base = cfg->base;

	const uint32_t UIstatus = UARTIntStatus(base, true);
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
#endif
	UARTIntClear(base, UIstatus);

	if(false == udma_channelIsActive(cfg->dma_tx_channel)) {
		k_sem_give(&dev_data->tx.done);
	}
}

static int uart_tiva_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	if (!UARTCharsAvail(base)) {
		return -1;
	}

	*c = UARTCharGetNonBlocking(base);

	return 0;
}

static void uart_tiva_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTCharPut(base, c);

	// wait till end of tansmission
	while (true == UARTBusy(base)) {
	}

}

static int uart_tiva_err_check(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	const uint32_t flags = UARTRxErrorGet(base);

	int error = (flags & UART_RXERROR_FRAMING ? UART_ERROR_FRAMING : 0) |
		    (flags & UART_RXERROR_PARITY ? UART_ERROR_PARITY : 0) |
		    (flags & UART_RXERROR_BREAK ? UART_BREAK : 0) |
		    (flags & UART_RXERROR_OVERRUN ? UART_ERROR_OVERRUN : 0);

	UARTRxErrorClear(base);

	return error;
}

static int uart_tiva_configure(const struct device *dev, const struct uart_config *cfg) {
	const struct uart_tiva_config * const devCfg = DEV_CFG(dev);
	const uint32_t base = devCfg->base;
	uint32_t tiCfg = 0;

	switch(cfg->parity) {
		case UART_CFG_PARITY_NONE:
			tiCfg = UART_CONFIG_PAR_NONE;
			break;
		case UART_CFG_PARITY_ODD:
			tiCfg = UART_CONFIG_PAR_ODD;
			break;
		case UART_CFG_PARITY_EVEN:
			tiCfg = UART_CONFIG_PAR_EVEN;
			break;
		case UART_CFG_PARITY_MARK:
		case UART_CFG_PARITY_SPACE:
		default:
			return -ENOTSUP;
			break;
	};

	switch(cfg->stop_bits) {
		case UART_CFG_STOP_BITS_1:
			tiCfg |= UART_CONFIG_STOP_ONE;
			break;
		case UART_CFG_STOP_BITS_2:
			tiCfg |= UART_CONFIG_STOP_TWO;
			break;
		case UART_CFG_STOP_BITS_0_5:
		case UART_CFG_STOP_BITS_1_5:
		default:
			return -ENOTSUP;
			break;
	};

	UART9BitDisable(base);
	switch(cfg->data_bits) {
		case UART_CFG_DATA_BITS_5:
			tiCfg |= UART_CONFIG_WLEN_5;
			break;
		case UART_CFG_DATA_BITS_6:
			tiCfg |= UART_CONFIG_WLEN_6;
			break;
		case UART_CFG_DATA_BITS_7:
			tiCfg |= UART_CONFIG_WLEN_7;
			break;
		case UART_CFG_DATA_BITS_8:
			tiCfg |= UART_CONFIG_WLEN_8;
			break;
		case UART_CFG_DATA_BITS_9:
			// ToDo: check how 9bit mode is right fully setup
			// tiCfg |= UART_CONFIG_WLEN_8;
			// UART9BitEnable(base);
			// break;
		default:
			return -ENOTSUP;
			break;
	};

	UARTEnable(base);
	UARTConfigSetExpClk(base, SysCtlClockGet(), cfg->baudrate, tiCfg);
	/* Clear all UART interrupts */
	UARTIntClear(base,
		UART_INT_OE | UART_INT_BE | UART_INT_PE |
		UART_INT_FE | UART_INT_RT | UART_INT_TX |
		UART_INT_RX | UART_INT_CTS);

	switch(cfg->flow_ctrl) {
		case UART_CFG_FLOW_CTRL_NONE:
			UARTFlowControlSet(base,UART_FLOWCONTROL_NONE);
			break;
		case UART_CFG_FLOW_CTRL_RTS_CTS:
			UARTFlowControlSet(base, UART_FLOWCONTROL_RX | UART_FLOWCONTROL_TX);
			break;
		case UART_CFG_FLOW_CTRL_DTR_DSR:
		default:
			return -ENOTSUP;
	};

	UARTEnable(base);

	return 0;
}

static int uart_tiva_config_get(const struct device *dev, struct uart_config *cfg){
	const struct uart_tiva_config * const devCfg = DEV_CFG(dev);
	const uint32_t base = devCfg->base;

	uint32_t br, tiCfg;
	UARTConfigGetExpClk(base, SysCtlClockGet(), &br, &tiCfg);

	cfg->baudrate = br;

	switch(tiCfg & UART_CONFIG_PAR_MASK) {
		case UART_CONFIG_PAR_NONE:
			cfg->parity = UART_CFG_PARITY_NONE;
			break;
		case UART_CONFIG_PAR_ODD:
			cfg->parity = UART_CFG_PARITY_ODD;
			break;
		case UART_CONFIG_PAR_EVEN:
			cfg->parity = UART_CFG_PARITY_EVEN;
			break;
		case UART_CFG_PARITY_MARK:
		case UART_CFG_PARITY_SPACE:
		default:
			return -ENOTSUP;
			break;
	};

	switch(tiCfg &  UART_CONFIG_STOP_MASK) {
		case UART_CONFIG_STOP_ONE:
			cfg->stop_bits = UART_CFG_STOP_BITS_1;
			break;
		case UART_CONFIG_STOP_TWO:
			cfg->stop_bits = UART_CFG_STOP_BITS_2;
			break;
		default:
			return -ENOTSUP;
			break;
	};

	switch(tiCfg &  UART_CONFIG_WLEN_MASK) {
		case UART_CONFIG_WLEN_5:
			cfg->data_bits = UART_CFG_DATA_BITS_5;
			break;
		case UART_CONFIG_WLEN_6:
			cfg->data_bits = UART_CFG_DATA_BITS_6;
			break;
		case UART_CONFIG_WLEN_7:
			cfg->data_bits = UART_CFG_DATA_BITS_7;
			break;
		case UART_CONFIG_WLEN_8:
			cfg->data_bits = UART_CFG_DATA_BITS_8;
			break;
		// ToDo: check how to check the 9bit mode
		default:
			return -ENOTSUP;
			break;
	};

	switch(UARTFlowControlGet(base)) {
		case UART_FLOWCONTROL_NONE:
			cfg->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
			break;
		case (UART_FLOWCONTROL_RX | UART_FLOWCONTROL_TX):
			cfg->flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
			break;
		default:
			return -ENOTSUP;
	};
	return 0;
}

static int uart_tiva_init(const struct device *dev) {
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;
	struct uart_tiva_runtime *const data = DEV_DATA(dev);

	pinmux_tiva_arrayCfg(cfg->pinctrl_list, cfg->pinctrl_list_size);

	// switch uart on
	sysctl_activatePeripheral(base);

	DEV_CFG(dev)->irq_config_func(dev);

	/*
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	UARTTxIntModeSet(base, UART_TXINT_MODE_EOT);
#endif

	UARTFIFODisable(base);
*/

	UARTFIFOLevelSet(base, UART_FIFO_TX4_8, UART_FIFO_RX1_8);
    UARTDMAEnable(base, UART_DMA_TX);

	const struct uart_config uart_cfg = {
			.baudrate = cfg->baudrate,
			.parity = UART_CFG_PARITY_NONE,
			.stop_bits = UART_CFG_STOP_BITS_1,
			.data_bits = UART_CFG_DATA_BITS_8,
			.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
		};

	uart_tiva_configure(dev, &uart_cfg);

	k_sem_init(&data->tx.done, 0, 1);
	k_mutex_init(&data->gm);

	udma_channelAssign(cfg->dma_tx_assignCfg);
	udma_channelSetupStaticCfg(cfg->dma_tx_channel, true, false, true);
	return 0;
}

void uart_tiva_blockSend(const struct device *dev, uint8_t * const src, const uint32_t N) {
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;
	struct uart_tiva_runtime *const data = DEV_DATA(dev);

	k_mutex_lock(&data->gm, K_FOREVER);

	udma_channelSetupDyn(cfg->dma_tx_channel, src, (void *)(base + UART_O_DR), N);
	udma_channelStartTransfer(cfg->dma_tx_channel);
	k_sem_take(&data->tx.done, K_FOREVER);
	k_mutex_unlock(&data->gm);
}

#if CONFIG_UART_INTERRUPT_DRIVEN

static int uart_tiva_fifo_fill(const struct device *dev, const uint8_t *buf, int len) {
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;
	int n = 0;

	while (n < len) {
		if (false == UARTCharPutNonBlocking(base, buf[n])) {
			break;
		}
		n++;
	}

	return n;
}

static int uart_tiva_fifo_read(const struct device *dev, uint8_t *buf, const int len) {
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	int n =0 ;
	while((true == UARTCharsAvail(base)) && (n < len)) {
		buf[n] = (uint8_t) UARTCharGet(base);
		n++;
	}

	return n;
}

static void uart_tiva_irq_tx_enable(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTIntEnable(base, UART_INT_TX);
}

static void uart_tiva_irq_tx_disable(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTIntDisable(base, UART_INT_TX);
}

static int uart_tiva_irq_tx_ready(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	return UARTSpaceAvail(base) ? 1 : 0;
}

static void uart_tiva_irq_rx_enable(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTIntEnable(base, UART_INT_RX | UART_INT_RT);
}

static void uart_tiva_irq_rx_disable(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTIntDisable(base, UART_INT_RX | UART_INT_RT);
}

static int uart_tiva_irq_rx_ready(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	return UARTCharsAvail(base) ? 1 : 0;
}

static void uart_tiva_irq_err_enable(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTIntEnable(base, UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
}

static void uart_tiva_irq_err_disable(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	UARTIntDisable(base, UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);
}

static int uart_tiva_irq_is_pending(const struct device *dev)
{
	const struct uart_tiva_config * const cfg = DEV_CFG(dev);
	const uint32_t base = cfg->base;

	const uint32_t s = UARTIntStatus(base, true);
	return s & (UART_INT_TX | UART_INT_RX) ? 1 : 0;
}

static int uart_tiva_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static void uart_tiva_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_tiva_runtime *const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_tiva_driver_api = {
	.poll_in = uart_tiva_poll_in,
	.poll_out = uart_tiva_poll_out,
	.err_check = uart_tiva_err_check,
	.configure=uart_tiva_configure,
	.config_get=uart_tiva_config_get,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

	.fifo_fill = uart_tiva_fifo_fill,
	.fifo_read = uart_tiva_fifo_read,
	.irq_tx_enable = uart_tiva_irq_tx_enable,
	.irq_tx_disable = uart_tiva_irq_tx_disable,
	.irq_tx_ready = uart_tiva_irq_tx_ready,
	.irq_rx_enable = uart_tiva_irq_rx_enable,
	.irq_rx_disable = uart_tiva_irq_rx_disable,
	.irq_rx_ready = uart_tiva_irq_rx_ready,
	.irq_err_enable = uart_tiva_irq_err_enable,
	.irq_err_disable = uart_tiva_irq_err_disable,
	.irq_is_pending = uart_tiva_irq_is_pending,
	.irq_update = uart_tiva_irq_update,
	.irq_callback_set = uart_tiva_irq_callback_set,

#endif
};

#define TIVA_UART_DEVICE(n)																	\
	static void uart_## n ##_tiva_config_func(const struct device *dev)	{ 					\
		const struct uart_tiva_config * const cfg = DEV_CFG(dev);							\
		const uint32_t base = cfg->base; 													\
		IRQ_CONNECT(DT_INST_IRQN(n),														\
			DT_INST_IRQ(n, priority),														\
			uart_tiva_isr,																	\
			DEVICE_DT_INST_GET(n), 0);														\
		UARTIntClear(base, UART_INT_RX | UART_INT_RT); 										\
		irq_enable(DT_INST_IRQN(n));														\
	} 																						\
	static const ti_tiva_gpio_pinctrl_t uart_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct uart_tiva_config uart_## n ##_tiva_cfg={							\
		.base = DT_INST_REG_ADDR(n),			 											\
		.irq_config_func = uart_## n ##_tiva_config_func,									\
		.baudrate = DT_INST_PROP(n, current_speed),											\
		.pinctrl_list = uart_pins_##n,														\
		.pinctrl_list_size = ARRAY_SIZE(uart_pins_##n),										\
		.dma_tx_channel = DT_INST_PHA_BY_IDX(n, dmas, 0, channel),							\
		.dma_tx_assignCfg = DT_INST_PHA_BY_IDX(n, dmas, 0, config),							\
	};																						\
	static struct uart_tiva_runtime uart_## n ##_tiva_runtime;								\
	DEVICE_DT_INST_DEFINE(n,																\
		uart_tiva_init,																		\
		NULL,																				\
		&uart_## n ##_tiva_runtime,															\
		&uart_## n ##_tiva_cfg,																\
		POST_KERNEL, 55,																	\
		&uart_tiva_driver_api); 															\

DT_INST_FOREACH_STATUS_OKAY(TIVA_UART_DEVICE)


// .dma_tx_channel = DT_INST_PROP_BY_IDX(n, dmas, 0),
