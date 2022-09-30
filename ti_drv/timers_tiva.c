/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_timers

#include <kernel.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"
#include "pinmux_tiva.h"

#include "timers_tiva.h"

struct timers_tiva_config {
	const uint32_t base;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	const uint32_t prescaler;
};

struct timers_tiva_runtime {
	uint32_t timerCfg;
	uint32_t intrSrc;
	bool timerA_used;
	bool timerB_used;
	bool timerA_enabled;
	bool timerB_enabled;
	pfkt_timers_cb_t cbFunc_A;
	void * cbParam_A;
	pfkt_timers_cb_t cbFunc_B;
	void * cbParam_B;
};

#define DEV_CFG(dev) ((const struct timers_tiva_config *const)(dev)->config)

#define DEV_DATA(dev) ((struct timers_tiva_runtime *const)(dev)->data)

static int timers_tiva_setupInterval(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t freqHz) {
	const uint32_t base = DEV_CFG(dev)->base;
	uint32_t timer;
	uint32_t reenable_timer = 0;

	DEV_DATA(dev)->timerCfg |= TIMER_CFG_SPLIT_PAIR;
	switch(timerCfg) {
		case timers_cfg_timerA:
			timer = TIMER_A;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_A_PERIODIC;
			DEV_DATA(dev)->intrSrc |= TIMER_TIMA_TIMEOUT;
			DEV_DATA(dev)->timerA_used = true;

			if (DEV_DATA(dev)->timerB_enabled)
			{
				reenable_timer = TIMER_B;
			}
			break;
		case timers_cfg_timerB:
			timer = TIMER_B;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_B_PERIODIC;
			DEV_DATA(dev)->intrSrc |= TIMER_TIMB_TIMEOUT;
			DEV_DATA(dev)->timerB_used = true;
			if (DEV_DATA(dev)->timerA_enabled)
			{
				reenable_timer = TIMER_A;
			}
			break;
		default:
			// not supported
			return -1;
	}

	TimerDisable(base, timer);
	TimerConfigure(base, DEV_DATA(dev)->timerCfg);

	/*
	 * Call to TimerConfigure disables both A and B subtimers
	 * so we should reenable other sub timer if it was already enabled.
	 */
	if (0 != reenable_timer)
	{
		TimerEnable(base, reenable_timer);
	}

	TimerPrescaleSet(base, timer, DEV_CFG(dev)->prescaler);

	const uint32_t countVal = SysCtlClockGet() / (freqHz * (TimerPrescaleGet(base, timer) + 1));
	TimerLoadSet(base, timer, countVal);

	TimerIntEnable(base, DEV_DATA(dev)->intrSrc);
	return 0;
}

static int timers_tiva_setupPwm(const struct device *dev, const timers_cfg_t timerCfg) {
	const uint32_t base = DEV_CFG(dev)->base;
	uint32_t timer;
	uint32_t reenable_timer = 0;

	DEV_DATA(dev)->timerCfg |= TIMER_CFG_SPLIT_PAIR;
	switch(timerCfg) {
		case timers_cfg_timerA:
			timer = TIMER_A;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_A_PWM;
			DEV_DATA(dev)->intrSrc |= 0;
			DEV_DATA(dev)->timerA_used = true;

			if (DEV_DATA(dev)->timerB_enabled)
			{
				reenable_timer = TIMER_B;
			}
			break;
		case timers_cfg_timerB:
			timer = TIMER_B;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_B_PWM;
			DEV_DATA(dev)->intrSrc |= 0;
			DEV_DATA(dev)->timerB_used = true;

			if (DEV_DATA(dev)->timerA_enabled)
			{
				reenable_timer = TIMER_A;
			}
			break;
		default:
			// not supported
			return -1;
	}

	TimerDisable(base, timer);
	TimerConfigure(base, DEV_DATA(dev)->timerCfg);

	/*
	 * Call to TimerConfigure disables both A and B subtimers
	 * so we should reenable other sub timer if it was already enabled.
	 */
	if (0 != reenable_timer)
	{
		TimerEnable(base, reenable_timer);
	}

	TimerPrescaleSet(base, timer, DEV_CFG(dev)->prescaler);

	TimerMatchSet(base, timer, 0);
	TimerControlLevel(base, timer, true);

	return 0;
}

static int timers_tiva_setupPosEdgeCapture(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t NedgesCapture) {
	const uint32_t base = DEV_CFG(dev)->base;
	uint32_t timer;
	uint32_t eventTimers;
	uint32_t reenable_timer = 0;

	DEV_DATA(dev)->timerCfg |= TIMER_CFG_SPLIT_PAIR;
	switch(timerCfg) {
		case timers_cfg_timerA:
			timer = TIMER_A;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_A_CAP_COUNT_UP | TIMER_CFG_A_PERIODIC;
			DEV_DATA(dev)->intrSrc |= TIMER_CAPA_MATCH;
			DEV_DATA(dev)->timerA_used = true;

			if (DEV_DATA(dev)->timerB_enabled)
			{
				reenable_timer = TIMER_B;
			}
			break;
		case timers_cfg_timerB:
			timer = TIMER_B;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_B_CAP_COUNT_UP | TIMER_CFG_B_PERIODIC;
			DEV_DATA(dev)->intrSrc |= TIMER_CAPB_MATCH;
			DEV_DATA(dev)->timerB_used = true;

			if (DEV_DATA(dev)->timerA_enabled)
			{
				reenable_timer = TIMER_A;
			}
			break;
		default:
			// not supported
			return -1;
	}

	TimerDisable(base, timer);
	TimerConfigure(base, DEV_DATA(dev)->timerCfg);

	/*
	 * Call to TimerConfigure disables both A and B subtimers
	 * so we should reenable other sub timer if it was already enabled.
	 */
	if (0 != reenable_timer)
	{
		TimerEnable(base, reenable_timer);
	}

	TimerPrescaleSet(base, timer, DEV_CFG(dev)->prescaler);

	eventTimers = 0;
	if (DEV_DATA(dev)->timerCfg & (TIMER_CFG_A_CAP_COUNT_UP | TIMER_CFG_B_CAP_COUNT_UP)) {
		eventTimers = TIMER_BOTH;
	} else {
		if (DEV_DATA(dev)->timerCfg & (TIMER_CFG_A_CAP_COUNT_UP)) {
			eventTimers = TIMER_A;
		}
		if (DEV_DATA(dev)->timerCfg & (TIMER_CFG_B_CAP_COUNT_UP)) {
			eventTimers = TIMER_B;
		}
	}
	if (0 == eventTimers) {
		return -1;
	}

    TimerControlEvent(base, eventTimers, TIMER_EVENT_POS_EDGE);
    TimerMatchSet(base, eventTimers, NedgesCapture);

	TimerIntEnable(base, DEV_DATA(dev)->intrSrc);

	return 0;
}

static int timers_tiva_setupPwmGenViaCapComp(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t countUpVal) {
	const uint32_t base = DEV_CFG(dev)->base;
	uint32_t timer;
	uint32_t reenable_timer = 0;

	DEV_DATA(dev)->timerCfg |= TIMER_CFG_SPLIT_PAIR;
	switch(timerCfg) {
		case timers_cfg_timerA:
			timer = TIMER_A;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_A_PWM;
			DEV_DATA(dev)->intrSrc |= TIMER_CAPA_EVENT;
			DEV_DATA(dev)->timerA_used = true;

			if (DEV_DATA(dev)->timerB_enabled)
			{
				reenable_timer = TIMER_B;
			}
			break;
		case timers_cfg_timerB:
			timer = TIMER_B;
			DEV_DATA(dev)->timerCfg |= TIMER_CFG_B_PWM;
			DEV_DATA(dev)->intrSrc |= TIMER_CAPB_EVENT;
			DEV_DATA(dev)->timerB_used = true;

			if (DEV_DATA(dev)->timerA_enabled)
			{
				reenable_timer = TIMER_A;
			}
			break;
		default:
			// not supported
			return -1;
	}

	TimerDisable(base, timer);
	TimerConfigure(base, DEV_DATA(dev)->timerCfg);

	/*
	 * Call to TimerConfigure disables both A and B subtimers
	 * so we should reenable other sub timer if it was already enabled.
	 */
	if (0 != reenable_timer)
	{
		TimerEnable(base, reenable_timer);
	}

	TimerPrescaleSet(base, timer, DEV_CFG(dev)->prescaler);

	TimerMatchSet(base, timer, 0);
	// we count up to countUpVal
	TimerLoadSet(base, timer, countUpVal);
	// then we reset the counter with the countUpVal value
	TimerUpdateMode(base, timer, TIMER_UP_MATCH_TIMEOUT);

	TimerControlEvent(base, timer, TIMER_EVENT_POS_EDGE);

	TimerIntEnable(base, DEV_DATA(dev)->intrSrc);
	return 0;
}

static void timers_tiva_setIntrCB(const struct device *dev, const timers_cfg_t timerCfg, const pfkt_timers_cb_t cb, const void * pParam) {
	switch(timerCfg) {
		case timers_cfg_timerA:
			DEV_DATA(dev)->cbFunc_A = cb;
			DEV_DATA(dev)->cbParam_A = pParam;
			break;
		case timers_cfg_timerB:
			DEV_DATA(dev)->cbFunc_B = cb;
			DEV_DATA(dev)->cbParam_B = pParam;
			break;
		default:
			// not supported
			return;
	}
}

static void timers_tiva_start(const struct device *dev, const timers_cfg_t timerCfg) {
	const uint32_t base = DEV_CFG(dev)->base;
	switch(timerCfg) {
		case timers_cfg_timerA:
			TimerEnable(base, TIMER_A);
			DEV_DATA(dev)->timerA_enabled = true;
			break;
		case timers_cfg_timerB:
			TimerEnable(base, TIMER_B);
			DEV_DATA(dev)->timerB_enabled = true;
			break;
		default:
			// not supported
			return;
	}
}

static void timers_tiva_stop(const struct device *dev, const timers_cfg_t timerCfg) {
	const uint32_t base = DEV_CFG(dev)->base;
	switch(timerCfg) {
		case timers_cfg_timerA:
			TimerDisable(base, TIMER_A);
			DEV_DATA(dev)->timerA_enabled = false;
			break;
		case timers_cfg_timerB:
			TimerDisable(base, TIMER_B);
			DEV_DATA(dev)->timerB_enabled = false;
			break;
		default:
			// not supported
			return;
	}
}

uint32_t timers_tiva_getTimerValue(const struct device *dev, const timers_cfg_t timerCfg) {
	const uint32_t base = DEV_CFG(dev)->base;
	switch(timerCfg) {
		case timers_cfg_timerA:
			return TimerValueGet(base, TIMER_A);
			break;
		case timers_cfg_timerB:
			return TimerValueGet(base, TIMER_B);
			break;
		default:
			// not supported
			break;
	}
	return 0;
}

uint32_t timers_tiva_getMatchValue(const struct device *dev, const timers_cfg_t timerCfg) {
	const uint32_t base = DEV_CFG(dev)->base;
	switch(timerCfg) {
		case timers_cfg_timerA:
			return TimerMatchGet(base, TIMER_A);
			break;
		case timers_cfg_timerB:
			return TimerMatchGet(base, TIMER_B);
			break;
		default:
			// not supported
			break;
	}
	return 0;
}

static void timers_tiva_setMatchValue(const struct device *dev, const timers_cfg_t timerCfg, uint32_t val) {
	const uint32_t base = DEV_CFG(dev)->base;
	switch(timerCfg) {
		case timers_cfg_timerA:
			TimerMatchSet(base, TIMER_A, val);
			break;
		case timers_cfg_timerB:
			TimerMatchSet(base, TIMER_B, val);
			break;
		default:
			// not supported
			break;
	}
}

static void timers_tiva_A_16_32BitCounter_isr(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct timers_tiva_runtime *data = DEV_DATA(dev);

	// restart timer
	TimerIntClear(base, DEV_DATA(dev)->intrSrc);
	data->cbFunc_A(data->cbParam_A);
}

static void timers_tiva_A_32_64BitCounter_isr(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct timers_tiva_runtime *data = DEV_DATA(dev);
	// restart timer
	TimerIntClear(base, DEV_DATA(dev)->intrSrc);
	data->cbFunc_A(data->cbParam_A);
}

static void timers_tiva_B_16_32BitCounter_isr(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct timers_tiva_runtime *data = DEV_DATA(dev);
	// restart timer
	TimerIntClear(base, DEV_DATA(dev)->intrSrc);
	data->cbFunc_B(data->cbParam_B);
}

static void timers_tiva_B_32_64BitCounter_isr(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct timers_tiva_runtime *data = DEV_DATA(dev);
	// restart timer
	TimerIntClear(base, DEV_DATA(dev)->intrSrc);
	data->cbFunc_B(data->cbParam_B);
}

static const struct timers_driver_api timers_tiva_driver_api = {
	.setIntrCB = timers_tiva_setIntrCB,
	.setupInterval = timers_tiva_setupInterval,
	.setupPwm = timers_tiva_setupPwm,
	.setupPosEdgeCapture = timers_tiva_setupPosEdgeCapture,
	.setupPwmGenViaCapComp = timers_tiva_setupPwmGenViaCapComp,
	.start = timers_tiva_start,
	.stop = timers_tiva_stop,
	.getTimerVal = timers_tiva_getTimerValue,
	.getMatchVal = timers_tiva_getMatchValue,
	.setMatchVal = timers_tiva_setMatchValue,
};

#ifdef CONFIG_ZERO_LATENCY_IRQS
	#define INTR_EXTRA_FLAG IRQ_ZERO_LATENCY
#else
	#define INTR_EXTRA_FLAG 0
#endif

#define TIVA_TIMER_DEVICE(n)																\
	static const ti_tiva_gpio_pinctrl_t timers_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct timers_tiva_config timers_## n ##_tiva_cfg={						\
		.base = DT_INST_REG_ADDR(n),														\
		.pinctrl_list = timers_pins_##n,													\
		.pinctrl_list_size = ARRAY_SIZE(timers_pins_##n),									\
		.prescaler = DT_INST_PROP(n ,prescaler),											\
	};																						\
	static struct timers_tiva_runtime timers_## n ##_tiva_runtime;							\
	static int timers_## n ##_tiva_init(const struct device *dev) {							\
		struct timers_tiva_runtime *data = DEV_DATA(dev); 									\
		data->timerCfg = 0; 																\
		data->intrSrc = 0; 																	\
		data->timerA_used = false; 															\
		data->timerB_used = false; 															\
		data->timerA_enabled = false; 														\
		data->timerB_enabled = false; 														\
		data->cbFunc_A = NULL; 																\
		data->cbParam_A = NULL; 															\
		data->cbFunc_B = NULL; 																\
		data->cbParam_B = NULL; 															\
		pinmux_tiva_arrayCfg(DEV_CFG(dev)->pinctrl_list, DEV_CFG(dev)->pinctrl_list_size);	\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,0, irq), 											\
				DT_INST_IRQ_BY_IDX(n,0, priority), 											\
				timers_tiva_A_16_32BitCounter_isr, DEVICE_DT_INST_GET(n), INTR_EXTRA_FLAG); \
		irq_enable(DT_INST_IRQ_BY_IDX(n,0, irq)); 											\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,1, irq), 											\
				DT_INST_IRQ_BY_IDX(n,1, priority), 											\
				timers_tiva_A_32_64BitCounter_isr, DEVICE_DT_INST_GET(n), INTR_EXTRA_FLAG); \
		irq_enable(DT_INST_IRQ_BY_IDX(n,1, irq)); 											\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,2, irq), 											\
				DT_INST_IRQ_BY_IDX(n,2, priority), 											\
				timers_tiva_B_16_32BitCounter_isr, DEVICE_DT_INST_GET(n), INTR_EXTRA_FLAG); \
		irq_enable(DT_INST_IRQ_BY_IDX(n,2, irq)); 											\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,3, irq), 											\
				DT_INST_IRQ_BY_IDX(n,3, priority), 											\
				timers_tiva_B_32_64BitCounter_isr, DEVICE_DT_INST_GET(n), INTR_EXTRA_FLAG); \
		irq_enable(DT_INST_IRQ_BY_IDX(n,3, irq)); 											\
		sysctl_activatePeripheral(DEV_CFG(dev)->base);										\
		TimerClockSourceSet(DEV_CFG(dev)->base, TIMER_CLOCK_SYSTEM);						\
		return 0;																			\
	};																						\
	DEVICE_DT_INST_DEFINE(n,																\
		timers_## n ##_tiva_init,															\
		NULL,																				\
		&timers_## n ##_tiva_runtime,														\
		&timers_## n ##_tiva_cfg,															\
		POST_KERNEL, 50,																	\
		&timers_tiva_driver_api);															\


DT_INST_FOREACH_STATUS_OKAY(TIVA_TIMER_DEVICE)
