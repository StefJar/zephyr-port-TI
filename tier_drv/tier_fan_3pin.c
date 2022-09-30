/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tier_fan_3pin

#include <stddef.h>

#include <kernel.h>
#include <drivers/gpio.h>

#include "../ti_drv/cfg_tiva.h"
#include "../ti_drv/ti_tiva_dt.h"
#include "../ti_drv/pinmux_tiva.h"
#include "../ti_drv/timers_tiva.h"

#include "fan.h"

struct fan_3pin_config {
	const struct gpio_dt_spec gpio_drv;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	const char * timer;
	const int subtimerIndx; // 0 - timer a, 1 - timer b
};

struct fan_3pin_runtime {
	uint32_t Tlast;
	uint32_t dT;
	uint32_t Nedges;
	const struct device * timerDev;
};

#define DEV_CFG(dev)						\
	((const struct fan_3pin_config *const)  \
	(dev)->config)

#define DEV_DATA(dev)					\
	((struct fan_3pin_runtime *const)	\
	(dev)->data)

static void configurePin(const struct device *dev) {
	if (DEV_CFG(dev)->pinctrl_list_size <= 0) return;

	ti_tiva_gpio_pinctrl_t cfg = DEV_CFG(dev)->pinctrl_list[0];

	pinmux_tiva_cfg(&cfg);
}

static void unconfigurePin(const struct device *dev) {
	if (DEV_CFG(dev)->pinctrl_list_size <= 0) return;

	ti_tiva_gpio_pinctrl_t cfg = DEV_CFG(dev)->pinctrl_list[0];
	cfg.dirmodeIndx = 0; // configure pin from HW to IN
	cfg.pintypeIndx = 0; // configure STD as input mode
	cfg.strengthIndx = 0; // configure 2mA
	pinmux_tiva_cfg(&cfg);
}

static void fan_timer_cb (void * param) {
	const struct device * dev = (const struct device *)param;
	const uint32_t currT = k_uptime_get_32();
	DEV_DATA(dev)->dT = currT - DEV_DATA(dev)->Tlast;
	DEV_DATA(dev)->Tlast = currT;
}

static int fan_3pin_dev_init(const struct device *dev){
	gpio_pin_configure_dt(&(DEV_CFG(dev)->gpio_drv), GPIO_OUTPUT_LOW);

	DEV_DATA(dev)->timerDev = device_get_binding(DEV_CFG(dev)->timer);
	if (NULL == DEV_DATA(dev)->timerDev) {
		return -1;
	}
	if (DEV_CFG(dev)->pinctrl_list_size >= 2) {
		// this driver only works with 0 or 1 pin
		return -2;
	}

	DEV_DATA(dev)->Tlast = k_uptime_get_32();
	DEV_DATA(dev)->dT = 0;
	DEV_DATA(dev)->Nedges = 0;

	timers_setIntrCB(DEV_DATA(dev)->timerDev,(timers_cfg_t) DEV_CFG(dev)->subtimerIndx, fan_timer_cb, dev);

	return 0;
}

static void fan3pin_dev_set_switch(const struct device *dev, const bool on) {
	gpio_pin_set_dt(&(DEV_CFG(dev)->gpio_drv), true == on ? 1 : 0);
	DEV_DATA(dev)->dT = 0;
	DEV_DATA(dev)->Nedges = 0;
	DEV_DATA(dev)->Tlast = k_uptime_get_32();
}

static bool fan3pin_dev_get_switch(const struct device *dev) {
	return 1 == gpio_pin_get_dt(&(DEV_CFG(dev)->gpio_drv)) ? true : false;
}

static uint32_t fan3pin_dev_get_freq(const struct device *dev) {
	if (DEV_DATA(dev)->dT <= 0) {
		return 0;
	}
	return (DEV_DATA(dev)->Nedges * 1000)/ DEV_DATA(dev)->dT;

}

static void fan3pin_dev_initSensingAndStart(const struct device *dev, const uint32_t NedgesCapture) {
	configurePin(dev);
	DEV_DATA(dev)->Nedges = NedgesCapture;
	timers_setupPosEdgeCapture(DEV_DATA(dev)->timerDev,(timers_cfg_t) DEV_CFG(dev)->subtimerIndx, NedgesCapture);
	timers_start(DEV_DATA(dev)->timerDev,(timers_cfg_t) DEV_CFG(dev)->subtimerIndx);
}

static void fan3pin_dev_uninitSensing(const struct device *dev) {
	timers_stop(DEV_DATA(dev)->timerDev,(timers_cfg_t) DEV_CFG(dev)->subtimerIndx);
	unconfigurePin(dev);
}

static const struct fan_driver_api fan_3pin_driver_api = {
	.set_switch = fan3pin_dev_set_switch,
	.get_switch = fan3pin_dev_get_switch,
	.initSensingAndStart = fan3pin_dev_initSensingAndStart,
	.uninitSensing = fan3pin_dev_uninitSensing,
	.get_freq = fan3pin_dev_get_freq,
};

#define FAN_3PIN_DEVICE(n)										\
	static const ti_tiva_gpio_pinctrl_t fan_3pin_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct fan_3pin_config fan_3pin_## n ##_cfg={	\
		.gpio_drv = GPIO_DT_SPEC_INST_GET(n, drv_gpios),		\
		.pinctrl_list = fan_3pin_pins_##n,						\
		.pinctrl_list_size = ARRAY_SIZE(fan_3pin_pins_##n),		\
		.timer = DT_PROP(DT_INST_PHANDLE(n,timer), label),		\
		.subtimerIndx = DT_INST_PROP(n,subtimer_ENUM_IDX),		\
	};															\
	static struct fan_3pin_runtime fan_3pin_## n ##_runtime;	\
	DEVICE_DT_INST_DEFINE(n,									\
		fan_3pin_dev_init,										\
		NULL,													\
		&fan_3pin_## n ##_runtime,								\
		&fan_3pin_## n ##_cfg,									\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
		&fan_3pin_driver_api); 									\


DT_INST_FOREACH_STATUS_OKAY(FAN_3PIN_DEVICE)
