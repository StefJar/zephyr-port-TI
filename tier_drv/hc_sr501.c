/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hc_sr501

#include <stddef.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <devicetree.h>

#include "pir.h"

struct hc_sr501_config {
	const struct gpio_dt_spec detectSignalPin;
};

struct hc_sr501_runtime {

};

#define DEV_CFG(dev)	                    \
	((const struct hc_sr501_config *const)	\
	(dev)->config)

#define DEV_DATA(dev)					\
	((struct hc_sr501_runtime *const)   \
	(dev)->data)

static int hc_sr501_dev_getAlarmPinState(const struct device *dev){
	return gpio_pin_get_dt(&DEV_CFG(dev)->detectSignalPin);
}

static int hc_sr501_dev_init(const struct device *dev){
	gpio_pin_configure_dt(&(DEV_CFG(dev)->detectSignalPin), GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&(DEV_CFG(dev)->detectSignalPin), GPIO_INT_EDGE_BOTH);

	return 0;
}

static const struct pir_driver_api hc_sr501_driver_api = {
	.getAlarmPinState = hc_sr501_dev_getAlarmPinState,
};

#define HC_SR501_DEVICE(n)													\
	static const struct hc_sr501_config hc_sr501_## n ##_cfg={				\
		.detectSignalPin = GPIO_DT_SPEC_INST_GET (n ,detectsignal_gpios), 	\
	};																		\
	static struct hc_sr501_runtime hc_sr501_## n ##_runtime = {				\
	};																		\
	DEVICE_DT_INST_DEFINE(n,												\
		hc_sr501_dev_init,													\
		NULL,																\
		&hc_sr501_## n ##_runtime,											\
		&hc_sr501_## n ##_cfg,												\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,					\
		&hc_sr501_driver_api); 												\


DT_INST_FOREACH_STATUS_OKAY(HC_SR501_DEVICE)
