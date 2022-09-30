/*
 * Copyright (c) 2021 Kamil Sroka, TIER GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tier_gpio_light_controller

#include <stddef.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <sys/util.h>

#include "light_controller.h"

struct light_controller_config {
	const struct gpio_dt_spec light_pin;
};

#define DEV_CFG(dev)                           			\
	((const struct light_controller_config *const)		\
	(dev)->config)

static int light_controller_dev_init(const struct device *dev) {
	gpio_pin_configure_dt(&(DEV_CFG(dev)->light_pin), GPIO_OUTPUT_LOW);

	return 0;
}

static void light_controller_dev_set(const struct device *dev, uint8_t val) {
	gpio_pin_set_dt(&(DEV_CFG(dev)->light_pin), val);
}

static const struct light_controller_driver_api light_controller_driver_api = {
	.set = light_controller_dev_set,
};

#define LIGHT_CONTROLLER_DEVICE(n)															\
	static const struct light_controller_config light_controller_## n ##_cfg={				\
		.light_pin = GPIO_DT_SPEC_INST_GET (n ,light_gpios), 								\
	};																						\
	DEVICE_DT_INST_DEFINE(n,																\
		light_controller_dev_init,															\
		NULL,																				\
		NULL,																				\
		&light_controller_## n ##_cfg,														\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,									\
		&light_controller_driver_api);


DT_INST_FOREACH_STATUS_OKAY(LIGHT_CONTROLLER_DEVICE)
