/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_pcf8574

#include <stddef.h>
#include <stdint.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>

#include "ioExpander.h"

struct ti_pcf8574_config {
	const char * i2c_port;
	const uint16_t i2c_addr;
};

struct ti_pcf8574_runtime {
	const struct device * i2cDev;
};

#define DEV_CFG(dev)                                     \
	((const struct ti_pcf8574_config *const)     \
	(dev)->config)

#define DEV_DATA(dev)					 \
	((struct ti_pcf8574_runtime *const)          \
	(dev)->data)

static int ti_pcf8574_dev_init(const struct device *dev){
	DEV_DATA(dev)->i2cDev = device_get_binding(DEV_CFG(dev)->i2c_port);
	if (NULL == DEV_DATA(dev)->i2cDev) {
		return -ENODEV;
	}
	return 0;
}

static bool ti_pcf8574_get(const struct device *dev, uint8_t * ioPins) {
	if (i2c_read(DEV_DATA(dev)->i2cDev, ioPins, 1, DEV_CFG(dev)->i2c_addr) < 0) {
		return false;
	}
	return true;
}

static const struct ioExpander_driver_api ti_pcf8574_driver_api = {
	.get = ti_pcf8574_get,
};

#define TI_PCF8574_DEVICE(n)											\
	static const struct ti_pcf8574_config ti_pcf8574_## n ##_cfg={	\
		.i2c_port = DT_INST_BUS_LABEL(n),							\
		.i2c_addr = DT_INST_REG_ADDR(n),							\
	};																\
	static struct ti_pcf8574_runtime ti_pcf8574_## n ##_runtime;		\
	DEVICE_DT_INST_DEFINE(n,										\
		ti_pcf8574_dev_init,											\
		NULL,														\
		&ti_pcf8574_## n ##_runtime,									\
		&ti_pcf8574_## n ##_cfg,										\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
		&ti_pcf8574_driver_api); 									\


DT_INST_FOREACH_STATUS_OKAY(TI_PCF8574_DEVICE)
