/*
 * Copyright (c) 2021 Kamil Sroka, TIER GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT ti_drv8830drcr

#include <stddef.h>

#include <sys/__assert.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/util.h>

#include "motor_driver.h"

#define MIN_VSET 0x06
#define MAX_VSET 0x3f

#define CALC_VSET(power) CLAMP(power * (MAX_VSET - MIN_VSET) / 100 + MIN_VSET, MIN_VSET, MAX_VSET)

typedef enum drv8830drcr_regs {
	drv8830drcr_reg_control = 0x00,
	drv8830drcr_reg_fault = 0x01,
} drv8830drcr_reg_t;

struct drv8830drcr_config {
	const struct i2c_dt_spec i2c_device;
};

#define DEV_CFG(dev)							\
	((const struct drv8830drcr_config *const)	\
	(dev)->config)

#define DEV_DATA(dev)							\
	((struct drv8830drcr_runtime *const)		\
	(dev)->data)

static int drv8830drcr_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *reg_val) {
	return i2c_reg_read_byte_dt(&DEV_CFG(dev)->i2c_device, reg_addr, reg_val);
}

static int drv8830drcr_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t reg_val) {
	return i2c_reg_write_byte_dt(&DEV_CFG(dev)->i2c_device, reg_addr, reg_val);
}

static int drv8830drcr_dev_stop(const struct device *dev) {
	return drv8830drcr_reg_write(dev, drv8830drcr_reg_control, BIT(1) | BIT(0));
}

static int drv8830drcr_dev_idle(const struct device *dev) {
	return drv8830drcr_reg_write(dev, drv8830drcr_reg_control, 0);
}

static int drv8830drcr_dev_init(const struct device *dev) {
	return drv8830drcr_dev_idle(dev);
}

static int drv8830drcr_dev_run(const struct device *dev, uint8_t power, motor_driver_dir_t direction) {
	return drv8830drcr_reg_write(
		dev,
		drv8830drcr_reg_control,
		CALC_VSET(power) << 2 | (direction == MOTOR_DRIVER_FORWARD ? BIT(0) : BIT(1))
	);
}

static const struct motor_driver_driver_api drv8830drcr_driver_api = {
	.run = drv8830drcr_dev_run,
	.idle = drv8830drcr_dev_idle,
	.stop = drv8830drcr_dev_stop,
};

#define MOTOR_DRIVER_DEVICE(n)											\
	static const struct drv8830drcr_config drv8830drcr_## n ##_cfg={	\
		.i2c_device = I2C_DT_SPEC_INST_GET(n),							\
	};																	\
	DEVICE_DT_INST_DEFINE(n,											\
		drv8830drcr_dev_init,											\
		NULL,															\
		NULL,															\
		&drv8830drcr_## n ##_cfg,										\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,				\
		&drv8830drcr_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOTOR_DRIVER_DEVICE)
