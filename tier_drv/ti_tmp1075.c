/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tmp1075

#include <stddef.h>
#include <stdint.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>

#include "tempSensor.h"

typedef enum ti_tmp105_regs {
	ti_tmp105_reg_tmp = 0x00,
	ti_tmp105_reg_cfg = 0x01,
	ti_tmp105_reg_llim = 0x02,
	ti_tmp105_reg_hlim = 0x03,
	ti_tmp105_reg_dieid = 0x0F,
} ti_tmp105_reg_t;

struct ti_tmp105_config {
	const char * i2c_port;
	const uint16_t i2c_addr;
};

struct ti_tmp105_runtime {
	const struct device * i2cDev;
};

#define DEV_CFG(dev)                                     \
	((const struct ti_tmp105_config *const)     \
	(dev)->config)

#define DEV_DATA(dev)					 \
	((struct ti_tmp105_runtime *const)          \
	(dev)->data)

static int tmp105_reg_read(const struct device *dev, uint8_t reg_addr, uint16_t *reg_val) {
	if (i2c_burst_read(DEV_DATA(dev)->i2cDev, DEV_CFG(dev)->i2c_addr, reg_addr, (uint8_t *) reg_val, 2) < 0) {
			return -EIO;
	}
	*reg_val = sys_be16_to_cpu(*reg_val);
	return 0;
}

static int tmp105_reg_write(const struct device *dev, uint8_t reg_addr, uint16_t reg_val) {
	const uint8_t buf[3] = {reg_addr, reg_val >> 8, reg_val & 0xFF};

	return i2c_write(DEV_DATA(dev)->i2cDev, buf, sizeof(buf), DEV_CFG(dev)->i2c_addr);
}

static int ti_tmp105_dev_init(const struct device *dev){
	DEV_DATA(dev)->i2cDev = device_get_binding(DEV_CFG(dev)->i2c_port);
	if (NULL == DEV_DATA(dev)->i2cDev) {
		return -ENODEV;
	}
	return 0;
}

static bool ti_tmp105_setup(const struct device *dev) {
	uint16_t ver = 0;
	int r = tmp105_reg_read(dev, ti_tmp105_reg_dieid, &ver);
	if (r) return false;

	if (ver != 0x7500) {
		return false;
	}
	// continous conversation
	// every 220msec 11 = BIT(14) | BIT(13)
	// alarm at 2 faults 01 = BIT(11)
	// alarm pin goes low 0
	// comperator mode 0
	r = tmp105_reg_write(dev, ti_tmp105_reg_cfg, BIT(14) | BIT(13) | BIT(11));
	return (r) ? false : true;
}

static bool ti_tmp105_get(const struct device *dev, int *v) {
	uint16_t tBin = 0;
	const int r = tmp105_reg_read(dev, ti_tmp105_reg_tmp, &tBin);
	if (r) {
		*v = -1024;
		return false;
	}
	// shift right by 7, multiply by 10 to get 0.1° and divide by 2 to get °C
	*v = (tBin / 128) * 10 / 2;
	return true;
}

static uint16_t getTempVal(const float val) {
	uint8_t msb, lsb;

	if (val >= 0.0) {
		msb = (uint8_t)val;
		lsb = (uint8_t)((val - msb) * 256);
	} else {
		msb = (uint8_t)(256 + val);
		lsb = (uint8_t)((256 + val - msb) * 256);
	}
	return (msb << 8) | lsb;
}

static bool ti_tmp105_setAlarm(const struct device *dev, const float upperLvl, const float lowerLvl) {
	int r = tmp105_reg_write(dev, ti_tmp105_reg_llim, getTempVal(lowerLvl));
	if (r) return false;
	r = tmp105_reg_write(dev, ti_tmp105_reg_hlim, getTempVal(upperLvl));
	if (r) return false;
	return true;
}

static const struct tempSensor_driver_api ti_tmp105_driver_api = {
	.setup = ti_tmp105_setup,
	.get = ti_tmp105_get,
};

#define TI_TMP105_DEVICE(n)											\
	static const struct ti_tmp105_config ti_tmp105_## n ##_cfg={	\
		.i2c_port = DT_INST_BUS_LABEL(n),							\
		.i2c_addr = DT_INST_REG_ADDR(n),							\
	};																\
	static struct ti_tmp105_runtime ti_tmp105_## n ##_runtime;		\
	DEVICE_DT_INST_DEFINE(n,										\
		ti_tmp105_dev_init,											\
		NULL,														\
		&ti_tmp105_## n ##_runtime,									\
		&ti_tmp105_## n ##_cfg,										\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
		&ti_tmp105_driver_api); 									\


DT_INST_FOREACH_STATUS_OKAY(TI_TMP105_DEVICE)
