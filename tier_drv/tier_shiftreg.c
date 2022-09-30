/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT tier_shiftreg

#include <stddef.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <sys/util.h>

#include "shiftreg.h"

struct shiftreg_config {
	const struct gpio_dt_spec dinPin;
	const struct gpio_dt_spec srclkPin;
	const struct gpio_dt_spec latchPin;
	const struct gpio_dt_spec rclkPin;
	const struct gpio_dt_spec oePin;
	const unsigned int outputs;
};

struct shiftreg_runtime {
	uint8_t * pBitArray; // bitarray mapped to outputs/8 number of bytes
};

#define DEV_CFG(dev)                           \
	((const struct shiftreg_config *const)     \
	(dev)->config)

#define DEV_DATA(dev)					\
	((struct shiftreg_runtime *const)   \
	(dev)->data)

static unsigned int getBit(const uint8_t * pB, const unsigned int bitNum) {
	const unsigned indx = bitNum / 8; // byte number = bitNumber / 8
	const unsigned mask = BIT(bitNum - (indx * 8));
	return ((pB[indx] & mask) == mask) ? 1 : 0;
}

static void setBit(uint8_t * pB, const unsigned int bitNum, const unsigned int val) {
	const unsigned indx = bitNum / 8; // byte number = bitNumber / 8
	const unsigned mask = BIT(bitNum - (indx * 8));
	if (val) {
		pB[indx] |= mask;
	} else {
		pB[indx] &= ~mask;
	}
}

static void update(const struct device *dev) {
	gpio_pin_set_dt(&(DEV_CFG(dev)->srclkPin), 0);
	gpio_pin_set_dt(&(DEV_CFG(dev)->dinPin), 0);
	gpio_pin_set_dt(&(DEV_CFG(dev)->latchPin), 0);
	gpio_pin_set_dt(&(DEV_CFG(dev)->oePin), 0);

	// update the shift register starting with the last bit and
	// ending with the first bit

	for (int bitN = DEV_CFG(dev)->outputs - 1; bitN >= 0; bitN--) {
		const unsigned int pinVal = getBit(DEV_DATA(dev)->pBitArray, (unsigned int)bitN);
		// set value
		gpio_pin_set_dt(&(DEV_CFG(dev)->dinPin),pinVal);
		// gen rising edge
		gpio_pin_set_dt(&(DEV_CFG(dev)->srclkPin), 0);
		gpio_pin_set_dt(&(DEV_CFG(dev)->srclkPin), 1);
	}

	// all outputs have been written
	gpio_pin_set_dt(&(DEV_CFG(dev)->latchPin), 1);

	//set data line to low
	gpio_pin_set_dt(&(DEV_CFG(dev)->dinPin), 0);

	// enable output
	gpio_pin_set_dt(&(DEV_CFG(dev)->oePin), 1);

	// reset latch pin
	gpio_pin_set_dt(&(DEV_CFG(dev)->latchPin), 0);
}

static int shiftreg_dev_init(const struct device *dev){
	gpio_pin_configure_dt(&(DEV_CFG(dev)->dinPin), GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&(DEV_CFG(dev)->srclkPin), GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&(DEV_CFG(dev)->latchPin), GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&(DEV_CFG(dev)->rclkPin), GPIO_OUTPUT_HIGH);
	gpio_pin_configure_dt(&(DEV_CFG(dev)->oePin), GPIO_OUTPUT_HIGH);

	for (unsigned int i = 0; i < ROUND_UP(DEV_CFG(dev)->outputs / 8, 1);i++) {
		DEV_DATA(dev)->pBitArray[i] = 0;
	}

	update(dev);
	return 0;
}

unsigned int shiftreg_dev_getOutput(const struct device *dev, const unsigned int idx) {
	return getBit(DEV_DATA(dev)->pBitArray, idx);
}

void shiftreg_dev_setOutput(const struct device *dev, const unsigned int idx, const unsigned int val) {
	setBit(DEV_DATA(dev)->pBitArray, idx, val);
	update(dev);
}

unsigned int shiftreg_dev_getAmount(const struct device *dev) {
	return DEV_CFG(dev)->outputs;
}

static const struct shiftreg_driver_api shiftreg_driver_api = {
	.getOutput = shiftreg_dev_getOutput,
	.setOutput = shiftreg_dev_setOutput,
	.getAmount = shiftreg_dev_getAmount,
};

#define SHIFTREG_DEVICE(n)																	\
	static const struct shiftreg_config shiftreg_## n ##_cfg={								\
		.dinPin = GPIO_DT_SPEC_INST_GET (n ,din_gpios), 									\
		.srclkPin = GPIO_DT_SPEC_INST_GET (n ,srclk_gpios), 								\
		.latchPin = GPIO_DT_SPEC_INST_GET (n ,latch_gpios), 								\
		.rclkPin = GPIO_DT_SPEC_INST_GET (n ,rclk_gpios), 									\
		.oePin = GPIO_DT_SPEC_INST_GET (n ,oe_gpios), 										\
		.outputs = DT_INST_PROP(n ,outputs), 												\
	};																						\
	static uint8_t shiftreg_## n ##_outputs [ROUND_UP(DT_INST_PROP(n ,outputs) / 8, 1)];	\
	static struct shiftreg_runtime shiftreg_## n ##_runtime = {								\
		.pBitArray = shiftreg_## n ##_outputs,												\
	};																						\
	DEVICE_DT_INST_DEFINE(n,																\
		shiftreg_dev_init,																	\
		NULL,																				\
		&shiftreg_## n ##_runtime,															\
		&shiftreg_## n ##_cfg,																\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,									\
		&shiftreg_driver_api); 																\


DT_INST_FOREACH_STATUS_OKAY(SHIFTREG_DEVICE)
