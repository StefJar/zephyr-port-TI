/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SRC_DRV_LED1WIRE_H_
#define SRC_DRV_LED1WIRE_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct led1wire_driver_api {
	unsigned int (* getAmount)(const struct device *dev);
	void (* set8bitRGB)(const struct device *dev, const unsigned int indx, const uint8_t red, const uint8_t green, const uint8_t blue);
	void (* set12bitRGB)(const struct device *dev, const unsigned int indx, const uint16_t red, const uint16_t green, const uint16_t blue);
	void (* update)(const struct device *dev);
};

static inline unsigned int ledstripe_getAmount(const struct device *dev) {
	const struct led1wire_driver_api *api = (const struct led1wire_driver_api *)dev->api;
	return api->getAmount(dev);
}

static inline void ledstripe_set8bitRGB(const struct device *dev, const unsigned int indx, const uint8_t red, const uint8_t green, const uint8_t blue) {
	const struct led1wire_driver_api *api = (const struct led1wire_driver_api *)dev->api;
	api->set8bitRGB(dev, indx, red, green, blue);
}

static inline void ledstripe_set12bitRGB(const struct device *dev, const unsigned int indx, const uint16_t red, const uint16_t green, const uint16_t blue) {
	const struct led1wire_driver_api *api = (const struct led1wire_driver_api *)dev->api;
	api->set12bitRGB(dev, indx, red, green, blue);
}


static inline void ledstripe_update(const struct device *dev) {
	const struct led1wire_driver_api *api = (const struct led1wire_driver_api *)dev->api;
	api->update(dev);
}

#endif /* SRC_DRV_LED1WIRE_H_ */
