/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_FAN_H_
#define SRC_DRV_FAN_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct fan_driver_api {
	void (* set_switch)(const struct device *dev, const bool on);
	bool (* get_switch)(const struct device *dev);
	void (* initSensingAndStart)(const struct device *dev, const uint32_t NedgesCapture);
	void (* uninitSensing)(const struct device *dev);
	uint32_t (* get_freq)(const struct device *dev);
};

static inline void fan_set_switch(const struct device *dev, const bool on) {
	const struct fan_driver_api *api = (const struct fan_driver_api *)dev->api;
	api->set_switch(dev, on);
}

static inline bool fan_get_switch(const struct device *dev) {
	const struct fan_driver_api *api = (const struct fan_driver_api *)dev->api;
	return api->get_switch(dev);
}

static inline void fan_initSensingAndStart(const struct device *dev, const uint32_t NedgesCapture) {
	const struct fan_driver_api *api = (const struct fan_driver_api *)dev->api;
	api->initSensingAndStart(dev, NedgesCapture);
}

static inline void fan_uninitSensing(const struct device *dev) {
	const struct fan_driver_api *api = (const struct fan_driver_api *)dev->api;
	api->uninitSensing(dev);
}

static inline uint32_t fan_get_freq(const struct device *dev) {
	const struct fan_driver_api *api = (const struct fan_driver_api *)dev->api;
	return api->get_freq(dev);
}

#endif /* SRC_DRV_FAN_H_ */
