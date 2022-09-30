/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_SMOKEDETECTOR_FLYINGFISH_H_
#define SRC_DRV_SMOKEDETECTOR_FLYINGFISH_H_


#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct smokeDetector_flyingFish_driver_api {
	int (* getCurrentValue)(const struct device *dev, unsigned int * val);
	int (* getAlarmPinState)(const struct device *dev);
};

static inline int smokeDetector_flyingFish_getCurrentValue(const struct device *dev, unsigned int * val) {
	const struct smokeDetector_flyingFish_driver_api *api = (const struct smokeDetector_flyingFish_driver_api *)dev->api;
	return api->getCurrentValue(dev, val);
}

static inline int smokeDetector_flyingFish_getAlarmPinState(const struct device *dev) {
	const struct smokeDetector_flyingFish_driver_api *api = (const struct smokeDetector_flyingFish_driver_api *)dev->api;
	return api->getAlarmPinState(dev);
}


#endif /* SRC_DRV_SMOKEDETECTOR_FLYINGFISH_H_ */
