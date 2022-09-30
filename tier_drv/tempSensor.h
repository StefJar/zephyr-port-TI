/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SRC_DRV_TEMPSENSOR_H_
#define SRC_DRV_TEMPSENSOR_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct tempSensor_driver_api {
	bool (* get)(const struct device *dev, int * temp);
	bool (* setup)(const struct device *dev);
	bool ( *setAlarm) (const struct device *dev, const float upperLvl, const float lowerLvl);
};

static inline bool tempSensor_get(const struct device *dev, int * temp) {
	const struct tempSensor_driver_api *api = (const struct tempSensor_driver_api *)dev->api;
	return api->get(dev, temp);
}

static inline bool tempSensor_setup(const struct device *dev) {
	const struct tempSensor_driver_api *api = (const struct tempSensor_driver_api *)dev->api;
	return api->setup(dev);
}

static inline bool tempSensor_setAlarm(const struct device *dev, const float upperLvl, const float lowerLvl) {
	const struct tempSensor_driver_api *api = (const struct tempSensor_driver_api *)dev->api;
	return api->setAlarm(dev, upperLvl, lowerLvl);
}

#endif /* SRC_DRV_TEMPSENSOR_H_ */
