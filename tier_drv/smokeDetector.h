/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SMOKE_DETECTOR_H_
#define SMOKE_DETECTOR_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct smokeDetector_driver_api {
	bool (* checkDev)(const struct device *dev);
	bool (* calibrate)(const struct device *dev);
	bool (* start)(const struct device *dev);
	bool (* edgeISR)(const struct device *dev);
	uint32_t (* getSamplesInFIFO)(const struct device *dev);
	bool (* readSamplesFromFIFO)(const struct device *dev, uint32_t * buffer, const uint32_t Nsamples);
};

static inline bool smokeDetector_checkDrv (const struct device *dev) {
	const struct smokeDetector_driver_api *api = (const struct smokeDetector_driver_api *)dev->api;
	return api->checkDev(dev);
}

static inline bool smokeDetector_calibrate (const struct device *dev) {
	const struct smokeDetector_driver_api *api = (const struct smokeDetector_driver_api *)dev->api;
	return api->calibrate(dev);
}

static inline bool smokeDetector_start (const struct device *dev) {
	const struct smokeDetector_driver_api *api = (const struct smokeDetector_driver_api *)dev->api;
	return api->start(dev);
}

static inline bool smokeDetector_edgeISR (const struct device *dev) {
	const struct smokeDetector_driver_api *api = (const struct smokeDetector_driver_api *)dev->api;
	return api->edgeISR(dev);
}

static inline uint32_t smokeDetector_getSamplesInFIFO (const struct device *dev) {
	const struct smokeDetector_driver_api *api = (const struct smokeDetector_driver_api *)dev->api;
	return api->getSamplesInFIFO(dev);
}

static inline bool smokeDetector_readSamplesFromFIFO(const struct device *dev, uint32_t * buffer, const uint32_t Nsamples) {
	const struct smokeDetector_driver_api *api = (const struct smokeDetector_driver_api *)dev->api;
	return api->readSamplesFromFIFO(dev, buffer, Nsamples);
}

#endif /* SMOKE_DETECTOR_H_ */
