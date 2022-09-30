/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_SHIFTREG_H_
#define SRC_DRV_SHIFTREG_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct shiftreg_driver_api {
	unsigned int (* getOutput)(const struct device *dev, const unsigned int idx);
	void (* setOutput)(const struct device *dev, const unsigned int idx, const unsigned int val);
	unsigned int (* getAmount)(const struct device *dev);
};

static inline unsigned int shiftreg_getOutput(const struct device *dev, const unsigned int idx) {
	const struct shiftreg_driver_api *api = (const struct shiftreg_driver_api *)dev->api;
	return api->getOutput(dev, idx);
}

static inline void shiftreg_setOutput(const struct device *dev, const unsigned int idx, const unsigned int val) {
	const struct shiftreg_driver_api *api = (const struct shiftreg_driver_api *)dev->api;
	api->setOutput(dev, idx, val);
}

static inline unsigned int shiftreg_getAmount(const struct device *dev) {
	const struct shiftreg_driver_api *api = (const struct shiftreg_driver_api *)dev->api;
	return api->getAmount(dev);
}

#endif /* SRC_DRV_SHIFTREG_H_ */
