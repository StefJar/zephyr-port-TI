/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SRC_DRV_IOEXPANDER_H_
#define SRC_DRV_IOEXPANDER_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct ioExpander_driver_api {
	bool (* get)(const struct device *dev, uint8_t * ioPins);
};

static inline bool ioExpander_get(const struct device *dev, uint8_t * ioPins) {
	const struct ioExpander_driver_api *api = (const struct ioExpander_driver_api *)dev->api;
	return api->get(dev, ioPins);
}

#endif /* SRC_DRV_IOEXPANDER_H_ */
