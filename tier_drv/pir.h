/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SRC_DRV_PIR_H_
#define SRC_DRV_PIR_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct pir_driver_api {
	int (* getAlarmPinState)(const struct device *dev);
};

static inline int pir_getAlarmPinState(const struct device *dev) {
	const struct pir_driver_api *api = (const struct pir_driver_api *)dev->api;
	return api->getAlarmPinState(dev);
}

#endif /* SRC_DRV_PIR_H_ */
