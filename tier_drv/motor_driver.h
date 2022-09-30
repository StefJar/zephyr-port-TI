/*
 * Copyright (c) 2021 Kamil Sroka, TIER GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_MOTOR_DRIVER_H_
#define SRC_DRV_MOTOR_DRIVER_H_

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

typedef enum motor_driver_dir {
	MOTOR_DRIVER_FORWARD,
	MOTOR_DRIVER_REVERSE
} motor_driver_dir_t;

__subsystem struct motor_driver_driver_api {
	int (* run)(const struct device *dev, uint8_t power, motor_driver_dir_t direction);
	int (* idle)(const struct device *dev);
	int (* stop)(const struct device *dev);
};

/**
 * @brief Run motor with requested power and direction.
 *
 * @param dev Pointer to motor_driver compatible device.
 * @param power Value in range 0-100 determining percentage of max power to use.
 * @param direction Direction in which motor should turn.
 *
 * @return int - 0 in case of no error. Device specific error code otherwise.
 *
 */
static inline int motor_driver_run(const struct device *dev, uint8_t power, motor_driver_dir_t direction)
{
	return ((const struct motor_driver_driver_api *)dev->api)->run(dev, power, direction);
}

/**
 * @brief Put motor in idle (coast) mode.
 *
 * @param dev Pointer to motor_driver compatible device.
 *
 * @return int - 0 in case of no error. Device specific error code otherwise.
 */
static inline int motor_driver_idle(const struct device *dev)
{
	return ((const struct motor_driver_driver_api *)dev->api)->idle(dev);
}

/**
 * @brief Put motor in stop (active breaking) mode.
 *
 * @param dev Pointer to motor_driver compatible device.
 * @return int - 0 in case of no error. Device specific error code otherwise.
 */
static inline int motor_driver_stop(const struct device *dev)
{
	return ((const struct motor_driver_driver_api *)dev->api)->stop(dev);
}

#endif /* SRC_DRV_MOTOR_DRIVER_H_ */
