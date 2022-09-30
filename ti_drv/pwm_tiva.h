/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_TIVA_PWM_TIVA_H_
#define SRC_DRV_TIVA_PWM_TIVA_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>


__subsystem struct tiva_pwm_driver_api {
/*
	int (* setupSeqence)(const struct device *dev, const unsigned int seqN, const unsigned int cha);
	int (* sample)(const struct device *dev, const unsigned int seqN, unsigned int * adcVal);
*/
};
/*
static inline int tiva_pwm_setupSeqence(const struct device *dev, const unsigned int seqN, const unsigned int cha) {
	const struct tiva_pwm_driver_api *api = (const struct tiva_pwm_driver_api *)dev->api;
	return api->setupSeqence(dev, seqN, cha);
}

static inline int tiva_pwm_sample(const struct device *dev, const unsigned int seqN, unsigned int * adcVal) {
	const struct tiva_pwm_driver_api *api = (const struct tiva_pwm_driver_api *)dev->api;
	return api->sample(dev, seqN, adcVal);
}
*/
#endif /* SRC_DRV_TIVA_PWM_TIVA_H_ */
