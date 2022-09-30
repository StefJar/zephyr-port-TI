/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_TIVA_ADC_TIVA_H_
#define SRC_DRV_TIVA_ADC_TIVA_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>


__subsystem struct tiva_adc_driver_api {
	int (* setupSeqence)(const struct device *dev, const unsigned int seqN, const unsigned int cha);
	void (* disableSequence)(const struct device *dev, const unsigned int seqN);
	int (* sample)(const struct device *dev, const unsigned int seqN, unsigned int * adcVal);
};

static inline int tiva_adc_setupSeqence(const struct device *dev, const unsigned int seqN, const unsigned int cha) {
	const struct tiva_adc_driver_api *api = (const struct tiva_adc_driver_api *)dev->api;
	return api->setupSeqence(dev, seqN, cha);
}

static inline void tiva_adc_disableSequence(const struct device *dev, const unsigned int seqN) {
	const struct tiva_adc_driver_api *api = (const struct tiva_adc_driver_api *)dev->api;
	api->disableSequence(dev, seqN);
}

static inline int tiva_adc_sample(const struct device *dev, const unsigned int seqN, unsigned int * adcVal) {
	const struct tiva_adc_driver_api *api = (const struct tiva_adc_driver_api *)dev->api;
	return api->sample(dev, seqN, adcVal);
}

#endif /* SRC_DRV_TIVA_ADC_TIVA_H_ */
