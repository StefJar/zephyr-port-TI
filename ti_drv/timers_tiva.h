/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_TIVA_TIMERS_TIVA_H_
#define SRC_DRV_TIVA_TIMERS_TIVA_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

typedef enum timers_cfg {
	timers_cfg_timerA = 0,
	timers_cfg_timerB = 1,
	// timers_cfg_timerAB = 2 not implemented yet
} timers_cfg_t;

typedef void (* pfkt_timers_cb_t) (void * param);

__subsystem struct timers_driver_api {
	void (* setIntrCB)(const struct device *dev, const timers_cfg_t timerCfg, const pfkt_timers_cb_t cb, const void * pParam);
	int (* setupInterval)(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t freqHz);
	int (* setupPwm)(const struct device *dev, const timers_cfg_t timerCfg);
	int (* setupPosEdgeCapture)(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t NedgesCapture);
	int (* setupPwmGenViaCapComp)(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t countUpVal);
	void (* start)(const struct device *dev, const timers_cfg_t timerCfg);
	void (* stop)(const struct device *dev, const timers_cfg_t timerCfg);
	uint32_t (* getTimerVal)(const struct device *dev, const timers_cfg_t timerCfg);
	uint32_t (* getMatchVal)(const struct device *dev, const timers_cfg_t timerCfg);
	void (* setMatchVal)(const struct device *dev, const timers_cfg_t timerCfg, uint32_t val);
};

static inline int timers_setupInterval(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t freqHz) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	return api->setupInterval(dev, timerCfg, freqHz);
}

static inline int timers_setupPwm(const struct device *dev, const timers_cfg_t timerCfg) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	return api->setupPwm(dev, timerCfg);
}

static inline int timers_setupPosEdgeCapture(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t NedgesCapture) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	return api->setupPosEdgeCapture(dev, timerCfg, NedgesCapture);
}

static inline int timers_setupPwmGenViaCapComp(const struct device *dev, const timers_cfg_t timerCfg, const uint32_t countUpVal) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	return api->setupPwmGenViaCapComp(dev, timerCfg, countUpVal);
}

static inline void timers_setIntrCB(const struct device *dev, const timers_cfg_t timerCfg, const pfkt_timers_cb_t cb, const void * pParam) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	api->setIntrCB(dev, timerCfg, cb, pParam);
}

static inline void timers_start(const struct device *dev, const timers_cfg_t timerCfg) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	api->start(dev, timerCfg);
}

static inline void timers_stop(const struct device *dev, const timers_cfg_t timerCfg) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	api->stop(dev, timerCfg);
}

static inline uint32_t timers_getTimerVal(const struct device *dev, const timers_cfg_t timerCfg) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	return api->getTimerVal(dev, timerCfg);
}

static inline uint32_t timers_getMatchVal(const struct device *dev, const timers_cfg_t timerCfg) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	return api->getMatchVal(dev, timerCfg);
}

static inline void timers_setMatchVal(const struct device *dev, const timers_cfg_t timerCfg, uint32_t val) {
	const struct timers_driver_api *api = (const struct timers_driver_api *)dev->api;
	api->setMatchVal(dev, timerCfg, val);
}

#endif /* SRC_DRV_TIVA_TIMERS_TIVA_H_ */
