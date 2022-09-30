/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_TIVA_CAN_TIVA_H_
#define SRC_DRV_TIVA_CAN_TIVA_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <stdbool.h>
#include <device.h>

__subsystem struct tiva_can_driver_api {
	void (* activate) (const struct device *dev);
	void (* deactivate) (const struct device *dev);
	void (* setupRxMsg)(const struct device *dev, const unsigned int msgIndx, const uint32_t msgID, const bool msgExtID, const uint32_t msgMask, const uint32_t msgLen, uint8_t * pRxBuffer);
	int (* msgTx) (const struct device *dev, const unsigned int msgIndx, const uint32_t msgID, const bool msgExtID, const uint32_t msgLen, const uint8_t * payload, const k_timeout_t To);
	int (* waitForMsgRx)(const struct device *dev, const unsigned int msgIndx, const k_timeout_t To);
	uint32_t (* getMsgId)(const struct device *dev, const unsigned int msgIndx);
};

static inline void tiva_can_activate (const struct device *dev) {
	const struct tiva_can_driver_api *api = (const struct tiva_can_driver_api *)dev->api;
	api->activate(dev);
}

static inline void tiva_can_deactivate (const struct device *dev) {
	const struct tiva_can_driver_api *api = (const struct tiva_can_driver_api *)dev->api;
	api->deactivate(dev);
}

static inline void tiva_can_setupRxMsg(const struct device *dev, const unsigned int msgIndx, const uint32_t msgID, const bool msgExtID, const uint32_t msgMask, const uint32_t msgLen, uint8_t * pRxBuffer){
	const struct tiva_can_driver_api *api = (const struct tiva_can_driver_api *)dev->api;
	api->setupRxMsg(dev, msgIndx, msgID, msgExtID, msgMask, msgLen, pRxBuffer);
}

static inline int tiva_can_msgTx(const struct device *dev, const unsigned int msgIndx, const uint32_t msgID, const bool msgExtID, const uint32_t msgLen, const uint8_t * payload, const k_timeout_t To) {
	const struct tiva_can_driver_api *api = (const struct tiva_can_driver_api *)dev->api;
	return api->msgTx(dev, msgIndx, msgID, msgExtID, msgLen, payload, To);
}

static inline int tiva_can_waitForMsgRx(const struct device *dev, const unsigned int msgIndx, const k_timeout_t To) {
	const struct tiva_can_driver_api *api = (const struct tiva_can_driver_api *)dev->api;
	return api->waitForMsgRx(dev, msgIndx, To);
}

static inline uint32_t tiva_can_getMsgId(const struct device *dev, const unsigned int msgIndx){
	const struct tiva_can_driver_api *api = (const struct tiva_can_driver_api *)dev->api;
	return api->getMsgId(dev, msgIndx);
}

#endif /* SRC_DRV_TIVA_CAN_TIVA_H_ */
