/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_SYSCTL_TIVA_H_
#define SRC_DRV_SYSCTL_TIVA_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct sysctl_reset_info {
	uint8_t hardwareSystemServiceRequest : 1;
	uint8_t hibernateReset : 1;
	uint8_t watchdog1Reset : 1;
	uint8_t softwareReset : 1;
	uint8_t watchdog0Reset : 1;
	uint8_t brownOutReset : 1;
	uint8_t powerOnReset : 1;
	uint8_t externalReset : 1;
} sysctl_reset_info_t;

bool sysctl_getActiveState(const uint32_t pAddr);
bool sysctl_activatePeripheral(const uint32_t pAddr);
void sysctl_deactivatePeripheral(const uint32_t pAddr);
void sysctl_getResetState(sysctl_reset_info_t * ri);
void sysctl_clearResetState(void);

#endif /* SRC_DRV_SYSCTL_TIVA_H_ */
