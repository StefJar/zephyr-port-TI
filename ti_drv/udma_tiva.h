/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_TIVA_UDMA_TIVA_H_
#define SRC_DRV_TIVA_UDMA_TIVA_H_

#include <stdint.h>
#include <stdbool.h>

void udma_channelAssign(const uint32_t channelAssignValue);
void udma_channelSetupStaticCfg(const uint32_t channel, const bool srcInc, const bool destInc, const bool useBurst);
void udma_channelSetupDyn(const uint32_t channel, void * src, void * dst, const uint32_t N);
void udma_channelStartTransfer(const uint32_t channel);
bool udma_channelIsActive(const uint32_t channel);

#endif /* SRC_DRV_TIVA_UDMA_TIVA_H_ */
