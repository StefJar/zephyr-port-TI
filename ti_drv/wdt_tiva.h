/*
 * Copyright (c) 2022 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TI_DRV_WDT_TIVA_H_
#define TI_DRV_WDT_TIVA_H_

#include <kernel.h>

typedef void (* wdt_tiva_isr_funcCB_pfkt)(void);

void wdt_tiva_installISRcb(const struct device *dev, wdt_tiva_isr_funcCB_pfkt isrCB);


#endif /* TI_DRV_WDT_TIVA_H_ */
