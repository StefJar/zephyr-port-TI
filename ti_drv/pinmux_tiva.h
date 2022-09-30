/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_PINMUX_TIVA_H_
#define SRC_DRV_PINMUX_TIVA_H_

#include "ti_tiva_dt.h"

void pinmux_tiva_cfg(const ti_tiva_gpio_pinctrl_t * cfg);
void pinmux_tiva_arrayCfg(const ti_tiva_gpio_pinctrl_t * cfg, const unsigned int N);

#endif /* SRC_DRV_PINMUX_TIVA_H_ */
