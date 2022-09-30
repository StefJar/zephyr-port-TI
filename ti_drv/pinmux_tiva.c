/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_pinmux

#include <kernel.h>
#include <drivers/pinmux.h>

#include <stdbool.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "pinmux_tiva.h"

/*
 * MCU periphal -> port cntrl (GPIO PCTL) -> alternate output -> mode control (GPIO AFSEL, ADCCTL, DMACTL) -> package io pin
 * MCU periphal <- port cntrl (GPIO PCTL) <- alternate input <- mode control (GPIO AFSEL, ADCCTL, DMACTL) <-package io pin
 * MCU input -> mode control (GPIO AFSEL, ADCCTL, DMACTL) -> package io pin
 * MCU output <- mode control (GPIO AFSEL, ADCCTL, DMACTL) <- package io pin
 */


typedef struct ti_pinmux_index_to_value {
	uint8_t idx;
	uint32_t val;
} ti_pinmux_index_to_value_t;

static const ti_pinmux_index_to_value_t pinmux_dir_map [] = {
	{.idx = 0, .val = GPIO_DIR_MODE_IN},
	{.idx = 1, .val = GPIO_DIR_MODE_OUT},
	{.idx = 2, .val = GPIO_DIR_MODE_HW},
};

static const ti_pinmux_index_to_value_t pinmux_strength_map [] = {
	{.idx = 0, .val = GPIO_STRENGTH_2MA},
	{.idx = 1, .val = GPIO_STRENGTH_4MA},
	{.idx = 2, .val = GPIO_STRENGTH_6MA},
	{.idx = 3, .val = GPIO_STRENGTH_8MA},
	{.idx = 4, .val = GPIO_STRENGTH_8MA_SC},
	{.idx = 5, .val = GPIO_STRENGTH_10MA},
	{.idx = 6, .val = GPIO_STRENGTH_12MA},
};

static const ti_pinmux_index_to_value_t pinmux_pintype_map [] = {
	{.idx = 0, .val = GPIO_PIN_TYPE_STD},
	{.idx = 1, .val = GPIO_PIN_TYPE_STD_WPU},
	{.idx = 2, .val = GPIO_PIN_TYPE_STD_WPD},
	{.idx = 3, .val = GPIO_PIN_TYPE_OD},
	{.idx = 4, .val = GPIO_PIN_TYPE_ANALOG},
	{.idx = 5, .val = GPIO_PIN_TYPE_WAKE_HIGH},
	{.idx = 6, .val = GPIO_PIN_TYPE_WAKE_LOW},
};

static uint32_t getValue(const uint8_t idx, const ti_pinmux_index_to_value_t * pMap, const uint8_t N) {
	for (uint8_t i = 0; i < N; i++) {
		if (idx == pMap[i].idx) return pMap[i].val;
	}
	// we should never get here
	__ASSERT(0==0,"parameter error");
	return 0;
}

// stm32_dt_pinctrl_configure
void pinmux_tiva_cfg(const ti_tiva_gpio_pinctrl_t * cfg) {
	const uint8_t pinMask = BIT(cfg->pinNum);
	const uint32_t base = cfg->portAddr;
	const uint32_t dirmode = getValue(cfg->dirmodeIndx,pinmux_dir_map, ARRAY_SIZE(pinmux_dir_map));
	const uint32_t strength = getValue(cfg->strengthIndx,pinmux_strength_map, ARRAY_SIZE(pinmux_strength_map));
	const uint32_t pintype = getValue(cfg->pintypeIndx,pinmux_pintype_map, ARRAY_SIZE(pinmux_pintype_map));

	sysctl_activatePeripheral(base);

	if (1 == cfg->unlock) {
		GPIOUnlockPin(base, pinMask);
	}

	if (cfg->pinmapDef) {
		GPIOPinConfigure(cfg->pinmapDef);
	}

	GPIODirModeSet(base, pinMask, dirmode);

    GPIOPadConfigSet(base, pinMask, strength, pintype);
}

void pinmux_tiva_arrayCfg(const ti_tiva_gpio_pinctrl_t * cfg, const unsigned int N) {
	// 1st iterate through all ports and unlock pins
	for (unsigned int i = 0; i < N; i++) {
		const uint32_t base = cfg[i].portAddr;
		sysctl_activatePeripheral(base);
		if (1 == cfg[i].unlock) {
			const uint8_t pinMask = BIT(cfg[i].pinNum);
			GPIOUnlockPin(base, pinMask);
		}
	}

	// 2nd pin config
	for (unsigned int i = 0; i < N; i++) {
		if (cfg[i].pinmapDef) {
			GPIOPinConfigure(cfg[i].pinmapDef);
		}
	}

	// 2nd pad config
	for (unsigned int i = 0; i < N; i++) {
		const uint8_t pinMask = BIT(cfg[i].pinNum);
		const uint32_t base = cfg[i].portAddr;
		const uint32_t dirmode = getValue(cfg[i].dirmodeIndx,pinmux_dir_map, ARRAY_SIZE(pinmux_dir_map));
		const uint32_t strength = getValue(cfg[i].strengthIndx,pinmux_strength_map, ARRAY_SIZE(pinmux_strength_map));
		const uint32_t pintype = getValue(cfg[i].pintypeIndx,pinmux_pintype_map, ARRAY_SIZE(pinmux_pintype_map));
		GPIODirModeSet(base, pinMask, dirmode);
	    GPIOPadConfigSet(base, pinMask, strength, pintype);
	}
}
