/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>

#include <errno.h>
#include <stdint.h>

#include "cfg_tiva.h"

int tiva_pll_init(const struct device *dev) {
	// currently only setup for 80MHz pll
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    // Enable peripherals to operate when CPU is in sleep
    // SysCtlPeripheralClockGating(true);
	return 0;
}

DEVICE_DT_DEFINE(DT_NODELABEL(pll),
	&tiva_pll_init,
	NULL,
	NULL, NULL,
	PRE_KERNEL_1,
	1,
	NULL
);
