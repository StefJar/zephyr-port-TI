/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SRC_DRV_CFG_TIVA_H_
#define SRC_DRV_CFG_TIVA_H_

#if defined(TI_TM4C123GH6PGE)
#define PART_TM4C123GH6PGE
#include <tm4c123gh6pge.h>
#elif defined(TI_TM4C1230H6PM)
#define PART_TM4C1230H6PM
#include <tm4c1230h6pm.h>
#else
#error "Unsupported MCU specified"
#endif

#define TARGET_IS_TM4C123_RB1

#include <hw_types.h>
#include <hw_memmap.h>
#include <hw_nvic.h>
#include <hw_uart.h>
#include <hw_watchdog.h>
#include <hw_timer.h>
#include <hw_can.h>

#include <pin_map.h>
#include <gpio.h>
#include <sysctl.h>
#include <uart.h>
#include <i2c.h>
#include <can.h>
#include <timer.h>
#include <adc.h>
#include <pwm.h>
#include <rom.h>
#include <udma.h>
#include <watchdog.h>
#include <rom_map.h>

#endif /* SRC_DRV_CFG_TIVA_H_ */
