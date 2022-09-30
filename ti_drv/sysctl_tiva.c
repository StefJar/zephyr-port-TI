/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include "cfg_tiva.h"
#include "sysctl_tiva.h"

typedef struct peripheralAddrToSysCtrl {
	const uint32_t p; // peripheral addresss
	const uint32_t i; // peripheral ident for sysctl
} peripheralAddrToSysCtrl_t;

#define INVALID_PER 0

static const peripheralAddrToSysCtrl_t peripheralToSysCtrl [] = {
	// gpios
	{.p=GPIO_PORTA_BASE,.i=SYSCTL_PERIPH_GPIOA},
	{.p=GPIO_PORTB_BASE,.i=SYSCTL_PERIPH_GPIOB},
	{.p=GPIO_PORTC_BASE,.i=SYSCTL_PERIPH_GPIOC},
	{.p=GPIO_PORTD_BASE,.i=SYSCTL_PERIPH_GPIOD},
	{.p=GPIO_PORTE_BASE,.i=SYSCTL_PERIPH_GPIOE},
	{.p=GPIO_PORTF_BASE,.i=SYSCTL_PERIPH_GPIOF},
	{.p=GPIO_PORTG_BASE,.i=SYSCTL_PERIPH_GPIOG},
	{.p=GPIO_PORTH_BASE,.i=SYSCTL_PERIPH_GPIOH},
	{.p=GPIO_PORTJ_BASE,.i=SYSCTL_PERIPH_GPIOJ},
	{.p=GPIO_PORTK_BASE,.i=SYSCTL_PERIPH_GPIOK},
	{.p=GPIO_PORTL_BASE,.i=SYSCTL_PERIPH_GPIOL},
	{.p=GPIO_PORTM_BASE,.i=SYSCTL_PERIPH_GPIOM},
	{.p=GPIO_PORTN_BASE,.i=SYSCTL_PERIPH_GPION},
	{.p=GPIO_PORTP_BASE,.i=SYSCTL_PERIPH_GPIOP},
	{.p=GPIO_PORTQ_BASE,.i=SYSCTL_PERIPH_GPIOQ},
	{.p=GPIO_PORTR_BASE,.i=SYSCTL_PERIPH_GPIOR},
	{.p=GPIO_PORTS_BASE,.i=SYSCTL_PERIPH_GPIOS},
	{.p=GPIO_PORTT_BASE,.i=SYSCTL_PERIPH_GPIOT},
	// uart
	{.p=UART0_BASE,.i=SYSCTL_PERIPH_UART0},
	{.p=UART1_BASE,.i=SYSCTL_PERIPH_UART1},
	{.p=UART2_BASE,.i=SYSCTL_PERIPH_UART2},
	{.p=UART3_BASE,.i=SYSCTL_PERIPH_UART3},
	{.p=UART4_BASE,.i=SYSCTL_PERIPH_UART4},
	{.p=UART5_BASE,.i=SYSCTL_PERIPH_UART5},
	{.p=UART6_BASE,.i=SYSCTL_PERIPH_UART6},
	{.p=UART7_BASE,.i=SYSCTL_PERIPH_UART7},
	// i2c
	{.p=I2C0_BASE,.i=SYSCTL_PERIPH_I2C0},
	{.p=I2C1_BASE,.i=SYSCTL_PERIPH_I2C1},
	{.p=I2C2_BASE,.i=SYSCTL_PERIPH_I2C2},
	{.p=I2C3_BASE,.i=SYSCTL_PERIPH_I2C3},
	{.p=I2C4_BASE,.i=SYSCTL_PERIPH_I2C4},
	{.p=I2C5_BASE,.i=SYSCTL_PERIPH_I2C5},
	// can
	{.p=CAN0_BASE,.i=SYSCTL_PERIPH_CAN0},
	{.p=CAN1_BASE,.i=SYSCTL_PERIPH_CAN1},
	// timer
	{.p=TIMER0_BASE,.i=SYSCTL_PERIPH_TIMER0},
	{.p=TIMER1_BASE,.i=SYSCTL_PERIPH_TIMER1},
	{.p=TIMER2_BASE,.i=SYSCTL_PERIPH_TIMER2},
	{.p=TIMER3_BASE,.i=SYSCTL_PERIPH_TIMER3},
	{.p=TIMER4_BASE,.i=SYSCTL_PERIPH_TIMER4},
	{.p=TIMER5_BASE,.i=SYSCTL_PERIPH_TIMER5},
	{.p=TIMER6_BASE,.i=SYSCTL_PERIPH_TIMER6},
	{.p=TIMER7_BASE,.i=SYSCTL_PERIPH_TIMER7},
	// adc
	{.p=ADC0_BASE,.i=SYSCTL_PERIPH_ADC0},
	{.p=ADC1_BASE,.i=SYSCTL_PERIPH_ADC1},
	// pwm
	{.p=PWM0_BASE,.i=SYSCTL_PERIPH_PWM0},
	{.p=PWM1_BASE,.i=SYSCTL_PERIPH_PWM1},
	// udma
	{.p=UDMA_BASE,.i=SYSCTL_PERIPH_UDMA},
	// watchdog
	{.p=WATCHDOG0_BASE,.i=SYSCTL_PERIPH_WDOG0},
	{.p=WATCHDOG1_BASE,.i=SYSCTL_PERIPH_WDOG1},
};

static bool syscontrolOn[ARRAY_SIZE(peripheralToSysCtrl)] = {false, };

static int findPeripheral(const uint32_t pAddr) {
	for (int i = 0; i < ARRAY_SIZE(peripheralToSysCtrl); i++) {
		if (peripheralToSysCtrl[i].p == pAddr) {
			return i;
		}
	}
	return -1;
}

bool sysctl_getActiveState(const uint32_t pAddr) {
	const int indx = findPeripheral(pAddr);
	if (indx < 0) return false;

	return syscontrolOn[indx];
}

bool sysctl_activatePeripheral(const uint32_t pAddr) {
	const int indx =findPeripheral(pAddr);
	if (indx < 0) return false;

	if (syscontrolOn[indx] == true) {
		return true;
	}

	const uint32_t sp = peripheralToSysCtrl[indx].i;
	// SysCtlPeripheralPowerOn(sp);

	SysCtlPeripheralEnable(sp);
	while(!SysCtlPeripheralReady(sp)){
	}

	syscontrolOn[indx] = true;
	return true;
}

void sysctl_deactivatePeripheral(const uint32_t pAddr) {
	const int indx =findPeripheral(pAddr);
	if (false == syscontrolOn[indx] ) {
		return;
	}
	const uint32_t sp = peripheralToSysCtrl[indx].i;
	SysCtlPeripheralDisable(sp);
	SysCtlPeripheralReset(sp);
	syscontrolOn[indx] = false;
}

void sysctl_getResetState(sysctl_reset_info_t * ri) {
	uint32_t rs = SysCtlResetCauseGet();
	ri->hardwareSystemServiceRequest = rs & SYSCTL_CAUSE_HSRVREQ ? 1 : 0;
	ri->hibernateReset = rs & SYSCTL_CAUSE_HIB ? 1 : 0;
	ri->watchdog1Reset = rs & SYSCTL_CAUSE_WDOG1 ? 1 : 0;
	ri->watchdog0Reset = rs & SYSCTL_CAUSE_WDOG0 ? 1 : 0;
	ri->softwareReset = rs & SYSCTL_CAUSE_SW ? 1 : 0;
	ri->brownOutReset = rs & SYSCTL_CAUSE_BOR ? 1 : 0;
	ri->powerOnReset = rs & SYSCTL_CAUSE_POR ? 1 : 0;
	ri->externalReset = rs & SYSCTL_CAUSE_EXT ? 1 : 0;
}

void sysctl_clearResetState(void) {
	SysCtlResetCauseClear(SysCtlResetCauseGet());
}

