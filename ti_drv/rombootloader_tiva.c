/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/timer/system_timer.h>

#include <errno.h>
#include <stdint.h>

#include "cfg_tiva.h"

#include "rombootloader_tiva.h"

const uint32_t tiva_bt_getVer(void) {
	return ROM_VERSION;
}

void tiva_bl_activateUART(void) {

	// stop systick
	sys_clock_disable();
	// disable all intr
	irq_lock();

	HWREG(NVIC_DIS0) = 0xffffffff;
	HWREG(NVIC_DIS1) = 0xffffffff;
	HWREG(NVIC_DIS2) = 0xffffffff;
	HWREG(NVIC_DIS3) = 0xffffffff;
	HWREG(NVIC_DIS4) = 0xffffffff;
/*
	CANDisable(CAN0_BASE);
	CANDisable(CAN1_BASE);

	I2CMasterDisable(I2C0_BASE);
	I2CMasterDisable(I2C1_BASE);
	I2CMasterDisable(I2C2_BASE);
	I2CMasterDisable(I2C3_BASE);

	TimerDisable(TIMER0_BASE, TIMER_BOTH);
	TimerDisable(TIMER1_BASE, TIMER_BOTH);
	TimerDisable(TIMER4_BASE, TIMER_BOTH);
	TimerDisable(TIMER5_BASE, TIMER_BOTH);
*/

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // UARTDisable(UART0_BASE);
	//
	// Configure the UART for 115200, 8-N-1.
	//
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));

	UARTEnable(UART0_BASE);
	UARTFIFOEnable(UART0_BASE);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    UARTDMADisable(UART0_BASE, UART_DMA_TX | UART_DMA_RX);
    UARTIntDisable(UART0_BASE, UART_INT_RX | UART_INT_RT |
		UART_INT_TX |
		UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE
	);
	UARTEnable(UART0_BASE);

	// start bootloader
	ROM_UpdateUART();
}
