.. _nrf52demo:

nRF52 DK - nRF52840
#####################

Overview
********

The nRF52840 SoC on the nRF9160 DK (PCA10090) hardware provides support for the
Nordic Semiconductor nRF52840 ARM Cortex-M4F CPU and the following devices:

* CLOCK
* FLASH
* :abbr:`GPIO (General Purpose Input Output)`
* :abbr:`MPU (Memory Protection Unit)`
* :abbr:`NVIC (Nested Vectored Interrupt Controller)`
* :abbr:`PWM (Pulse Width Modulation)`
* RADIO (Bluetooth Low Energy and 802.15.4)
* :abbr:`RTC (nRF RTC System Clock)`
* Segger RTT (RTT Console)
* :abbr:`UART (Universal asynchronous receiver-transmitter)`
* :abbr:`WDT (Watchdog Timer)`

The nRF52840 SoC does not have any connection to the any of the LEDs,
buttons, switches, and Arduino pin headers on the nRF9160 DK board. It is,
however, possible to route some of the pins of the nRF52840 SoC to the nRF9160
SiP.

More information about the board can be found at
the `Nordic Low power cellular IoT`_ website.
The `Nordic Semiconductor Infocenter`_
contains the processor's information and the datasheet.


Hardware
********

The nRF9160 DK has two external oscillators. The frequency of
the slow clock is 32.768 kHz. The frequency of the main clock
is 32 MHz.

Supported Features
==================

The nrf9160dk_nrf52840 board configuration supports the following
hardware features:

+-----------+------------+----------------------+
| Interface | Controller | Driver/Component     |
+===========+============+======================+
| CLOCK     | on-chip    | clock_control        |
+-----------+------------+----------------------+
| FLASH     | on-chip    | flash                |
+-----------+------------+----------------------+
| GPIO      | on-chip    | gpio                 |
+-----------+------------+----------------------+
| MPU       | on-chip    | arch/arm             |
+-----------+------------+----------------------+
| NVIC      | on-chip    | arch/arm             |
+-----------+------------+----------------------+
| PWM       | on-chip    | pwm                  |
+-----------+------------+----------------------+
| RADIO     | on-chip    | Bluetooth,           |
|           |            | ieee802154           |
+-----------+------------+----------------------+
| RTC       | on-chip    | system clock         |
+-----------+------------+----------------------+
| RTT       | Segger     | console              |
+-----------+------------+----------------------+
| UART      | on-chip    | serial               |
+-----------+------------+----------------------+
| WDT       | on-chip    | watchdog             |
+-----------+------------+----------------------+

Make sure that the PROG/DEBUG switch on the DK is set to nRF52.

Flashing
========

Remember to set the PROG/DEBUG switch on the DK to nRF52.


Debugging
=========

Remember to set the PROG/DEBUG switch on the DK to nRF52.

.. _nrf9160dk_board_controller_firmware:

Board controller firmware
*************************

+--------------------------------+----------------------------------+
| nRF9160 pins                   | Routed to                        |
+================================+==================================+
| P0.26, P0.27, P0.28, and P0.29 | VCOM0                            |
+--------------------------------+----------------------------------+
| P0.01, P0.00, P0.15, and P0.14 | VCOM2                            |
+--------------------------------+----------------------------------+
| P0.02                          | LED1                             |
+--------------------------------+----------------------------------+
| P0.03                          | LED2                             |
+--------------------------------+----------------------------------+
| P0.04                          | LED3                             |
+--------------------------------+----------------------------------+
| P0.05                          | LED4                             |
+--------------------------------+----------------------------------+
| P0.08                          | Switch 1                         |
+--------------------------------+----------------------------------+
| P0.09                          | Switch 2                         |
+--------------------------------+----------------------------------+
| P0.06                          | Button 1                         |
+--------------------------------+----------------------------------+
| P0.07                          | Button 2                         |
+--------------------------------+----------------------------------+
| P0.17, P0.18, and P0.19        | Arduino pin headers              |
+--------------------------------+----------------------------------+
| P0.21, P0.22, and P0.23        | Trace interface                  |
+--------------------------------+----------------------------------+
| COEX0, COEX1, and COEX2        | COEX interface                   |
+--------------------------------+----------------------------------+

+------------------------------------+------------------------------+
| Devicetree node label              | Analog switch name           |
+====================================+==============================+
| ``vcom0_pins_routing``             | nRF91_UART1 (nRF91_APP1)     |
+------------------------------------+------------------------------+
| ``vcom2_pins_routing``             | nRF91_UART2 (nRF91_APP2)     |
+------------------------------------+------------------------------+
| ``led1_pin_routing``               | nRF91_LED1                   |
+------------------------------------+------------------------------+
| ``led2_pin_routing``               | nRF91_LED2                   |
+------------------------------------+------------------------------+
| ``led3_pin_routing``               | nRF91_LED3                   |
+------------------------------------+------------------------------+
| ``led4_pin_routing``               | nRF91_LED4                   |
+------------------------------------+------------------------------+
| ``switch1_pin_routing``            | nRF91_SWITCH1                |
+------------------------------------+------------------------------+
| ``switch2_pin_routing``            | nRF91_SWITCH2                |
+------------------------------------+------------------------------+
| ``button1_pin_routing``            | nRF91_BUTTON1                |
+------------------------------------+------------------------------+
| ``button2_pin_routing``            | nRF91_BUTTON2                |
+------------------------------------+------------------------------+
| ``nrf_interface_pins_0_2_routing`` | nRF_IF0-2_CTRL (nRF91_GPIO)  |
+------------------------------------+------------------------------+
| ``nrf_interface_pins_3_5_routing`` | nRF_IF3-5_CTRL (nRF91_TRACE) |
+------------------------------------+------------------------------+
| ``nrf_interface_pins_6_8_routing`` | nRF_IF6-8_CTRL (nRF91_COEX)  |
+------------------------------------+------------------------------+

+------------------------------------+------------------------------+
| Devicetree node label              | Analog switch name           |
+====================================+==============================+
| ``nrf_interface_pin_9_routing``    | nRF_IF9_CTRL                 |
+------------------------------------+------------------------------+
| ``io_expander_pins_routing``       | IO_EXP_EN                    |
+------------------------------------+------------------------------+
| ``external_flash_pins_routing``    | EXT_MEM_CTRL                 |
+------------------------------------+------------------------------+

References
**********

.. target-notes::
.. _Nordic Low power cellular IoT: https://www.nordicsemi.com/Products/Low-power-cellular-IoT
.. _Nordic Semiconductor Infocenter: https://infocenter.nordicsemi.com
.. _J-Link Software and documentation pack: https://www.segger.com/jlink-software.html
.. _nRF9160 DK board control section in the nRF9160 DK User Guide: https://infocenter.nordicsemi.com/topic/ug_nrf91_dk/UG/nrf91_DK/board_controller.html
