TI Zephyr port
#########################

.. contents::

Introduction
************

For 2 projects TIER SE created a Zephyr port of TI some chips(tm4c123xxx). After getting permission from TIER, I am happy to publish our source code. Of course without any warenty. Feel free to use and share it.

Thanks to my colleagues and co workers Dale Whitfield and Kamil Sroka. 


Installation
************

install to your local git repro

link following folders:
	- dts
	- soc
	- ti_drv
	- ti_hal
	- tier_drv
to your project.
add cmake files in ti_drv, ti_hal and tier_drv to your project cmake file.

Content
************

dts
===

contains the TI MCU and TI & TIER driver definitions for the device tree compiler

soc
===

contains the TI MCU kernel config and startup code

ti_hal
======

contains the TI HAL which is used by the TI TIVA zephyr drivers. We build our drivers against the sources and not the ROM version.

ti_drv
======

contains the TI TIVA MCU drivers, like GPIO, I2C etc.

tier_drv
========

contains IC and pheriphal drivers used at TIER firmware projects. Usually these drivers are closely linked with the ti_drv so you might need these to use them.
