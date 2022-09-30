cmake_minimum_required(VERSION 3.8)

list(APPEND INC
	${tierDrvDir}/
)

list(APPEND SRC
	${tierDrvDir}/tier_fan_3pin.c
	${tierDrvDir}/adpd188bi.c
	${tierDrvDir}/tier_shiftreg.c
	${tierDrvDir}/ti_tlc5973.c
	${tierDrvDir}/ti_tmp1075.c
	${tierDrvDir}/ti_pcf8574.c
	${tierDrvDir}/tier_mq2.c
	${tierDrvDir}/hc_sr501.c
	${tierDrvDir}/tier_muxed_can.c
	${tierDrvDir}/ti_drv8830drcr.c
	${tierDrvDir}/tier_gpio_light_controller.c
	${tierDrvDir}/tier_pwm_light_controller.c


target_sources(app PRIVATE ${SRC})

target_include_directories(app PRIVATE ${INC})
