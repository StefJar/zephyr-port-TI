cmake_minimum_required(VERSION 3.8)

list(APPEND INC
	${tiDrvDir}/
)

list(APPEND SRC
	${tiDrvDir}/sysctl_tiva.c
	${tiDrvDir}/pll_tiva.c
	${tiDrvDir}/pinmux_tiva.c
	${tiDrvDir}/gpio_tiva.c
	${tiDrvDir}/uart_tiva.c
	${tiDrvDir}/i2c_tiva.c
	${tiDrvDir}/can_tiva.c
	${tiDrvDir}/timers_tiva.c
	${tiDrvDir}/adc_tiva.c
	${tiDrvDir}/pwm_tiva.c
	${tiDrvDir}/rombootloader_tiva.c
	${tiDrvDir}/udma_tiva.c
	${tiDrvDir}/wdt_tiva.c
)

target_sources(app PRIVATE ${SRC})

target_include_directories(app PRIVATE ${INC})
