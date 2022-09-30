cmake_minimum_required(VERSION 3.8)

add_compile_definitions(gcc)

list(APPEND INC
	${halDir}/
	${halDir}/inc
	${halDir}/driverlib
)

list(APPEND SRC
	${halDir}/adc.c
	${halDir}/aes.c
	${halDir}/can.c
	${halDir}/comp.c
	${halDir}/cpu.c
	${halDir}/crc.c
	${halDir}/des.c
	${halDir}/eeprom.c
	${halDir}/emac.c
	${halDir}/epi.c
	${halDir}/flash.c
	${halDir}/fpu.c
	${halDir}/gpio.c
	${halDir}/hibernate.c
	${halDir}/i2c.c
	${halDir}/interrupt.c
	${halDir}/lcd.c
	${halDir}/mpu.c
	${halDir}/onewire.c
	${halDir}/pwm.c
	${halDir}/qei.c
	${halDir}/shamd5.c
	${halDir}/ssi.c
	${halDir}/sw_crc.c
	${halDir}/sysctl.c
	${halDir}/sysexc.c
	${halDir}/systick.c
	${halDir}/timer.c
	${halDir}/uart.c
	${halDir}/udma.c
	${halDir}/usb.c
	${halDir}/watchdog.c
)

target_sources(app PRIVATE ${SRC})

target_include_directories(app PRIVATE ${INC})
