/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// single line protocoll
// led_x = <12 Bit header><12Bit red><12Bit green><12Bit blue>
// 1 bit = 1 * t_cycle
// t_cycle = lo->hi, bit

// One Communication Cycle End of Sequence (EOS) = sin -> low for 4 x t_cycle
// GS Data Latch (GSLAT) Sequence = sin -> low for 8 x t_cycle

#define DT_DRV_COMPAT ti_tlc5973

#include <stddef.h>

#include <kernel.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <devicetree.h>

#include "../ti_drv/cfg_tiva.h"
#include "../ti_drv/sysctl_tiva.h"
#include "../ti_drv/ti_tiva_dt.h"
#include "../ti_drv/pinmux_tiva.h"
#include "../ti_drv/timers_tiva.h"

#include "led1wire.h"

#define TLC5973_F_INTERVAL_HZ 250000
//#define TLC5973_F_INTERVAL_HZ 100000
#define TLC5973_WRITE_CMD 0x3AA
#define TLC5973_DATA_WIDTH 12
#define TLC5973_EOS_CYCLES 4
#define TLC5973_GSLAT_CYCLES 8

typedef struct tlc5973_led{
	uint16_t r : 12;
	uint16_t g : 12;
	uint16_t b : 12;
} tlc5973_led_t;

typedef enum tlc5973_states {
	tlc5973_state_start = 0,
	tlc5973_state_led_header = 1,
	tlc5973_state_led_r = 2,
	tlc5973_state_led_g = 3,
	tlc5973_state_led_b = 4,
	tlc5973_state_eos = 5,
	tlc5973_state_gslat = 6,
	tlc5973_state_done = 7
} tlc5973_state_t;

struct tlc5973_config {
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	const struct gpio_dt_spec sdiPin;
	const char * clkGen;
	const unsigned int amount;
};

struct tlc5973_runtime {
	tlc5973_led_t * pLedArray; // led array
	struct {
		const struct device * timerDev;
		struct k_mutex gm;
		tlc5973_state_t s;
		unsigned int ledIndx;
		unsigned int bitIndx;
		unsigned int counter;
		uint16_t curWord;
	} updateProc;
};

#define DEV_CFG(dev)                          \
	((const struct tlc5973_config *const)     \
	(dev)->config)

#define DEV_DATA(dev)					\
	((struct tlc5973_runtime *const)   	\
	(dev)->data)

static unsigned int tlc5973_getAmount(const struct device *dev){
	return DEV_CFG(dev)->amount;
}

static void tlc5973_set12bitRGB(const struct device *dev, const unsigned int indx, const uint16_t red, const uint16_t green, const uint16_t blue){
	DEV_DATA(dev)->pLedArray[indx].r = red;
	DEV_DATA(dev)->pLedArray[indx].g = green;
	DEV_DATA(dev)->pLedArray[indx].b = blue;
}

static void tlc5973_set8bitRGB(const struct device *dev, const unsigned int indx, const uint8_t red, const uint8_t green, const uint8_t blue){
}

static void tlc5973_timerFunc(void * param) {
	struct tlc5973_runtime * d = DEV_DATA((const struct device *)param);
	const struct tlc5973_config * c = DEV_CFG((const struct device *)param);

	bool transferFlag;
	// check initial state
	switch(d->updateProc.s) {
		case tlc5973_state_start:
			d->updateProc.bitIndx = 0;
			d->updateProc.ledIndx = 0;
			// fall through
		case tlc5973_state_led_header:
			if (0 == d->updateProc.bitIndx) {
				d->updateProc.curWord = TLC5973_WRITE_CMD;
			}
			transferFlag = true;
			break;
		case tlc5973_state_led_r:
			if (0 == d->updateProc.bitIndx) {
				d->updateProc.curWord = d->pLedArray[d->updateProc.ledIndx].r;
			}
			transferFlag = true;
			break;
		case tlc5973_state_led_g:
			if (0 == d->updateProc.bitIndx) {
				d->updateProc.curWord = d->pLedArray[d->updateProc.ledIndx].g;
			}
			transferFlag = true;
			break;
		case tlc5973_state_led_b:
			if (0 == d->updateProc.bitIndx) {
				d->updateProc.curWord = d->pLedArray[d->updateProc.ledIndx].b;
			}
			transferFlag = true;
			break;
		case tlc5973_state_eos:
			d->updateProc.counter--;
			if (0 == d->updateProc.counter) {
				d->updateProc.ledIndx++;
			}
			transferFlag = false;
			break;
		case tlc5973_state_gslat:
			d->updateProc.counter--;
			transferFlag = false;
			break;
		case tlc5973_state_done:
			transferFlag = false;
			break;
		default:
			__ASSERT(0==0,"unknown state");
			return;
			break;
	}
	// transfer data if needed
	if (true == transferFlag) {
		// generate rising edge
		gpio_pin_set_dt(&(c->sdiPin), 1);
		gpio_pin_set_dt(&(c->sdiPin), 0);
		// write pin value
		const unsigned int v = d->updateProc.curWord & BIT(TLC5973_DATA_WIDTH - 1- d->updateProc.bitIndx);
		d->updateProc.bitIndx++;
		if (v) {
			gpio_pin_set_dt(&(c->sdiPin), 1);
			gpio_pin_set_dt(&(c->sdiPin), 0);
		}
	}
	// get new state
	switch(d->updateProc.s) {
		case tlc5973_state_start:
			d->updateProc.s = tlc5973_state_led_header;
			break;
		case tlc5973_state_led_header:
			if (TLC5973_DATA_WIDTH == d->updateProc.bitIndx) {
				d->updateProc.s = tlc5973_state_led_r;
				d->updateProc.bitIndx = 0;
			}
			break;
		case tlc5973_state_led_r:
			if (TLC5973_DATA_WIDTH == d->updateProc.bitIndx) {
				d->updateProc.s = tlc5973_state_led_g;
				d->updateProc.bitIndx = 0;
			}
			break;
		case tlc5973_state_led_g:
			if (TLC5973_DATA_WIDTH == d->updateProc.bitIndx) {
				d->updateProc.s = tlc5973_state_led_b;
				d->updateProc.bitIndx = 0;
			}
			break;
		case tlc5973_state_led_b:
			if (TLC5973_DATA_WIDTH == d->updateProc.bitIndx) {
				d->updateProc.bitIndx = 0;
				if (d->updateProc.ledIndx >= c->amount) {
					d->updateProc.counter = TLC5973_GSLAT_CYCLES;
					d->updateProc.s = tlc5973_state_gslat;
				} else {
					d->updateProc.counter = TLC5973_EOS_CYCLES;
					d->updateProc.s = tlc5973_state_eos;
				}
			}
			break;
		case tlc5973_state_eos:
			if (0 == d->updateProc.counter) {
				d->updateProc.s = tlc5973_state_led_header;
			}
			break;
		case tlc5973_state_gslat:
			if (0 == d->updateProc.counter) {
				d->updateProc.s = tlc5973_state_done;
			}
			break;
		case tlc5973_state_done:
			timers_stop(d->updateProc.timerDev, timers_cfg_timerA);
			break;
		default:
			__ASSERT(0==0,"unknown state");
			break;
	}
}

static void tlc5973_update(const struct device *dev){
	struct tlc5973_runtime * d = DEV_DATA((const struct device *)dev);
	const struct tlc5973_config * c = DEV_CFG((const struct device *)dev);

	k_mutex_lock(&(DEV_DATA(dev)->updateProc.gm), K_FOREVER);
	gpio_pin_set_dt(&(DEV_CFG(dev)->sdiPin), 0);
	d->updateProc.s = tlc5973_state_start;
	d->updateProc.curWord = 0;
	d->updateProc.bitIndx = 0;
	d->updateProc.ledIndx = 0;
	d->updateProc.counter = 0;
	const unsigned int key = irq_lock();
	timers_start(d->updateProc.timerDev, timers_cfg_timerA);
	// wait for the intr to change the state
	while(tlc5973_state_done != d->updateProc.s) {

	}
	gpio_pin_set_dt(&(DEV_CFG(dev)->sdiPin), 1);
	irq_unlock(key);
	k_mutex_unlock(&(d->updateProc.gm));
}

static int tlc5973_dev_init(const struct device *dev){
	const tlc5973_led_t ledInit = {
		.r = 0x0,
		.g = 0x0,
		.b = 0x0,
	};
	gpio_pin_configure_dt(&(DEV_CFG(dev)->sdiPin), GPIO_OUTPUT_LOW);
	pinmux_tiva_arrayCfg(DEV_CFG(dev)->pinctrl_list, DEV_CFG(dev)->pinctrl_list_size);

	k_mutex_init(&(DEV_DATA(dev)->updateProc.gm));

	for (unsigned int i = 0; i < DEV_CFG(dev)->amount;i++) {
		DEV_DATA(dev)->pLedArray[i] = ledInit;
	}

	const struct device *timerDev = device_get_binding(DEV_CFG(dev)->clkGen);
	DEV_DATA(dev)->updateProc.timerDev = timerDev;
	timers_setupIntervalCallback(timerDev, timers_cfg_timerA, tlc5973_timerFunc, dev);
	timers_setupInterval(timerDev, timers_cfg_timerA, TLC5973_F_INTERVAL_HZ);
	timers_stop(timerDev, timers_cfg_timerA);
	tlc5973_update(dev);
	return 0;
}

static const struct led1wire_driver_api tlc5973_driver_api = {
	.getAmount = tlc5973_getAmount,
	.set8bitRGB = tlc5973_set8bitRGB,
	.set12bitRGB = tlc5973_set12bitRGB,
	.update = tlc5973_update,
};

#define TLC5973_DEVICE(n)																	\
	static const ti_tiva_gpio_pinctrl_t tlc5973_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static tlc5973_led_t tlc5973_leds_##n[DT_INST_PROP(n ,amount)];							\
	static const struct tlc5973_config tlc5973_## n ##_cfg={								\
		.pinctrl_list = tlc5973_pins_##n,													\
		.pinctrl_list_size = ARRAY_SIZE(tlc5973_pins_##n),									\
		.sdiPin = GPIO_DT_SPEC_INST_GET (n ,sdi_gpios), 									\
		.clkGen = DT_PROP(DT_INST_PHANDLE(n ,clkgen), label),								\
		.amount = DT_INST_PROP(n ,amount),	 												\
	};																						\
	static struct tlc5973_runtime tlc5973_## n ##_runtime = {								\
		.pLedArray = tlc5973_leds_##n,														\
	};																						\
	DEVICE_DT_INST_DEFINE(n,																\
		tlc5973_dev_init,																	\
		NULL,																				\
		&tlc5973_## n ##_runtime,															\
		&tlc5973_## n ##_cfg,																\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,									\
		&tlc5973_driver_api); 																\


DT_INST_FOREACH_STATUS_OKAY(TLC5973_DEVICE)
