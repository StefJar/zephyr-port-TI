/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_gpio

#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include <soc.h>
#include <sys/sys_io.h>
#include "gpio_utils.h"

#include <stdint.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"

typedef void (*config_func_t)(const struct device *dev);

struct gpio_tiva_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint32_t base;
	config_func_t config_func;
};

struct gpio_tiva_runtime {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t cb;
};

#define DEV_CFG(dev)                         \
	((const struct gpio_tiva_config *const)  \
	(dev)->config)

#define DEV_DATA(dev)					 \
	((struct gpio_tiva_runtime *const)   \
	(dev)->data)

static void gpio_tiva_isr(const struct device *dev)
{
	const struct gpio_tiva_config * const cfg = DEV_CFG(dev);
	struct gpio_tiva_runtime *context = DEV_DATA(dev);

	const uint32_t portBase = cfg->base;
	uint32_t int_stat = GPIOIntStatus(portBase, true);
	GPIOIntClear(portBase, int_stat);
	gpio_fire_callbacks(&context->cb, dev, int_stat);
}

static int gpio_tiva_configure(const struct device *dev,gpio_pin_t pin, gpio_flags_t flags) {
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	struct gpio_tiva_runtime *context = DEV_DATA(dev);
	const uint32_t portBase = cfg->base;

	const uint8_t pinMask = BIT(pin);

	if (GPIO_OUTPUT == (flags & GPIO_OUTPUT)) {
		GPIOPinTypeGPIOOutput(portBase, pinMask);
		uint32_t v = GPIOPinRead(portBase, pinMask);
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			v |= BIT(pin);
		} else {
			v &= ~BIT(pin);
		}
		GPIOPinWrite(portBase, pinMask, v);
	}

	if (GPIO_INPUT == (flags & GPIO_INPUT)) {
		GPIOPinTypeGPIOInput(portBase, pinMask);
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		uint32_t strength, pinType;

		GPIOPadConfigGet(portBase, pinMask, &strength, &pinType);

		if (GPIO_PULL_UP == (flags & GPIO_PULL_UP)) {
			pinType = GPIO_PIN_TYPE_STD_WPU;
		}
		if (GPIO_PULL_DOWN == (flags & GPIO_PULL_DOWN)) {
			pinType = GPIO_PIN_TYPE_STD_WPD;
		}
		GPIOPadConfigSet(portBase, pinMask, strength, pinType);
	}

	if (GPIO_SINGLE_ENDED == (flags & GPIO_SINGLE_ENDED)) {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_tiva_port_get_raw(const struct device *dev, uint32_t *value) {
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	uint32_t portBase = cfg->base;
	*value = GPIOPinRead(portBase, 0xFF);
	return 0;
}

static int gpio_tiva_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value) {
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	const uint32_t portBase = cfg->base;
	GPIOPinWrite(portBase, mask, value);
	return 0;
}

static int gpio_tiva_port_set_bits_raw(const struct device *dev, uint32_t mask){
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	const uint32_t portBase = cfg->base;
	uint32_t v = GPIOPinRead(portBase, 0xFF);
	v |= mask;
	GPIOPinWrite(portBase, mask, v);
	return 0;
}

static int gpio_tiva_port_clear_bits_raw(const struct device *dev, uint32_t mask) {
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	const uint32_t portBase = cfg->base;
	uint32_t v = GPIOPinRead(portBase, 0xFF);
	v &= ~(mask);
	GPIOPinWrite(portBase, mask, v);

	return 0;
}

static int gpio_tiva_port_toggle_bits(const struct device *dev,uint32_t mask) {
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	const uint32_t portBase = cfg->base;
	uint32_t v = GPIOPinRead(portBase, 0xFF);
	v ^= mask;
	GPIOPinWrite(portBase, mask, v);
	return 0;
}

static const uint32_t gpioNumberToIntrNumer [] = {
	GPIO_INT_PIN_0,
	GPIO_INT_PIN_1,
	GPIO_INT_PIN_2,
	GPIO_INT_PIN_3,
	GPIO_INT_PIN_4,
	GPIO_INT_PIN_5,
	GPIO_INT_PIN_6,
	GPIO_INT_PIN_7,
};

static int gpio_tiva_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	const uint32_t portBase = cfg->base;
	if (pin > ARRAY_SIZE(gpioNumberToIntrNumer)) {
		return -ENODEV;
	}
	const uint32_t intPinMask = gpioNumberToIntrNumer[pin];


	// Check if GPIO port needs interrupt support
	if (mode == GPIO_INT_MODE_DISABLED) {
		GPIOIntDisable(portBase, intPinMask);
	} else {
		uint32_t modeFlags = 0;
		// edge detection
		if (mode == GPIO_INT_MODE_EDGE) {
			switch (trig) {
				case GPIO_INT_TRIG_BOTH:
					modeFlags = GPIO_BOTH_EDGES;
					break;
				case GPIO_INT_TRIG_HIGH:
					modeFlags = GPIO_RISING_EDGE;
					break;
				case GPIO_INT_TRIG_LOW:
				default:
					modeFlags = GPIO_FALLING_EDGE;
					break;
			}
		} else {
			// level detection
			switch (trig) {
				case GPIO_INT_TRIG_HIGH:
					modeFlags = GPIO_HIGH_LEVEL;
					break;
				case GPIO_INT_TRIG_LOW:
				default:
					modeFlags = GPIO_LOW_LEVEL;
					break;
			}
		}
		GPIOIntTypeSet(portBase, BIT(pin), modeFlags);
		GPIOIntClear(portBase, intPinMask);
		GPIOIntEnable(portBase, intPinMask);
	}
	return 0;
}

static int gpio_tiva_init(const struct device *dev)
{
	const struct gpio_tiva_config *cfg = DEV_CFG(dev);
	struct gpio_tiva_runtime *context = DEV_DATA(dev);

	cfg->config_func(dev);
	return 0;
}

static int gpio_tiva_manage_callback(const struct device *dev, struct gpio_callback *callback, bool set)
{
	struct gpio_tiva_runtime *context = DEV_DATA(dev);

	gpio_manage_callback(&context->cb, callback, set);

	return 0;
}

static const struct gpio_driver_api gpio_tiva_driver_api = {
	.pin_configure = gpio_tiva_configure,
	.port_get_raw = gpio_tiva_port_get_raw,
	.port_set_masked_raw = gpio_tiva_port_set_masked_raw,
	.port_set_bits_raw = gpio_tiva_port_set_bits_raw,
	.port_clear_bits_raw = gpio_tiva_port_clear_bits_raw,
	.port_toggle_bits = gpio_tiva_port_toggle_bits,
	.pin_interrupt_configure = gpio_tiva_pin_interrupt_configure,
	.manage_callback = gpio_tiva_manage_callback,
};

#define TIVA_GPIO_DEVICE(n)													\
	static void port_## n ##_tiva_config_func(const struct device *dev);	\
	static struct gpio_tiva_runtime port_## n ##_tiva_runtime;				\
	static const struct gpio_tiva_config gpio_tiva_port_## n ##_config = {	\
		.common = {															\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),			\
		},																	\
		.base = DT_INST_REG_ADDR(n),										\
		.config_func = port_## n ##_tiva_config_func,						\
	};																		\
	DEVICE_DT_INST_DEFINE(n,												\
			    gpio_tiva_init,												\
			    NULL,														\
			    &port_## n ##_tiva_runtime,									\
			    &gpio_tiva_port_## n ##_config,								\
			    POST_KERNEL, 40,											\
			    &gpio_tiva_driver_api);										\
	static void port_## n ##_tiva_config_func(const struct device *dev)	{	\
		sysctl_activatePeripheral(DT_INST_REG_ADDR(n)); 					\
		IRQ_CONNECT(DT_INST_IRQN(n),										\
			DT_INST_IRQ(n, priority),										\
			gpio_tiva_isr,													\
			DEVICE_DT_INST_GET(n), 0);										\
		irq_enable(DT_INST_IRQN(n));										\
	}																		\


DT_INST_FOREACH_STATUS_OKAY(TIVA_GPIO_DEVICE)
