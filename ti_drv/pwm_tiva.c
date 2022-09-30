/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_pwm

#include <kernel.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"
#include "pinmux_tiva.h"

#include "pwm_tiva.h"

struct pwm_tiva_config {
	const uint32_t base;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	const uint32_t sysClkDiv;
};

struct pwm_tiva_runtime {
};

#define DEV_CFG(dev) ((const struct pwm_tiva_config *const)(dev)->config)

#define DEV_DATA(dev) ((struct pwm_tiva_runtime *const)(dev)->data)

static void pwm_tiva_isr_fault(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct pwm_tiva_runtime *data = DEV_DATA(dev);
}

static void pwm_tiva_isr_gen0(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct pwm_tiva_runtime *data = DEV_DATA(dev);
}

static void pwm_tiva_isr_gen1(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct pwm_tiva_runtime *data = DEV_DATA(dev);
}

static void pwm_tiva_isr_gen2(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct pwm_tiva_runtime *data = DEV_DATA(dev);
}

static void pwm_tiva_isr_gen3(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct pwm_tiva_runtime *data = DEV_DATA(dev);
}

static int pwm_tiva_init(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct pwm_tiva_runtime *data = DEV_DATA(dev);
	uint32_t sysDivFactor;

	pinmux_tiva_arrayCfg(DEV_CFG(dev)->pinctrl_list, DEV_CFG(dev)->pinctrl_list_size);

	switch (DEV_CFG(dev)->sysClkDiv) {
		case 1:
			sysDivFactor = SYSCTL_PWMDIV_1;
			break;
		case 2:
			sysDivFactor = SYSCTL_PWMDIV_2;
			break;
		case 4:
			sysDivFactor = SYSCTL_PWMDIV_4;
			break;
		case 8:
			sysDivFactor = SYSCTL_PWMDIV_8;
			break;
		case 16:
			sysDivFactor = SYSCTL_PWMDIV_16;
			break;
		case 32:
			sysDivFactor = SYSCTL_PWMDIV_32;
			break;
		case 64:
			sysDivFactor = SYSCTL_PWMDIV_64;
			break;
		default:
			return -1;
	}
	SysCtlPWMClockSet(sysDivFactor);
	sysctl_activatePeripheral(base);

	/*
	// counter set up
	PWMGenConfigure(base,PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(base, PWM_GEN_3, 40000);

	// pwm gen
	PWMPulseWidthSet(base, PWM_OUT_7, PWMGenPeriodGet(base, PWM_GEN_3) / 4);
	PWMOutputState(base, PWM_OUT_7_BIT, true);

	// activate pwm generator
	PWMGenEnable(base, PWM_GEN_3);
	*/
	return 0;
}

static const struct tiva_pwm_driver_api api = {
};

#define TIVA_PWM_DEVICE(n)										\
	static const ti_tiva_gpio_pinctrl_t pwm_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct pwm_tiva_config pwm_## n ##_tiva_cfg={	\
		.base = DT_INST_REG_ADDR(n),							\
		.pinctrl_list = pwm_pins_##n,							\
		.pinctrl_list_size = ARRAY_SIZE(pwm_pins_##n),			\
		.sysClkDiv = DT_INST_PROP(n, sysclk_div),				\
	};															\
	static struct pwm_tiva_runtime pwm_## n ##_tiva_runtime;	\
	static int pwm_## n ##_tiva_init(const struct device *dev) {\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,0, irq), 				\
				DT_INST_IRQ_BY_IDX(n,0, priority), 				\
				pwm_tiva_isr_fault, DEVICE_DT_INST_GET(n), 0); 	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,0, irq)); 				\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,1, irq), 				\
				DT_INST_IRQ_BY_IDX(n,1, priority), 				\
				pwm_tiva_isr_gen0, DEVICE_DT_INST_GET(n), 0); 	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,1, irq)); 				\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,2, irq), 				\
				DT_INST_IRQ_BY_IDX(n,2, priority), 				\
				pwm_tiva_isr_gen1, DEVICE_DT_INST_GET(n), 0); 	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,2, irq)); 				\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,3, irq), 				\
				DT_INST_IRQ_BY_IDX(n,3, priority), 				\
				pwm_tiva_isr_gen2, DEVICE_DT_INST_GET(n), 0); 	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,3, irq)); 				\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,4, irq), 				\
				DT_INST_IRQ_BY_IDX(n,4, priority), 				\
				pwm_tiva_isr_gen3, DEVICE_DT_INST_GET(n), 0); 	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,3, irq)); 				\
		return pwm_tiva_init(dev);								\
	};															\
	DEVICE_DT_INST_DEFINE(n,									\
		pwm_## n ##_tiva_init,									\
		NULL,													\
		&pwm_## n ##_tiva_runtime,								\
		&pwm_## n ##_tiva_cfg,									\
		POST_KERNEL, 50,										\
		&api);													\


DT_INST_FOREACH_STATUS_OKAY(TIVA_PWM_DEVICE)
