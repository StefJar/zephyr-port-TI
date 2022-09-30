/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_adc

#include <stddef.h>

#include <kernel.h>
#include <zephyr.h>
#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"
#include "pinmux_tiva.h"
#include "adc_tiva.h"


struct adc_tiva_config {
	uint32_t base;
	const ti_tiva_gpio_pinctrl_t *pinctrl_list;
	const size_t pinctrl_list_size;
	const uint32_t VRef;
	const uint32_t oversampling;
};

struct adc_tiva_runtime {
	struct k_sem seqS[4];
};

#define DEV_CFG(dev) ((const struct adc_tiva_config *const)(dev)->config)

#define DEV_DATA(dev) ((struct adc_tiva_runtime *const)(dev)->data)

static void adc_tiva_isr_seq0(const struct device *dev) {
	ADCIntClear(DEV_CFG(dev)->base, 0);
	k_sem_give(&(DEV_DATA(dev)->seqS[0]));
}

static void adc_tiva_isr_seq1(const struct device *dev) {
	ADCIntClear(DEV_CFG(dev)->base, 1);
	k_sem_give(&(DEV_DATA(dev)->seqS[1]));
}

static void adc_tiva_isr_seq2(const struct device *dev) {
	ADCIntClear(DEV_CFG(dev)->base, 2);
	k_sem_give(&(DEV_DATA(dev)->seqS[2]));
}

static void adc_tiva_isr_seq3(const struct device *dev) {
	ADCIntClear(DEV_CFG(dev)->base, 3);
	k_sem_give(&(DEV_DATA(dev)->seqS[3]));
}

static int adc_tiva_dev_init(const struct device *dev){
	const uint32_t base = DEV_CFG(dev)->base;
	struct adc_tiva_runtime *data = DEV_DATA(dev);

	pinmux_tiva_arrayCfg(DEV_CFG(dev)->pinctrl_list, DEV_CFG(dev)->pinctrl_list_size);

	sysctl_activatePeripheral(base);
	for (unsigned int i = 0; i < 4; i++) {
		k_sem_init(&(data->seqS[i]), 0, 1);
	};
	return 0;

	ADCHardwareOversampleConfigure(base, DEV_CFG(dev)->oversampling);
	if (0 == DEV_CFG(dev)->VRef) {
		ADCReferenceSet(base,ADC_REF_INT);
	} else {
		ADCReferenceSet(base,ADC_REF_EXT_3V);
	}
	ADCIntClear(base, 0);
}

static const uint32_t channelToValueMap [] = {
	ADC_CTL_CH0,// Input channel 0
	ADC_CTL_CH1,// Input channel 1
	ADC_CTL_CH2,// Input channel 2
	ADC_CTL_CH3,// Input channel 3
	ADC_CTL_CH4,// Input channel 4
	ADC_CTL_CH5,// Input channel 5
	ADC_CTL_CH6,// Input channel 6
	ADC_CTL_CH7,// Input channel 7
	ADC_CTL_CH8,// Input channel 8
	ADC_CTL_CH9,// Input channel 9
	ADC_CTL_CH10,// Input channel 10
	ADC_CTL_CH11,// Input channel 11
	ADC_CTL_CH12,// Input channel 12
	ADC_CTL_CH13,// Input channel 13
	ADC_CTL_CH14,// Input channel 14
	ADC_CTL_CH15,// Input channel 15
	ADC_CTL_CH16,// Input channel 16
	ADC_CTL_CH17,// Input channel 17
	ADC_CTL_CH18,// Input channel 18
	ADC_CTL_CH19,// Input channel 19
	ADC_CTL_CH20,// Input channel 20
	ADC_CTL_CH21,// Input channel 21
	ADC_CTL_CH22,// Input channel 22
	ADC_CTL_CH23,// Input channel 23
};

static int adc_tiva_setupSeqence(const struct device *dev, const unsigned int seqN, const unsigned int cha) {
	const uint32_t base = DEV_CFG(dev)->base;

	if (seqN > 3) {
		return -1;
	}
	if (cha >= ARRAY_SIZE(channelToValueMap)) {
		return -1;
	}

	// sequencer without trigger
	ADCSequenceConfigure(base, seqN, ADC_TRIGGER_PROCESSOR, 0);

	const uint32_t adc_ch = channelToValueMap[cha];
	ADCSequenceStepConfigure(base, seqN, 0, adc_ch | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(base, seqN);
	ADCIntClear(base, seqN);
	ADCIntEnable(base,seqN);
	return 0;
}

static void adc_tiva_disableSeqence(const struct device *dev, const unsigned int seqN) {
	const uint32_t base = DEV_CFG(dev)->base;

	if (seqN > 3) {
		return;
	}
	ADCIntDisable(base, seqN);
	ADCSequenceDisable(base, seqN);
	ADCIntClear(base, seqN);
	return 0;
}

static int adc_tiva_sample(const struct device *dev, const unsigned int seqN, unsigned int * adcVal) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct adc_tiva_runtime *data = DEV_DATA(dev);

	ADCProcessorTrigger(base, seqN);

	k_sem_take(&(data->seqS[seqN]), K_FOREVER);

	uint32_t v = 0xFFFFFF;
	ADCSequenceDataGet(base, seqN, &v);
	*adcVal = v;
	return 0;
}

static const struct tiva_adc_driver_api adc_tiva_driver_api = {
	.setupSeqence = adc_tiva_setupSeqence,
	.disableSequence = adc_tiva_disableSeqence,
	.sample = adc_tiva_sample,
};

#define TIVA_ADC_DEVICE(n)																\
	static const ti_tiva_gpio_pinctrl_t adc_pins_##n[] = TI_TIVA_DT_INST_PINCTRL(n, 0);	\
	static const struct adc_tiva_config adc_## n ##_tiva_cfg={							\
			.base = DT_INST_REG_ADDR(n), 												\
			.pinctrl_list = adc_pins_##n,												\
			.pinctrl_list_size = ARRAY_SIZE(adc_pins_##n),								\
			.VRef = DT_INST_PROP(n ,reference_ENUM_IDX),								\
			.oversampling = DT_INST_PROP(n ,oversampling),								\
	};																					\
	static struct adc_tiva_runtime adc_## n ##_tiva_runtime ={							\
	}; 																					\
	static int adc_## n ##_tiva_init(const struct device *dev) { 						\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,0, irq), 										\
				DT_INST_IRQ_BY_IDX(n,0, priority), 										\
			    adc_tiva_isr_seq0, DEVICE_DT_INST_GET(n), 0); 							\
		irq_enable(DT_INST_IRQ_BY_IDX(n,0, irq)); 										\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,1, irq), 										\
				DT_INST_IRQ_BY_IDX(n,1, priority), 										\
				adc_tiva_isr_seq1, DEVICE_DT_INST_GET(n), 0); 							\
		irq_enable(DT_INST_IRQ_BY_IDX(n,1, irq)); 										\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,2, irq), 										\
				DT_INST_IRQ_BY_IDX(n,2, priority), 										\
				adc_tiva_isr_seq2, DEVICE_DT_INST_GET(n), 0); 							\
		irq_enable(DT_INST_IRQ_BY_IDX(n,2, irq)); 										\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,3, irq), 										\
				DT_INST_IRQ_BY_IDX(n,3, priority), 										\
				adc_tiva_isr_seq3, DEVICE_DT_INST_GET(n), 0); 							\
		irq_enable(DT_INST_IRQ_BY_IDX(n,3, irq)); 										\
		return adc_tiva_dev_init(dev); 													\
	}																					\
	DEVICE_DT_INST_DEFINE(n,															\
		adc_## n ##_tiva_init,															\
		NULL,																			\
		&adc_## n ##_tiva_runtime,														\
		&adc_## n ##_tiva_cfg,															\
		POST_KERNEL, 50,																\
		&adc_tiva_driver_api); 															\


DT_INST_FOREACH_STATUS_OKAY(TIVA_ADC_DEVICE)
