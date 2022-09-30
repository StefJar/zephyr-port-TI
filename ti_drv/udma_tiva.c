/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_udma

#include <zephyr.h>
#include <kernel.h>
#include <device.h>
#include <string.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"

// the control table to use for channel control structures
static uint8_t __aligned(1024) udmaControlTable[1024];

struct udma_tiva_config {
	const uint32_t base;
};

#define DEV_CFG(dev) ((const struct udma_tiva_config *const)(dev)->config)

static void udma_tiva_isr_error(const struct device *dev) {
	const uint32_t s = uDMAErrorStatusGet();
    if(s) {
        uDMAErrorStatusClear();
    }
}

static void udma_tiva_isr_software(const struct device *dev) {

}

static int udma_tiva_init(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	sysctl_activatePeripheral(base);
	memset(udmaControlTable, 0, sizeof(udmaControlTable));
	uDMAEnable();
	uDMAControlBaseSet(udmaControlTable);
	return 0;
}

void udma_channelAssign(const uint32_t channelAssignValue) {
	uDMAChannelAssign(channelAssignValue);
}

void udma_channelSetupStaticCfg(const uint32_t channel, const bool srcInc, const bool destInc, const bool useBurst) {
	uDMAChannelAttributeDisable(channel,
		UDMA_ATTR_ALTSELECT |
		UDMA_ATTR_HIGH_PRIORITY |
		UDMA_ATTR_REQMASK
	);

	if (true ==  useBurst) {
		uDMAChannelAttributeEnable(channel, UDMA_ATTR_USEBURST);
	}
	uDMAChannelControlSet(channel | UDMA_PRI_SELECT,
		UDMA_SIZE_8 |
		(false == srcInc) ? UDMA_SRC_INC_NONE : UDMA_SRC_INC_8 |
		(false == destInc) ? UDMA_DST_INC_NONE : UDMA_DST_INC_8 |
		UDMA_ARB_4
	);
}

void udma_channelSetupDyn(const uint32_t channel, void * src, void * dst, const uint32_t N) {
	uDMAChannelTransferSet(channel | UDMA_PRI_SELECT, UDMA_MODE_BASIC, src, dst, N);
}

void udma_channelStartTransfer(const uint32_t channel) {
	uDMAChannelEnable(channel);
}

bool udma_channelIsActive(const uint32_t channel) {
	return uDMAChannelIsEnabled(channel);
}

#define TIVA_UDMA_DEVICE(n)											\
	static const struct udma_tiva_config udma_## n ##_tiva_cfg={	\
		.base = DT_INST_REG_ADDR(n),								\
	};																\
	static int udma_## n ##_tiva_init(const struct device *dev) {	\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,0, irq), 					\
				DT_INST_IRQ_BY_IDX(n,0, priority), 					\
				udma_tiva_isr_software, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,0, irq)); 					\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n,1, irq), 					\
				DT_INST_IRQ_BY_IDX(n,1, priority), 					\
				udma_tiva_isr_error, DEVICE_DT_INST_GET(n), 0); 	\
		irq_enable(DT_INST_IRQ_BY_IDX(n,1, irq)); 					\
		return udma_tiva_init(dev);									\
	};																\
	DEVICE_DT_INST_DEFINE(n,										\
		udma_## n ##_tiva_init,										\
		NULL,														\
		NULL,														\
		&udma_## n ##_tiva_cfg,										\
		POST_KERNEL, 50,											\
		NULL);														\


DT_INST_FOREACH_STATUS_OKAY(TIVA_UDMA_DEVICE)
