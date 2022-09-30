/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tiva_wdt
#include <string.h>

#include <kernel.h>
#include <drivers/watchdog.h>

#include "cfg_tiva.h"
#include "sysctl_tiva.h"
#include "ti_tiva_dt.h"
#include "wdt_tiva.h"

struct wdt_tiva_config {
	const uint32_t base;
	const uint32_t intrNumber;
};

struct wdt_tiva_runtime {
	int32_t feedcounter;
	uint32_t reloadValue;
	wdt_tiva_isr_funcCB_pfkt isrCB;
};

#define DEV_CFG(dev) ((const struct wdt_tiva_config *const)(dev)->config)

#define DEV_DATA(dev) ((struct wdt_tiva_runtime *const)(dev)->data)

static void wdt_tiva_isr(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct wdt_tiva_runtime * const data = DEV_DATA(dev);
	// if we don't clear the int the MCU goes to reset
	//WatchdogIntClear(base);

	if (data->isrCB) data->isrCB();
}

static int wdt_tiva_set_config(const struct device *dev, uint8_t options) {
	struct wdt_tiva_runtime * const data = DEV_DATA(dev);
	const uint32_t base = DEV_CFG(dev)->base;
	const uint32_t intrNumber = DEV_CFG(dev)->intrNumber;

	WatchdogIntEnable(base);
	// irq_enable(intrNumber);
	WatchdogResetEnable(base);
	WatchdogEnable(base);
	return 0;
}

static int wdt_tiva_disable(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	const uint32_t intrNumber = DEV_CFG(dev)->intrNumber;
	// irq_disable(intrNumber);
	WatchdogResetDisable(base);
	WatchdogIntClear(base);
	return 0;
}

static int wdt_tiva_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg) {
	const uint32_t base = DEV_CFG(dev)->base;

	struct wdt_tiva_runtime * const data = DEV_DATA(dev);

	if (cfg->flags != WDT_FLAG_RESET_SOC) {
		return -ENOTSUP;
	}

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		return -EINVAL;
	}

	data->feedcounter = 2;
	data->reloadValue = (SysCtlClockGet()/1000) * cfg->window.max;
	WatchdogReloadSet(base, data->reloadValue);
	return 0;
}

static int wdt_tiva_feed(const struct device *dev, int channel_id)
{
	const uint32_t base = DEV_CFG(dev)->base;
	struct wdt_tiva_runtime * const data = DEV_DATA(dev);

	data->feedcounter++;
	// WatchdogIntClear(base);
	WatchdogReloadSet(base, data->reloadValue);
	return 0;
}

static int wdt_tiva_init(const struct device *dev) {
	const uint32_t base = DEV_CFG(dev)->base;
	struct wdt_tiva_runtime * const data = DEV_DATA(dev);
	memset(data, 0 , sizeof(struct wdt_tiva_runtime));
	data->isrCB = NULL;
	sysctl_activatePeripheral(base);

	// for debugging
	WatchdogStallEnable(base);
	return 0;
}

static const struct wdt_driver_api api = {
	.setup = wdt_tiva_set_config,
	.disable = wdt_tiva_disable,
	.install_timeout = wdt_tiva_install_timeout,
	.feed = wdt_tiva_feed
};

void wdt_tiva_installISRcb(const struct device *dev, wdt_tiva_isr_funcCB_pfkt isrCB) {
	struct wdt_tiva_runtime * const data = DEV_DATA(dev);
	data->isrCB = isrCB;
}

#define TIVA_WDT_DEVICE(n)										\
	static const struct wdt_tiva_config wdt_## n ##_tiva_cfg={	\
		.base = DT_INST_REG_ADDR(n),							\
		.intrNumber = DT_INST_IRQN(n),							\
	};															\
	static struct wdt_tiva_runtime wdt_## n ##_tiva_runtime;	\
	static int wdt_## n ##_tiva_init(const struct device *dev) {\
		IRQ_CONNECT(DT_INST_IRQN(n),							\
			DT_INST_IRQ(n, priority),							\
			wdt_tiva_isr,										\
			DEVICE_DT_INST_GET(n), 0);							\
		irq_enable(DT_INST_IRQN(n));							\
		return wdt_tiva_init(dev);								\
	};															\
	DEVICE_DT_INST_DEFINE(n,									\
		wdt_## n ##_tiva_init,									\
		NULL,													\
		&wdt_## n ##_tiva_runtime,								\
		&wdt_## n ##_tiva_cfg,									\
		POST_KERNEL, 50,										\
		&api);													\


DT_INST_FOREACH_STATUS_OKAY(TIVA_WDT_DEVICE)
