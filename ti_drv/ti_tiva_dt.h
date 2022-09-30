/*
 * Copyright (c) 2021 Stefan Jaritz, TIER SE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _TI_TIVA_DT_H_
#define _TI_TIVA_DT_H_

#include <devicetree.h>
#include <stdint.h>

typedef struct ti_tiva_gpio_pinctrl {
	uint32_t portAddr;
	uint8_t pinNum;
	uint32_t pinmapDef;
	uint8_t unlock;
	uint8_t dirmodeIndx;
	uint8_t strengthIndx;
	uint8_t pintypeIndx;
} ti_tiva_gpio_pinctrl_t;

#define TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i) DT_INST_PHANDLE_BY_IDX(inst, pinctrl_##x, i)

#define TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i) DT_PHANDLE_BY_IDX(DT_NODELABEL(name), pinctrl_##x, i)

#define TI_TIVA_DT_INST_PORT(inst, x, i) DT_PROP_BY_PHANDLE(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), port, reg_IDX_0)
#define TI_TIVA_DT_PORT(name, x, i) DT_PROP_BY_PHANDLE(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(inst, x, i), port, reg_IDX_0)

#define TI_TIVA_DT_INST_PIN(inst, x, i) DT_PROP(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), pin)
#define TI_TIVA_DT_PIN(name, x, i) DT_PROP(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i), pin)

#define TI_TIVA_DT_INST_MAP(inst, x, i) DT_PROP(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), map)
#define TI_TIVA_DT_MAP(name, x, i) DT_PROP(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i), map)

#define TI_TIVA_DT_INST_UNLOCK(inst, x, i) DT_PROP(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), unlock)
#define TI_TIVA_DT_UNLOCK(name, x, i) DT_PROP(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i), unlock)

#define TI_TIVA_DT_INST_DIRMODE(inst, x, i) DT_ENUM_IDX(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), dirmode)
#define TI_TIVA_DT_DIRMODE(name, x, i) DT_ENUM_IDX(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i), dirmode)

#define TI_TIVA_DT_INST_STRENGTH(inst, x, i) DT_ENUM_IDX(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), strength)
#define TI_TIVA_DT_STRENGTH(name, x, i) DT_ENUM_IDX(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i), strength)

#define TI_TIVA_DT_INST_TYPE(inst, x, i) DT_ENUM_IDX(TI_TIVA_DT_INST_NODE_ID_FROM_PINCTRL(inst, x, i), type)
#define TI_TIVA_DT_TYPE(name, x, i) DT_ENUM_IDX(TI_TIVA_DT_NODE_ID_FROM_PINCTRL(name, x, i), type)

#define TI_TIVA_DT_INST_PIN_ELEM(i, x, inst)			\
	{							\
		.portAddr=TI_TIVA_DT_INST_PORT(inst, x, i),		\
		.pinNum=TI_TIVA_DT_INST_PIN(inst, x, i),		\
		.pinmapDef=TI_TIVA_DT_INST_MAP(inst, x, i),		\
		.unlock=TI_TIVA_DT_INST_UNLOCK(inst, x, i),		\
		.dirmodeIndx=TI_TIVA_DT_INST_DIRMODE(inst, x, i),		\
		.strengthIndx=TI_TIVA_DT_INST_STRENGTH(inst, x, i),		\
		.pintypeIndx=TI_TIVA_DT_INST_TYPE(inst, x, i),		\
	},

#define TI_TIVA_DT_PIN_ELEM(i, x, name)		\
	{						\
		.portAddr=TI_TIVA_DT_PORT(name, x, i),		\
		.pinNum=TI_TIVA_DT_PIN(name, x, i),		\
		.pinmapDef=TI_TIVA_DT_MAP(name, x, i),		\
		.unlock=TI_TIVA_DT_UNLOCK(name, x, i),		\
		.dirmodeIndx=TI_TIVA_DT_DIRMODE(name, x, i),		\
		.strengthIndx=TI_TIVA_DT_STRENGTH(name, x, i),		\
		.pintypeIndx=TI_TIVA_DT_TYPE(name, x, i),		\
	},


#define TI_TIVA_DT_INST_NUM_PINS(inst, x) DT_INST_PROP_LEN(inst, pinctrl_##x)

#define TI_TIVA_DT_NUM_PINS(name, x) DT_PROP_LEN(DT_NODELABEL(name), pinctrl_##x)

#define TI_TIVA_DT_INST_PINCTRL(inst, x)				\
	{ COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pinctrl_##x),		\
		      (UTIL_LISTIFY(TI_TIVA_DT_INST_NUM_PINS(inst, x),	\
				   TI_TIVA_DT_INST_PIN_ELEM,		\
				   x,					\
				   inst)),				\
		      ())						\
	}

#define TI_TIVA_DT_PINCTRL(name, x)					\
	{ COND_CODE_1(DT_NODE_HAS_PROP(DT_NODELABEL(name), pinctrl_##x),\
		      (UTIL_LISTIFY(TI_TIVA_DT_NUM_PINS(name, x),	\
				   TI_TIVA_DT_PIN_ELEM,		\
				   x,					\
				   name)),				\
		      ())						\
	}



#define TI_TIVA_DT_SINGLE_PINMUX(name)		\
	{						\
		.portAddr=DT_PROP_BY_PHANDLE(DT_NODELABEL(name),port, reg_IDX_0),		\
		.pinNum=DT_PROP(DT_NODELABEL(name),pin),		\
		.pinmapDef=DT_PROP(DT_NODELABEL(name),map),		\
		.unlock=DT_PROP(DT_NODELABEL(name),unlock),		\
		.dirmodeIndx=DT_ENUM_IDX(DT_NODELABEL(name),dirmode),		\
		.strengthIndx=DT_ENUM_IDX(DT_NODELABEL(name),strength),		\
		.pintypeIndx=DT_ENUM_IDX(DT_NODELABEL(name),type),		\
	}


#endif /* _TI_TIVA_DT_H_ */
