/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_RISCV_RISCV_PRIVILEGE_CH32V_PINCTRL_SOC_H
#define ZEPHYR_SOC_RISCV_RISCV_PRIVILEGE_CH32V_PINCTRL_SOC_H

#include <devicetree.h>
#include <zephyr/types.h>
#include <dt-bindings/pinctrl/ch32-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

/** Type for CH32 pin. */
typedef struct pinctrl_soc_pin {
	/** Pinmux settings (port, pin and function). */
	uint32_t pinmux;
	/** Pin configuration (bias, drive and slew rate). */
	uint32_t pincfg;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pinmux field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_CH32_PINMUX_INIT(node_id) DT_PROP(node_id, pinmux)

/**
 * @brief Definitions used to initialize fields in #pinctrl_pin_t
 */
#define CH32_NO_PULL     0x0
#define CH32_PULL_UP     0x1
#define CH32_PULL_DOWN   0x2
#define CH32_PUSH_PULL   0x0
#define CH32_OPEN_DRAIN  0x1
#define CH32_OUTPUT_LOW  0x0
#define CH32_OUTPUT_HIGH 0x1

/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_CH32_PINCFG_INIT(node_id)				       \
	(((CH32_NO_PULL * DT_PROP(node_id, bias_disable)) << CH32_PUPDR_SHIFT) | \
	 ((CH32_PULL_UP * DT_PROP(node_id, bias_pull_up)) << CH32_PUPDR_SHIFT) | \
	 ((CH32_PULL_DOWN * DT_PROP(node_id, bias_pull_down)) << CH32_PUPDR_SHIFT) | \
	 ((CH32_PUSH_PULL * DT_PROP(node_id, drive_push_pull)) << CH32_OTYPER_SHIFT) | \
	 ((CH32_OPEN_DRAIN * DT_PROP(node_id, drive_open_drain)) << CH32_OTYPER_SHIFT) | \
	 ((CH32_OUTPUT_LOW * DT_PROP(node_id, output_low)) << CH32_ODR_SHIFT) | \
	 ((CH32_OUTPUT_HIGH * DT_PROP(node_id, output_high)) << CH32_ODR_SHIFT) | \
	 (DT_ENUM_IDX(node_id, slew_rate) << CH32_OSPEEDR_SHIFT))


/**
 * @brief Utility macro to initialize each pin.
 *
 * @param node_id Node identifier.
 * @param state_prop State property name.
 * @param idx State property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, state_prop, idx)		       \
	{ .pinmux = Z_PINCTRL_CH32_PINMUX_INIT(			       \
		DT_PROP_BY_IDX(node_id, state_prop, idx)),		       \
	  .pincfg = Z_PINCTRL_CH32_PINCFG_INIT(			       \
		DT_PROP_BY_IDX(node_id, state_prop, idx)) },

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_RISCV_RISCV_PRIVILEGE_CH32V_PINCTRL_SOC_H */
