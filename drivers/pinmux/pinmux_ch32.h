/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file header for CH32 pin multiplexing
 */

#ifndef ZEPHYR_DRIVERS_PINMUX_PINMUX_CH32_H
#define ZEPHYR_DRIVERS_PINMUX_PINMUX_CH32_H

#include <zephyr/types.h>
#include <drivers/clock_control.h>
#include <dt-bindings/pinctrl/ch32-pinctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief structure to convey pinctrl information for stm32 soc
 * value
 */
struct soc_gpio_pinctrl {
	uint32_t pinmux;
	uint32_t pincfg;
};

/**
 * @brief helper to extract IO port number from CH32_PINMUX() encoded
 * value
 */
#define CH32_DT_PINMUX_PORT(__pin) \
	(((__pin) >> CH32_PORT_SHIFT) & CH32_PORT_MASK)

/**
 * @brief helper to extract IO pin number from CH32_PINMUX() encoded
 * value
 */
#define CH32_DT_PINMUX_LINE(__pin) \
	(((__pin) >> CH32_LINE_SHIFT) & CH32_LINE_MASK)

/**
 * @brief helper to extract IO pin func from CH32_PINMUX() encoded
 * value
 */
#define CH32_DT_PINMUX_FUNC(__pin) \
	(((__pin) >> CH32_MODE_SHIFT) & CH32_MODE_MASK)

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl)
/**
 * @brief helper to extract IO pin remap from CH32_PINMUX() encoded
 * value
 */
#define CH32_DT_PINMUX_REMAP(__pin) \
	(((__pin) >> CH32_REMAP_SHIFT) & CH32_REMAP_MASK)
#endif

/**
 * @brief helper to extract IO port number from CH32PIN() encoded
 * value
 */
#define CH32_PORT(__pin) \
	((__pin) >> 4)

/**
 * @brief helper to extract IO pin number from CH32PIN() encoded
 * value
 */
#define CH32_PIN(__pin) \
	((__pin) & 0xf)

/**
 * @brief helper for converting dt stm32 pinctrl format to existing pin config
 *        format
 *
 * @param *pinctrl pointer to soc_gpio_pinctrl list
 * @param list_size list size
 * @param base device base register value
 *
 * @return 0 on success, -EINVAL otherwise
 */
int ch32_dt_pinctrl_configure(const struct soc_gpio_pinctrl *pinctrl,
			       size_t list_size, uint32_t base);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl)
/**
 * @brief Helper function to check and apply provided pinctrl remap
 *        configuration
 *
 * Check operation verifies that pin remapping configuration is the same on all
 * pins. If configuration is valid AFIO clock is enabled and remap is applied
 *
 * @param *pinctrl pointer to soc_gpio_pinctrl list
 * @param list_size list size
 *
 * @return 0 value on success, -EINVAL otherwise
 */
int ch32_dt_pinctrl_remap(const struct soc_gpio_pinctrl *pinctrl,
			   size_t list_size);
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl) */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_PINMUX_PINMUX_CH32_H */
