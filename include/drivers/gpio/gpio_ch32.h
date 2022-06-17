/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_CH32_H
#define ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_CH32_H

/**
 * @file header for CH32 GPIO
 */

#include <drivers/clock_control/ch32_clock_control.h>
#include <drivers/gpio.h>


/**
 * @brief structure to convey pinctrl information for stm32 soc
 * value
 */
struct soc_gpio_pinctrl {
	uint32_t pinmux;
	uint32_t pincfg;
};


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

/**
 * @brief configuration of GPIO device
 */
struct gpio_ch32_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	const struct device *dev;

	sys_slist_t cb;
};

struct gpio_ch32_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* port base address */
	uint32_t *base;
	/* IO port */
	int port;
	struct ch32_pclken pclken;
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param dev GPIO port device pointer
 * @param pin IO pin
 * @param conf GPIO mode
 * @param func Pin function
 *
 * @return 0 on success, negative errno code on failure
 */
int gpio_ch32_configure(const struct device *dev, int pin, int conf, int func);

#endif /* ZEPHYR_INCLUDE_DRIVERS_GPIO_GPIO_CH32_H */
