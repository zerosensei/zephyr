/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief
 *
 * A common driver for CH32 pinmux.
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <soc.h>

#include <drivers/pinmux.h>
#include <gpio/gpio_ch32.h>
#include <drivers/clock_control/ch32_clock_control.h>
#include <pinmux/pinmux_ch32.h>

const struct device * const gpio_ports[CH32_PORTS_MAX] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioa)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiob)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioc)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiod)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioe)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiof)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiog)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioh)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioi)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioj)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiok)),
};

static int ch32_pin_configure(uint32_t pin, uint32_t func, uint32_t altf)
{
	const struct device *port_device;

	if (CH32_PORT(pin) >= CH32_PORTS_MAX) {
		return -EINVAL;
	}

	port_device = gpio_ports[CH32_PORT(pin)];

	if ((port_device == NULL) || (!device_is_ready(port_device))) {
		return -ENODEV;
	}

	return gpio_ch32_configure(port_device, CH32_PIN(pin), func, altf);
}

/**
 * @brief helper for converting dt ch32 pinctrl format to existing pin config
 *        format
 *
 * @param *pinctrl pointer to soc_gpio_pinctrl list
 * @param list_size list size
 * @param base device base register value
 *
 * @return 0 on success, -EINVAL otherwise
 */
int ch32_dt_pinctrl_configure(const struct soc_gpio_pinctrl *pinctrl,
			       size_t list_size, uint32_t base)
{
	uint32_t pin, mux;
	uint32_t func = 0;
	int ret = 0;

	if (!list_size) {
		/* Empty pinctrl. Exit */
		return 0;
	}

	ARG_UNUSED(base);

	for (int i = 0; i < list_size; i++) {
		mux = pinctrl[i].pinmux;

		if (CH32_DT_PINMUX_FUNC(mux) < CH32_ANALOG) {
			func = pinctrl[i].pincfg | CH32_MODER_ALT_MODE;
		} else if (CH32_DT_PINMUX_FUNC(mux) == CH32_ANALOG) {
			func = CH32_MODER_ANALOG_MODE;
		} else {
			/* Not supported */
			__ASSERT_NO_MSG(CH32_DT_PINMUX_FUNC(mux));
		}

		pin = CH32PIN(CH32_DT_PINMUX_PORT(mux),
			       CH32_DT_PINMUX_LINE(mux));

		ret = ch32_pin_configure(pin, func, CH32_DT_PINMUX_FUNC(mux));
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}
