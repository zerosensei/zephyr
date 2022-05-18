/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_pinctrl

#include <errno.h>
#include <drivers/clock_control/ch32_clock_control.h>
#include <drivers/pinctrl.h>
#include <pinmux/pinmux_ch32.h>

/**
 * @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static const struct device * const gpio_ports[] = {
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioa)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiob)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioc)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiod)),
	DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioe)),
};

/** Number of GPIO ports. */
static const size_t gpio_ports_cnt = ARRAY_SIZE(gpio_ports);


static int ch32_pin_configure(uint32_t pin, uint32_t pin_cgf, uint32_t pin_func)
{
	const struct device *port_device;

	if (CH32_PORT(pin) >= gpio_ports_cnt) {
		return -EINVAL;
	}

	port_device = gpio_ports[CH32_PORT(pin)];

	if ((port_device == NULL) || (!device_is_ready(port_device))) {
		return -ENODEV;
	}

	return gpio_ch32_configure(port_device, CH32_PIN(pin), pin_cgf, pin_func);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	uint32_t pin, mux;
	uint32_t pin_cgf = 0;
	int ret = 0;

	ARG_UNUSED(reg);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl)
	ret = stm32_pins_remap(pins, pin_cnt);
	if (ret < 0) {
		return ret;
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl) */

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		mux = pins[i].pinmux;

#if DT_HAS_COMPAT_STATUS_OKAY(wch_ch32_pinctrl)
		uint32_t pupd;

		if (CH32_DT_PINMUX_FUNC(mux) == ALTERNATE) {
			pin_cgf = pins[i].pincfg | CH32_MODE_OUTPUT | CH32_CNF_ALT_FUNC;
		} else if (CH32_DT_PINMUX_FUNC(mux) == ANALOG) {
			pin_cgf = pins[i].pincfg | CH32_MODE_INPUT | CH32_CNF_IN_ANALOG;
		} else if (CH32_DT_PINMUX_FUNC(mux) == GPIO_IN) {
			pin_cgf = pins[i].pincfg | CH32_MODE_INPUT;
			pupd = pin_cgf & (CH32_PUPD_MASK << CH32_PUPD_SHIFT);
			if (pupd == CH32_PUPD_NO_PULL) {
				pin_cgf = pin_cgf | CH32_CNF_IN_FLOAT;
			} else {
				pin_cgf = pin_cgf | CH32_CNF_IN_PUPD;
			}
		} else if (CH32_DT_PINMUX_FUNC(mux) == GPIO_OUT) {
			pin_cgf = pins[i].pincfg | CH32_MODE_OUTPUT | CH32_CNF_GP_OUTPUT;
		} else {
			/* Not supported */
			__ASSERT_NO_MSG(CH32_DT_PINMUX_FUNC(mux));
		}
#else
		if (CH32_DT_PINMUX_FUNC(mux) < CH32_ANALOG) {
			pin_cgf = pins[i].pincfg | CH32_MODER_ALT_MODE;
		} else if (CH32_DT_PINMUX_FUNC(mux) == CH32_ANALOG) {
			pin_cgf = CH32_MODER_ANALOG_MODE;
		} else if (CH32_DT_PINMUX_FUNC(mux) == CH32_GPIO) {
			uint32_t gpio_out = pins[i].pincfg &
						(CH32_ODR_MASK << CH32_ODR_SHIFT);
			if (gpio_out != 0) {
				pin_cgf = pins[i].pincfg | CH32_MODER_OUTPUT_MODE;
			} else {
				pin_cgf = pins[i].pincfg | CH32_MODER_INPUT_MODE;
			}
		} else {
			/* Not supported */
			__ASSERT_NO_MSG(CH32_DT_PINMUX_FUNC(mux));
		}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32f1_pinctrl) */

		pin = CH32PIN(CH32_DT_PINMUX_PORT(mux),
			       CH32_DT_PINMUX_LINE(mux));

		ret = stm32_pin_configure(pin, pin_cgf, CH32_DT_PINMUX_FUNC(mux));
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}