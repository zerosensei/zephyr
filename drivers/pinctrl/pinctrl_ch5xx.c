/*
 * Copyright (c) 2024 zerosensei
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_pinctrl

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#define REG_PIN_ALTERNATE				\
		(*((uint16_t *)DT_INST_REG_ADDR_BY_NAME(0, PIN_ALTERNATE)))

#define REG_PIN_ANALOG_IE			\
		(*((uint16_t *)DT_INST_REG_ADDR_BY_NAME(0, PIN_ANALOG)))

const struct device *gpio[] = {
    DEVICE_DT_GET(DT_NODELABEL(gpioa)),
    DEVICE_DT_GET(DT_NODELABEL(gpiob))
};

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t regval;

	if (pin->port >= ARRAY_SIZE(gpio) || !device_is_ready(gpio[pin->port])) {
		return;
	}

	if (pin->remap_bit) {
		regval = REG_PIN_ALTERNATE;
		if (pin->remap_en) {
			regval |= BIT(pin->remap_bit);
		} else {
			regval &= ~BIT(pin->remap_bit);
		}
        REG_PIN_ALTERNATE = regval;
	}

	gpio_pin_configure(gpio[pin->port], pin->pin, pin->flags);
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}