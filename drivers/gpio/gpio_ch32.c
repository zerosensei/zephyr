/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_gpio

#include <drivers/gpio.h>
#include <drivers/clock_control/ch32_clock_control.h>
#include <dt-bindings/gpio/gpio.h>
#include <dt-bindings/pinctrl/ch32-pinctrl.h>
#include "gpio_utils.h"
#include <ch32v30x.h>

struct gpio_ch32_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	const struct device *dev;

	sys_slist_t callbacks;
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
 * @brief Common gpio flags to custom flags
 */
static int gpio_ch32_flags_to_conf(int flags, int *pincfg)
{

	if(flags & GPIO_OUTPUT) {
		/* Output only or Output/Input */

		if(flags & GPIO_SINGLE_ENDED) {
			if (flags & GPIO_LINE_OPEN_DRAIN) {
				*pincfg = GPIO_Mode_Out_OD;
			} else  {
				/* Output can't be open source */
				return -ENOTSUP;
			}
		} else {
			*pincfg = GPIO_Mode_Out_PP;
		}

		if(flags & GPIO_PULL_UP || flags & GPIO_PULL_DOWN){
			return -ENOTSUP;
		}

	} else if(flags & GPIO_INPUT) {
		/* Input */

		if ((flags & GPIO_PULL_UP) != 0) {
			*pincfg = GPIO_Mode_IPU;
		} else if ((flags & GPIO_PULL_DOWN) != 0) {
			*pincfg = GPIO_Mode_IPD;
		} else {
			*pincfg = GPIO_Mode_IN_FLOATING;
		}
	} else {
		/* Deactivated: Analog */
		*pincfg = GPIO_Mode_AIN;
	}

	return 0;
}

static inline uint32_t ch32_pinval_get(int pin)
{
	uint32_t pinval;

	pinval = 1 << pin;

	return pinval;
}

static int gpio_ch32_pin_configure(const struct device *port, gpio_pin_t pin, 
					gpio_flags_t flags)
{
	const struct gpio_ch32_cfg *cfg = port->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	int pincfg = GPIO_Mode_AIN;

	gpio_ch32_flags_to_conf(flags, &pincfg);

	/* enable clock */
	if(clock_control_on(port,  
			(clock_control_subsys_t *)&cfg->pclken) != 0) {
		return -EIO; 
	}

	if(flags & GPIO_OUTPUT) {
		if(flags & GPIO_OUTPUT_INIT_HIGH) {
			// gpio_stm32_port_set_bits_raw(dev, BIT(pin));
		} else if(flags & GPIO_OUTPUT_INIT_LOW) {
			// pio_stm32_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	int pin_ll = ch32_pinval_get(pin);
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	GPIO_InitStructure.GPIO_Pin = pin_ll;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = pincfg;

	GPIO_Init(gpio, &GPIO_InitStructure);

	return 0;
}

static int gpio_ch32_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_ch32_cfg *cfg = dev->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;

	*value = (uint32_t)GPIO_ReadInputData(gpio);

	return 0;
}

static int gpio_ch32_port_set_masked_raw(const struct device *dev,
					  gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct gpio_ch32_cfg *cfg = dev->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	uint32_t port_value;

	port_value = GPIO_ReadOutputData(gpio);
	GPIO_Write(gpio, (port_value & ~mask) | (mask & value));

	return 0;
}

static int gpio_ch32_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins)
{
	const struct gpio_ch32_cfg *cfg = dev->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;

	GPIO_SetBits(gpio, pins);

	return 0;
}

static int gpio_ch32_port_clear_bits_raw(const struct device *dev,
					  gpio_port_pins_t pins)
{
	const struct gpio_ch32_cfg *cfg = dev->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;

	GPIO_ResetBits(gpio, pins);

	return 0;
}

static int gpio_ch32_port_toggle_bits(const struct device *dev,
				       gpio_port_pins_t pins)
{
	const struct gpio_ch32_cfg *cfg = dev->config;
	GPIO_TypeDef *gpio = (GPIO_TypeDef *)cfg->base;
	uint32_t value;

	value = (uint32_t)GPIO_ReadOutputData(gpio);
	GPIO_Write(gpio, value ^ pins);

	return 0;
}


static const struct gpio_driver_api gpio_ch32_driver = {
	.pin_configure = gpio_ch32_pin_configure,
	.port_get_raw = gpio_ch32_port_get_raw,
	.port_set_masked_raw = gpio_ch32_port_set_masked_raw,
	.port_set_bits_raw = gpio_ch32_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ch32_port_clear_bits_raw,
	.port_toggle_bits = gpio_ch32_port_toggle_bits,
	// .pin_interrupt_configure = gpio_ch32_pin_interrupt_configure,
	// .manage_callback = gpio_ch32_manage_callback,
};


static int gpio_ch32_init(const struct device *dev)
{
	int ret;
	struct gpio_ch32_data *data = dev->data;
	const struct gpio_ch32_cfg *cfg = dev->config;

	data->dev = dev;

	const struct device *clk = DEVICE_DT_GET(CH32_CLOCK_CONTROL_NODE);

	ret = clock_control_on(clk,
				(clock_control_subsys_t *)&cfg->pclken);

	return ret;

}

#define GPIO_DEVICE_INIT(__node, __suffix, __base_addr, __port, __cenr, __bus) \
	static const struct gpio_ch32_cfg gpio_ch32_cfg_## __suffix = {   \
		.common = {						       \
			 .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(16U), \
		},							       \
		.base = (uint32_t *)__base_addr,				       \
		.port = __port,						       \
		.pclken = { .bus = __bus, .enr = __cenr }		       \
	};								       \
	static struct gpio_ch32_data gpio_ch32_data_## __suffix;	       \
		       \
	DEVICE_DT_DEFINE(__node,					       \
			    gpio_ch32_init,				       \
			    NULL,			       \
			    &gpio_ch32_data_## __suffix,		       \
			    &gpio_ch32_cfg_## __suffix,		       \
			    PRE_KERNEL_1,				       \
			    CONFIG_GPIO_INIT_PRIORITY,			       \
			    &gpio_ch32_driver)

#define GPIO_DEVICE_INIT_CH32(__suffix, __SUFFIX)			\
	GPIO_DEVICE_INIT(DT_NODELABEL(gpio##__suffix),	\
			 __suffix,					\
			 DT_REG_ADDR(DT_NODELABEL(gpio##__suffix)),	\
			 CH32_PORT##__SUFFIX,				\
			 DT_CLOCKS_CELL(DT_NODELABEL(gpio##__suffix), bits),\
			 DT_CLOCKS_CELL(DT_NODELABEL(gpio##__suffix), bus))

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay)
GPIO_DEVICE_INIT_CH32(a, A);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioa), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay)
GPIO_DEVICE_INIT_CH32(b, B);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiob), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay)
GPIO_DEVICE_INIT_CH32(c, C);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioc), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay)
GPIO_DEVICE_INIT_CH32(d, D);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpiod), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay)
GPIO_DEVICE_INIT_CH32(e, E);
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(gpioe), okay) */

