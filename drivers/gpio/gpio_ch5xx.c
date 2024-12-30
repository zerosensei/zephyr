/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_gpio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include "gpio_ch5xx.h"

typedef struct {
    volatile uint32_t DIR;
    volatile uint32_t PIN;
    volatile uint32_t OUT;
    volatile uint32_t CLR;
    volatile uint32_t PU;
    volatile uint32_t PD_DRV;
} WCH_GPIO_Type;

struct gpio_ch5xx_config {
    /* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
    uint16_t *int_en;
    uint16_t *int_mode;
    uint16_t *int_if;
	/* port base address */
	WCH_GPIO_Type *base;
};

struct gpio_ch5xx_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* user ISR cb */
	sys_slist_t callbacks;
};

static inline void hal_gpio_pins_reset(WCH_GPIO_Type *port, uint32_t pins)
{
    port->CLR |= pins;
}

static inline void hal_gpio_pins_set(WCH_GPIO_Type *port, uint32_t pins)
{
    port->OUT |= pins;
}

static inline void hal_gpio_pins_invert(WCH_GPIO_Type *port, uint32_t pins)
{
    port->OUT ^= pins;
}

static inline uint32_t hal_gpio_port_get(WCH_GPIO_Type *port)
{
    return port->PIN;
}

static inline uint32_t hal_gpio_pins_get(WCH_GPIO_Type *port, uint32_t pins)
{
    return port->PIN & (pins);
}

static uint32_t hal_gpio_int_flag_port_get(const struct device *port)
{
    const struct gpio_ch5xx_config *cfg = port->config;

    if (port == DEVICE_DT_GET(DT_NODELABEL(gpioa))) {
        return *(cfg->int_if) & BIT_MASK(16);
    } else if (port == DEVICE_DT_GET(DT_NODELABEL(gpiob))) {
        return ((*(cfg->int_if) & (~((GPIO_Pin_22 | GPIO_Pin_23) >> 14))) 
                    | ((*(cfg->int_if) << 14) & (GPIO_Pin_22 | GPIO_Pin_23)));
    } else {
        return 0;
    }
}

static void hal_gpio_int_flag_pins_clear(const struct device *port, 
            uint32_t pins)
{
    uint16_t pins_int = 0;
    const struct gpio_ch5xx_config *cfg = port->config;

    if (port == DEVICE_DT_GET(DT_NODELABEL(gpioa))) {
        pins_int = pins & BIT_MASK(16);
    } else if (port == DEVICE_DT_GET(DT_NODELABEL(gpiob))) {
        pins_int = (pins | ((pins & (GPIO_Pin_22 | GPIO_Pin_23)) >> 14)) & BIT_MASK(16);
    }

    *(cfg->int_if) = pins_int;
}

static void hal_gpio_mode_config(WCH_GPIO_Type *port, uint32_t pins, gpio_mode_t mode)
{
    switch (mode) {
    case GPIO_MODE_IN_FLOATING:
        port->PD_DRV &= ~pins;
        port->PU &= ~pins;
        port->DIR &= ~pins;
        break;
    case GPIO_MODE_IN_PU:
        port->PD_DRV &= ~pins;
        port->PU |= pins;
        port->DIR &= ~pins;
        break;
    case GPIO_MODE_IN_PD:
        port->PD_DRV |= pins;
        port->PU &= ~pins;
        port->DIR &= ~pins;
        break;
    case GPIO_MODE_OUT_PP_5MA:
        port->PD_DRV &= ~pins;
        port->DIR |= pins;
        break;  
    case GPIO_MODE_OUT_PP_20MA:
        port->PD_DRV |= pins;
        port->DIR |= pins;
        break;
    default:
        break;
    }
}

static void hal_gpio_int_mode_config(const struct device *port, uint32_t pins,
                         gpio_int_mode_t mode)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    uint16_t pins_int = 0;

    if (port == DEVICE_DT_GET(DT_NODELABEL(gpioa))) {
        pins_int = pins & BIT_MASK(16);
    } else if (port == DEVICE_DT_GET(DT_NODELABEL(gpiob))) {
        pins_int = (pins | ((pins & (GPIO_Pin_22 | GPIO_Pin_23)) >> 14)) 
                & BIT_MASK(16);
    }

    switch (mode) {
    case GPIO_IT_MODE_LOWLEVEL:
        cfg->base->CLR |= pins;
        *(cfg->int_mode) &= ~pins_int;
        *(cfg->int_en) |= pins_int;
        break;
    case GPIO_IT_MODE_HIGHLEVEL:
        cfg->base->OUT |= pins;
        *(cfg->int_mode) &= ~pins_int;
        *(cfg->int_en) |= pins_int;
        break;
    case GPIO_IT_MODE_FALLEDGE:
        cfg->base->CLR |= pins;
        *(cfg->int_mode) |= pins_int;
        *(cfg->int_en) |= pins_int;
        break;
    case GPIO_IT_MODE_RISEEDGE:
        cfg->base->OUT |= pins;
        *(cfg->int_mode) |= pins_int;
        *(cfg->int_en) |= pins_int;
        break;
    case GPIO_IT_MODE_DISABLE:
        *(cfg->int_en) &= ~pins_int;
        break;
    default:
        break;
    }

    *(cfg->int_if) = pins_int;
}

static int gpio_ch5xx_flags_to_cfg(uint32_t flag, int *pin_cfg)
{
    if ((flag & GPIO_OUTPUT) != 0) {
        if ((flag & GPIO_SINGLE_ENDED) != 0) {
            /* open drain or open source */
            return -ENOTSUP;
        } else {
            /* push-pull mode */
            *pin_cfg = GPIO_MODE_OUT_PP_5MA;
        }
    } else if ((flag & GPIO_INPUT) != 0) {
        if ((flag & GPIO_PULL_UP) != 0) {
            *pin_cfg = GPIO_MODE_IN_PU;
        } else if ((flag & GPIO_PULL_DOWN) != 0) {
            *pin_cfg = GPIO_MODE_IN_PD;
        } else {
            *pin_cfg = GPIO_MODE_IN_FLOATING;
        }
    } else {
        //TODO: analog deal
        // *pin_cfg = GPIO_MODE_ANALOG;
    }

    return 0;
}

static int gpio_ch5xx_port_get_raw(const struct device *port, 
                 gpio_port_value_t *value)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    WCH_GPIO_Type *gpio = (WCH_GPIO_Type *)cfg->base;

    *value = hal_gpio_port_get(gpio);

    return 0;
}

static int gpio_ch5xx_port_set_masked_raw(const struct device *port,
				   gpio_port_pins_t mask,
				   gpio_port_value_t value)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    WCH_GPIO_Type *gpio = (WCH_GPIO_Type *)cfg->base;
    uint32_t get_val;

    get_val = hal_gpio_port_get(gpio);

    hal_gpio_pins_set(gpio, (mask & value) | (get_val & ~mask));

    return 0;
}

static int gpio_ch5xx_port_set_bits_raw(const struct device *port,
				 gpio_port_pins_t pins)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    WCH_GPIO_Type *gpio = (WCH_GPIO_Type *)cfg->base;

    hal_gpio_pins_set(gpio, pins);

    return 0;
}

static int gpio_ch5xx_port_clear_bits_raw(const struct device *port,
				   gpio_port_pins_t pins)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    WCH_GPIO_Type *gpio = (WCH_GPIO_Type *)cfg->base;

    hal_gpio_pins_reset(gpio, pins);
    
    return 0;
}

static int gpio_ch5xx_port_toggle_bits(const struct device *port,
				gpio_port_pins_t pins)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    WCH_GPIO_Type *gpio = (WCH_GPIO_Type *)cfg->base;

    hal_gpio_pins_invert(gpio, pins);

    return 0;
}

static int gpio_ch5xx_pin_interrupt_configure(const struct device *port,
				       gpio_pin_t pin,
				       enum gpio_int_mode mode, enum gpio_int_trig trig)
{
    if (mode == GPIO_INT_MODE_DISABLED) {
        hal_gpio_int_mode_config(port, BIT(pin), GPIO_IT_MODE_DISABLE);
    } else if (mode == GPIO_INT_MODE_LEVEL) {
        if (trig == GPIO_INT_TRIG_LOW) {
            hal_gpio_int_mode_config(port, BIT(pin), GPIO_IT_MODE_LOWLEVEL);
        } else if (trig == GPIO_INT_TRIG_HIGH) {
            hal_gpio_int_mode_config(port, BIT(pin), GPIO_IT_MODE_HIGHLEVEL);
        } else {
            return -ENOTSUP;
        }
    } else if (mode == GPIO_INT_MODE_EDGE) {
        if (trig == GPIO_INT_TRIG_LOW) {
            hal_gpio_int_mode_config(port, BIT(pin), GPIO_IT_MODE_FALLEDGE);
        } else if (trig == GPIO_INT_TRIG_HIGH) {
            hal_gpio_int_mode_config(port, BIT(pin), GPIO_IT_MODE_RISEEDGE);
        } else {
            return -ENOTSUP;
        }
    } else {
        return -ENOTSUP;
    }

    return 0;
}

static int gpio_ch5xx_manage_callback(const struct device *port,
			       struct gpio_callback *cb,
			       bool set)
{
    struct gpio_ch5xx_data *data = port->data;

	return gpio_manage_callback(&data->callbacks, cb, set);

    return 0;
}

static uint32_t gpio_ch5xx_get_pending_int(const struct device *dev)
{

    return 0;
}

static int gpio_ch5xx_pin_configure(const struct device *port, gpio_pin_t pin,
			     gpio_flags_t flags)
{
    const struct gpio_ch5xx_config *cfg = port->config;
    WCH_GPIO_Type *gpio = (WCH_GPIO_Type *)cfg->base;
    int err;
    int pin_cfg = 0;

    err = gpio_ch5xx_flags_to_cfg(flags, &pin_cfg);

    if (err != 0) {
        return err;
    }

    if ((flags & GPIO_OUTPUT) != 0) {
        if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
            gpio_ch5xx_port_set_bits_raw(port, BIT(pin));
        } else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
            gpio_ch5xx_port_clear_bits_raw(port, BIT(pin));
        }
    }

    hal_gpio_mode_config(gpio, BIT(pin), pin_cfg);
    
    return 0;
}

static void ch5xx_gpio_isr(const struct device *dev)
{
    struct gpio_ch5xx_data *data = dev->data;
    uint32_t int_status;

    int_status = hal_gpio_int_flag_port_get(dev);

    hal_gpio_int_flag_pins_clear(dev, int_status);
	/* Call the registered callbacks */
	gpio_fire_callbacks(&data->callbacks, dev, int_status);
}

static const struct gpio_driver_api gpio_ch5xx_api = {
    .pin_configure = gpio_ch5xx_pin_configure,
    .port_get_raw = gpio_ch5xx_port_get_raw,
    .port_set_masked_raw = gpio_ch5xx_port_set_masked_raw,
    .port_set_bits_raw = gpio_ch5xx_port_set_bits_raw,
    .port_clear_bits_raw = gpio_ch5xx_port_clear_bits_raw,
    .port_toggle_bits = gpio_ch5xx_port_toggle_bits,
    .pin_interrupt_configure = gpio_ch5xx_pin_interrupt_configure,
    .manage_callback = gpio_ch5xx_manage_callback,
    .get_pending_int = gpio_ch5xx_get_pending_int,
};

#define GPIO_IRQ_HANDLER_CONNECT(node_id) \
	IRQ_CONNECT(DT_INST_IRQN(node_id), DT_INST_IRQ(node_id, priority), ch5xx_gpio_isr, \
		    DEVICE_DT_INST_GET(node_id), 0);        \
     irq_enable(DT_INST_IRQN(node_id));


static int gpio_ch5xx_init(const struct device *dev)
{
	DT_INST_FOREACH_STATUS_OKAY(GPIO_IRQ_HANDLER_CONNECT);

    for(int i = 0; i < 32; i++) {
        gpio_ch5xx_pin_configure(dev, i, GPIO_INPUT | GPIO_PULL_DOWN);
    }

    return 0;
}

#define GOIO_CH5XX_INIT(index)       \
    static struct gpio_ch5xx_config gpio_ch5xx_##index##_config = {       \
        .common = {     \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index),   \
        },      \
        .int_en = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(index, INT_EN),     \
        .int_mode = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(index, INT_MODE),       \
        .int_if = (uint16_t *)DT_INST_REG_ADDR_BY_NAME(index, INT_IF),     \
        .base = (WCH_GPIO_Type *)DT_INST_REG_ADDR_BY_NAME(index, BASE),      \
    };      \
    static struct gpio_ch5xx_data gpio_ch5xx_##index##_data;       \
	DEVICE_DT_INST_DEFINE(index, gpio_ch5xx_init,		     \
			NULL, &gpio_ch5xx_##index##_data,			     \
			&gpio_ch5xx_##index##_config,			     \
			PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,		     \
			&gpio_ch5xx_api);

DT_INST_FOREACH_STATUS_OKAY(GOIO_CH5XX_INIT)
