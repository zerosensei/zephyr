/*
 * Copyright (c) 2022 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_CH32_PINCTRL_H
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_CH32_PINCTRL_H

/**
 * @brief numerical IDs for IO ports
 */

#define	CH32_PORTA 0	/* IO port A */
#define	CH32_PORTB 1	/* .. */
#define	CH32_PORTC 2
#define	CH32_PORTD 3
#define	CH32_PORTE 4

#define	CH32_PORTK 5	/* IO port K */

#ifndef CH32_PORTS_MAX
#define CH32_PORTS_MAX (CH32_PORTK + 1)
#endif

/**
 * @brief helper macro to encode an IO port pin in a numerical format
 */
#define CH32PIN(_port, _pin) \
	(_port << 4 | _pin)

/**
 * @brief Pin modes
 */

#define CH32_AF0     0x0
#define CH32_AF1     0x1
#define CH32_AF2     0x2
#define CH32_AF3     0x3
#define CH32_AF4     0x4
#define CH32_AF5     0x5
#define CH32_AF6     0x6
#define CH32_AF7     0x7
#define CH32_AF8     0x8
#define CH32_AF9     0x9
#define CH32_AF10    0xa
#define CH32_AF11    0xb
#define CH32_AF12    0xc
#define CH32_AF13    0xd
#define CH32_AF14    0xe
#define CH32_AF15    0xf
#define CH32_ANALOG  0x10
#define CH32_GPIO    0x11


/**
 * @brief Macro to generate pinmux int using port, pin number and mode arguments
 * This is inspired from Linux equivalent st,stm32f429-pinctrl binding
 */

/**
 * @brief Pin configuration configuration bit field.
 *
 * Fields:
 *
 * - mode [ 0 : 4 ]
 * - line [ 5 : 8 ]
 * - port [ 9 : 12 ]
 *
 * @param port Port ('A'..'K')
 * @param line Pin (0..15)
 * @param mode Mode (ANALOG, GPIO_IN, ALTERNATE).
 */

#define CH32_MODE_SHIFT 0U
#define CH32_MODE_MASK  0x1FU
#define CH32_LINE_SHIFT 5U
#define CH32_LINE_MASK  0xFU
#define CH32_PORT_SHIFT 9U
#define CH32_PORT_MASK  0xFU

#define CH32_PINMUX(port, line, mode)					       \
		(((((port) - 'A') & CH32_PORT_MASK) << CH32_PORT_SHIFT) |    \
		(((line) & CH32_LINE_MASK) << CH32_LINE_SHIFT) |	       \
		(((CH32_ ## mode) & CH32_MODE_MASK) << CH32_MODE_SHIFT))

/**
 * @brief PIN configuration bitfield
 *
 * Pin configuration is coded with the following
 * fields
 *    Alternate Functions [ 0 : 3 ]
 *    GPIO Mode           [ 4 : 5 ]
 *    GPIO Output type    [ 6 ]
 *    GPIO Speed          [ 7 : 8 ]
 *    GPIO PUPD config    [ 9 : 10 ]
 *    GPIO Output data     [ 11 ]
 *
 */

/* GPIO Mode */
#define CH32_MODER_INPUT_MODE		(0x0 << CH32_MODER_SHIFT)
#define CH32_MODER_OUTPUT_MODE		(0x1 << CH32_MODER_SHIFT)
#define CH32_MODER_ALT_MODE			(0x2 << CH32_MODER_SHIFT)
#define CH32_MODER_ANALOG_MODE		(0x3 << CH32_MODER_SHIFT)
#define CH32_MODER_MASK	 			0x3
#define CH32_MODER_SHIFT			4

/* GPIO Output type */
#define CH32_OTYPER_PUSH_PULL		(0x0 << CH32_OTYPER_SHIFT)
#define CH32_OTYPER_OPEN_DRAIN		(0x1 << CH32_OTYPER_SHIFT)
#define CH32_OTYPER_MASK			0x1
#define CH32_OTYPER_SHIFT			6

/* GPIO speed */
#define CH32_OSPEEDR_LOW_SPEED		(0x0 << CH32_OSPEEDR_SHIFT)
#define CH32_OSPEEDR_MEDIUM_SPEED	(0x1 << CH32_OSPEEDR_SHIFT)
#define CH32_OSPEEDR_HIGH_SPEED		(0x2 << CH32_OSPEEDR_SHIFT)
#define CH32_OSPEEDR_VERY_HIGH_SPEED	(0x3 << CH32_OSPEEDR_SHIFT)
#define CH32_OSPEEDR_MASK			0x3
#define CH32_OSPEEDR_SHIFT			7

/* GPIO High impedance/Pull-up/pull-down */
#define CH32_PUPDR_NO_PULL			(0x0 << CH32_PUPDR_SHIFT)
#define CH32_PUPDR_PULL_UP			(0x1 << CH32_PUPDR_SHIFT)
#define CH32_PUPDR_PULL_DOWN		(0x2 << CH32_PUPDR_SHIFT)
#define CH32_PUPDR_MASK				0x3
#define CH32_PUPDR_SHIFT			9

/* GPIO plain output value */
#define CH32_ODR_0					(0x0 << CH32_ODR_SHIFT)
#define CH32_ODR_1					(0x1 << CH32_ODR_SHIFT)
#define CH32_ODR_MASK				0x1
#define CH32_ODR_SHIFT				11

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

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_CH32_PINCTRL_H */