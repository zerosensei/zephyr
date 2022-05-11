/*
 *
 * Copyright (c) 2022 Nanjing Qinheng Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "soc.h"
#include <drivers/clock_control.h>
#include <drivers/clock_control/ch32_clock_control.h>
#include <ch32v30x.h>

#if CONFIG_CLOCK_CH32_MCO_SRC_NOCLOCK
	#define MCO_SOURCE		RCC_MCO_NoClock
#elif CONFIG_CLOCK_CH32_MCO_SRC_SYSCLK
	#define MCO_SOURCE		RCC_MCO_SYSCLK
#elif CONFIG_CLOCK_CH32_MCO_SRC_HSI
	#define MCO_SOURCE		RCC_MCO_HSI
#elif CONFIG_CLOCK_CH32_MCO_SRC_HSE
	#define MCO_SOURCE		RCC_MCO_HSE
#elif CONFIG_CLOCK_CH32_MCO_SRC_PLL_DIV2
	#define MCO_SOURCE		RCC_MCO_PLLCLK_Div2
#elif CONFIG_CLOCK_CH32_MCO_SRC_PLL2
	#define MCO_SOURCE		RCC_MCO_PLL2CLK
#elif CONFIG_CLOCK_CH32_MCO_SRC_PLL3_DIV2
	#define MCO_SOURCE		RCC_MCO_PLL3CLK_Div2
#elif CONFIG_CLOCK_CH32_MCO_SRC_XT1
	#define MCO_SOURCE		RCC_MCO_XT1
#elif CONFIG_CLOCK_CH32_MCO_SRC_PLL3
	#define MCO_SOURCE		RCC_MCO_PLL3CLK
#endif


// static uint32_t get_bus_clock(uint32_t clock, uint32_t prescaler)
// {
// 	return clock / prescaler;
// }

static inline int ch32_clock_control_on(const struct device *dev, 
					clock_control_subsys_t sub_system)
{
	struct ch32_pclken *pclken = (struct ch32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	switch (pclken->bus) {
	case CH32_CLOCK_BUS_AHB:
		RCC_AHBPeriphClockCmd(pclken->enr, 1);
		break;

	case CH32_CLOCK_BUS_APB1:
		RCC_APB1PeriphClockCmd(pclken->enr, 1);
		break;

	case CH32_CLOCK_BUS_APB2:
		RCC_APB2PeriphClockCmd(pclken->enr, 1);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline int ch32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	struct ch32_pclken *pclken = (struct ch32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	switch (pclken->bus) {
	case CH32_CLOCK_BUS_AHB:
		RCC_AHBPeriphClockCmd(pclken->enr, 0);
		break;

	case CH32_CLOCK_BUS_APB1:
		RCC_APB1PeriphClockCmd(pclken->enr, 0);
		break;

	case CH32_CLOCK_BUS_APB2:
		RCC_APB2PeriphClockCmd(pclken->enr, 0);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

// static int ch32_clock_control_get_subsys_rate(const struct device *clock,
// 						clock_control_subsys_t sub_system,
// 						uint32_t *rate)
// {
// 	struct ch32_pclken *pclken = (struct ch32_pclken *)(sub_system);
// 	/*
// 	 * Get AHB Clock (= SystemCoreClock = SYSCLK/prescaler)
// 	 * SystemCoreClock is preferred to CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
// 	 * since it will be updated after clock configuration and hence
// 	 * more likely to contain actual clock speed
// 	 */
// 	uint32_t ahb_clock = SystemCoreClock;
// 	uint32_t apb1_clock = get_bus_clock(ahb_clock, CH32_APB1_PRESCALER);
// 	uint32_t apb2_clock = get_bus_clock(ahb_clock, CH32_APB2_PRESCALER);

// 	ARG_UNUSED(clock);

// 	switch (pclken->bus) {
// 	case CH32_CLOCK_BUS_AHB:
// 		*rate = ahb_clock;
// 		break;
// 	case CH32_CLOCK_BUS_APB1:
// 		*rate = apb1_clock;
// 		break;

// 	case CH32_CLOCK_BUS_APB2:
// 		*rate = apb2_clock;
// 		break;

// 	default:
// 		return -ENOTSUP;
// 	}

// 	return 0;
// }

static struct clock_control_driver_api ch32_clock_control_api = {
	.on = ch32_clock_control_on,
	.off = ch32_clock_control_off,
	// .get_rate = ch32_clock_control_get_subsys_rate,
};

static inline void ch32_clock_control_mco_init(void)
{
#ifndef CONFIG_CLOCK_CH32_MCO_SRC_NOCLOCK
	RCC_MCOConfig(MCO_SOURCE);
#endif /* CONFIG_CLOCK_CH32_MCO_SRC_NOCLOCK */
}


static int ch32_clock_control_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	SystemInit();

	return 0;
}

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		    ch32_clock_control_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &ch32_clock_control_api);