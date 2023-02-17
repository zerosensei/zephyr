/*
 * Copyright (c) 2023 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for CH32V307 processor
 */

#include <init.h>
/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int ch32v30x_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */ 
	SystemInit();

	irq_unlock(key);

	/* Update CMSIS SystemCoreClock variable (HCLK) */
	/* At reset, system core clock is set to 8 MHz from HSI */
	SystemCoreClock = 8000000;

	return 0;
}

SYS_INIT(ch32v30x_init, PRE_KERNEL_1, 0);
