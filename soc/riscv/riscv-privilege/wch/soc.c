/*
 * Copyright (c) 2023 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <ch58x_common.h>

static int ch58x_init(const struct device *arg)
{
    uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */ 
	SetSysClock(CLK_SOURCE_PLL_60MHz);

	irq_unlock(key);

	return 0;
}

SYS_INIT(ch58x_init, PRE_KERNEL_1, 0);