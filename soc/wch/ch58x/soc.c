/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <CH58x_common.h>


#include <stdio.h>
static int ch58x_init(void)
{
    uint32_t key;
	
	key = irq_lock();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */ 
	SetSysClock(CLK_SOURCE_PLL_80MHz);
    GPIOA_SetBits(GPIO_Pin_9);
    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();


	irq_unlock(key);

	return 0;
}

SYS_INIT(ch58x_init, PRE_KERNEL_1, 0);
