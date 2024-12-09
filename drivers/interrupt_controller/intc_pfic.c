/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT wch_pfic

#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <soc.h>

#define SEVONPEND (1 << 4)
#define WFITOWFE  (1 << 3)

void arch_irq_enable(unsigned int irq)
{
	PFIC_EnableIRQ(irq);
}

void arch_irq_disable(unsigned int irq)
{
	PFIC_DisableIRQ(irq);
}

int arch_irq_is_enabled(unsigned int irq)
{
	return PFIC_GetStatusIRQ(irq);
}
void z_riscv_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	PFIC_SetPriority(irq, prio);
}

// static int pfic_init(void)
// {
// 	/* `wfi` is called with interrupts disabled. Configure the PFIC to wake up on any event,
// 	 * including any interrupt.
// 	 */
// 	PFIC->SCTLR = SEVONPEND | WFITOWFE;
// 	return 0;
// }

// SYS_INIT(pfic_init, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);
