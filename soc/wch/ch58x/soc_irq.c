/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <soc.h>

void z_soc_irq_enable(unsigned int irq)
{
	PFIC_EnableIRQ(irq);
}

void z_soc_irq_disable(unsigned int irq)
{
	PFIC_DisableIRQ(irq);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	return PFIC_GetStatusIRQ(irq);
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
	PFIC_SetPriority(irq, prio);
}
