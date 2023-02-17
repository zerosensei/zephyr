/*
 * Copyright (c) 2023 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _CH32V30x_SOC_H_
#define _CH32V30x_SOC_H_

/* ECALL Exception numbers */
#define SOC_MCAUSE_ECALL_EXP 5 /* Machine ECALL instruction */
#define SOC_MCAUSE_USER_ECALL_EXP 8 /* User ECALL instruction */

/* Interrupt Mask */
#define SOC_MCAUSE_IRQ_MASK (1 << 31)
/* Exception code Mask */
#define SOC_MCAUSE_EXP_MASK 0x7FFFFFFF

/* SOC-Specific EXIT ISR command */
#define SOC_ERET mret

#ifndef _ASMLANGUAGE
#include <toolchain.h>
#include <ch32v30x.h>
#endif  /* !_ASMLANGUAGE */

#endif /* _CH32V307_SOC_H_ */
