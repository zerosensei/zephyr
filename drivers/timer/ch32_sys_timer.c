/*
 * Copyright (c) 2018 Foundries.io Ltd
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <soc.h>

#include <ch32v30x.h>

#include <drivers/clock_control.h>
#include <drivers/clock_control/ch32_clock_control.h>
#include <drivers/timer/system_timer.h>
#include <sys_clock.h>

#include <spinlock.h>

/* SysTick Control / Status Register Definitions */
#define SysTick_CTLR_COUNTFLAG_Pos         16U                                            /*!< SysTick CTLR: COUNTFLAG Position */
#define SysTick_CTLR_COUNTFLAG_Msk         (1UL << SysTick_CTLR_COUNTFLAG_Pos)            /*!< SysTick CTLR: COUNTFLAG Mask */

#define SysTick_CTLR_CLKSOURCE_Pos          2U                                            /*!< SysTick CTLR: CLKSOURCE Position */
#define SysTick_CTLR_CLKSOURCE_Msk         (1UL << SysTick_CTLR_CLKSOURCE_Pos)            /*!< SysTick CTLR: CLKSOURCE Mask */

#define SysTick_CTLR_TICKINT_Pos            1U                                            /*!< SysTick CTLR: TICKINT Position */
#define SysTick_CTLR_TICKINT_Msk           (1UL << SysTick_CTLR_TICKINT_Pos)              /*!< SysTick CTLR: TICKINT Mask */

#define SysTick_CTLR_ENABLE_Pos             0U                                            /*!< SysTick CTLR: ENABLE Position */
#define SysTick_CTLR_ENABLE_Msk            (1UL /*<< SysTick_CTLR_ENABLE_Pos*/)           /*!< SysTick CTLR: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_CMP_RECMP_Pos             0U                                            /*!< SysTick CMP: RECMP Position */
#define SysTick_CMP_RECMP_Msk            (0xFFFFFFUL /*<< SysTick_CMP_RECMP_Pos*/)    /*!< SysTick CMP: RECMP Mask */

/* SysTick Current Register Definitions */
#define SysTick_CNT_CURRENT_Pos             0U                                            /*!< SysTick CNT: CURRENT Position */
#define SysTick_CNT_CURRENT_Msk            (0xFFFFFFUL /*<< SysTick_CNT_CURRENT_Pos*/)    /*!< SysTick CNT: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31U                                            /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)               /*!< SysTick CALIB: NOREF Mask */

#define SysTick_CALIB_SKEW_Pos             30U                                            /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)                /*!< SysTick CALIB: SKEW Mask */

#define SysTick_CALIB_TENMS_Pos             0U                                            /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/)    /*!< SysTick CALIB: TENMS Mask */

#define CYC_PER_TICK ((uint32_t)((uint64_t) (sys_clock_hw_cycles_per_sec()			 \
					     >> CONFIG_RISCV_MACHINE_TIMER_SYSTEM_CLOCK_DIVIDER) \
				 / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_CYC INT_MAX
#define MAX_TICKS ((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1000

#define TICKLESS IS_ENABLED(CONFIG_TICKLESS_KERNEL)

static struct k_spinlock lock;
static uint64_t last_count;

static void set_systimer_cmp(uint64_t time)
{
	SysTick->CTLR &= ~(1<<0);
    SysTick->CMP  = time-1;
    SysTick->CNT  = 0;
    SysTick->SR  = 0;
    NVIC_ClearPendingIRQ(SysTicK_IRQn);
    SysTick->CTLR |= (1<<0);
}

static uint64_t get_systimer_cnt(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t cnt = SysTick->CNT;
	k_spin_unlock(&lock, key);

	return cnt;
}

static void timer_isr(const void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t now = get_systimer_cnt();
	uint32_t dticks = (uint32_t)((now - last_count) / CYC_PER_TICK);

	last_count = now;

	if (!TICKLESS) {
		uint64_t next = last_count + CYC_PER_TICK;

		if ((int64_t)(next - now) < MIN_DELAY) {
			next += CYC_PER_TICK;
		}
		set_systimer_cmp(next);
	}

	k_spin_unlock(&lock, key);
	sys_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL)
	/* RISCV has no idle handler yet, so if we try to spin on the
	 * logic below to reset the comparator, we'll always bump it
	 * forward to the "next tick" due to MIN_DELAY handling and
	 * the interrupt will never fire!  Just rely on the fact that
	 * the OS gave us the proper timeout already.
	 */
	if (idle) {
		return;
	}

	ticks = ticks == K_TICKS_FOREVER ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t now = get_systimer_cnt();
	uint32_t adj, cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary. */
	adj = (uint32_t)(now - last_count) + (CYC_PER_TICK - 1);
	if (cyc <= MAX_CYC - adj) {
		cyc += adj;
	} else {
		cyc = MAX_CYC;
	}
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;

	if ((int32_t)(cyc + last_count - now) < MIN_DELAY) {
		cyc += CYC_PER_TICK;
	}

	set_systimer_cmp(cyc + last_count);
	k_spin_unlock(&lock, key);
#endif
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = ((uint32_t)get_systimer_cnt() - (uint32_t)last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);
	return ret;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return (uint32_t)get_systimer_cnt();
}

uint64_t sys_clock_cycle_get_64(void)
{
	return get_systimer_cnt() ;
}


static int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(SysTicK_IRQn, 0, timer_isr, NULL, 0);
	NVIC_SetPriority(SysTicK_IRQn, 0);
	last_count = CYC_PER_TICK - 1;
	SysTick->SR = 0;
	SysTick->CMP = last_count;
	SysTick->CNT = 0; /* resets timer to last_load */
	SysTick->CTLR |= (SysTick_CTLR_ENABLE_Msk |
			  SysTick_CTLR_TICKINT_Msk |
			  SysTick_CTLR_CLKSOURCE_Msk);
	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
