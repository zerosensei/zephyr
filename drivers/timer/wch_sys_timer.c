/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_systimer

#include <zephyr/init.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>
#include <soc.h>

#define CYC_PER_TICK (uint32_t)((uint64_t) (sys_clock_hw_cycles_per_sec()\
				 / (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC))
#define MAX_CYC INT32_MAX
#define MAX_TICKS ((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1000

#define TICKLESS IS_ENABLED(CONFIG_TICKLESS_KERNEL)

static struct k_spinlock lock;
static uint64_t last_count;

static void set_systimer_cmp(uint64_t time)
{
	SysTick->CMP = time - 1;
}

static uint64_t get_systimer_cnt(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t cnt = SysTick->CNT;
	k_spin_unlock(&lock, key);

	return cnt;
}

__ramfunc static void timer_isr(const void *arg)
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

    SysTick->SR = 0;
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


static int sys_clock_driver_init(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), 0, timer_isr, NULL, 0);
	irq_enable(DT_INST_IRQN(0));

	last_count = 0;

	set_systimer_cmp(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ?
		MAX_CYC : CYC_PER_TICK);

    SysTick->CMP = last_count; /* set reload register */
    SysTick->CTLR = SysTick_CTLR_INIT |
                    SysTick_CTLR_STCLK |
                    SysTick_CTLR_STIE |
                    SysTick_CTLR_STE;

	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
