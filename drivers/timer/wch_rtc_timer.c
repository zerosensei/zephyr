/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_rtc

#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>
#include <soc.h>

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec()	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define RTC_CNT_SPAN (0xa8c00000ULL)
#define RTC_CNT_MAX (RTC_CNT_SPAN - 1ULL)
#define RTC_CNT_HALF (RTC_CNT_MAX / 2) 
#define MAX_TICKS ((RTC_CNT_HALF - CYC_PER_TICK) / CYC_PER_TICK)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

static uint64_t last_count;
static uint32_t overflow_cnt;
static struct k_spinlock lock;

static inline uint64_t anchor_updata(uint32_t rtc_val)
{
	static uint32_t last_val = 0U;
	uint64_t anchor = 0ULL;

	if (rtc_val < last_val) {
		overflow_cnt++;
	}

	anchor = (uint64_t) (rtc_val + RTC_CNT_MAX * overflow_cnt);
	last_val = rtc_val;

	return anchor;
}

static inline uint64_t wch_rtc_timer_read(void)
{
	uint32_t cnt = RTC_GetCycle32k();

	return anchor_updata(cnt);
}

static inline void RTC_SetTignTime(uint32_t time)
{
    sys_safe_access_enable();
    R32_RTC_TRIG = time;
    sys_safe_access_disable();
}

__ramfunc static int wch_rtc_set(uint64_t target_time)
{
	uint32_t rtc_val;
	uint64_t curr_time = wch_rtc_timer_read();

	rtc_val = (uint32_t) (target_time % RTC_CNT_SPAN);

	if (curr_time < target_time) {
		if (target_time - curr_time > RTC_CNT_HALF) {
			printf("err1\r\n");
			printf("target_time: %#llx\n", target_time);
			printf("curr_time: %#llx\n", curr_time);

			return -EINVAL;
		}

		RTC_SetTignTime(rtc_val);
		// printf("set %lld, %d\n", curr_time, rtc_val);
	} else {
		printf("err2\r\n");
		printf("target_time: %#llx\n", target_time);
		printf("curr_time: %#llx\n", curr_time);
		printf("overflow_cnt: %d\n", overflow_cnt);

		return -EINVAL;
	}

	return 0;
}

__ramfunc static void wch_rtc_isr(const void *arg)
{
    R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);	

	uint64_t curr_time;
	k_spinlock_key_t key = k_spin_lock(&lock);
	curr_time = wch_rtc_timer_read();

	uint32_t dticks = (uint32_t)(curr_time - last_count) / CYC_PER_TICK;
	last_count += dticks * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		/* protection is not needed because we are in the RTC interrupt
		 * so it won't get preempted by the interrupt.
		 */
		wch_rtc_set(last_count + CYC_PER_TICK);
	}

	k_spin_unlock(&lock, key);
	sys_clock_announce(dticks);
}

__ramfunc uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t cyc = wch_rtc_timer_read() - last_count;
	k_spin_unlock(&lock, key);

	return cyc / CYC_PER_TICK;
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t cyc = (uint32_t) wch_rtc_timer_read();
	
	k_spin_unlock(&lock, key);

	return cyc;
}

__ramfunc void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);
	uint32_t cyc;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	if (ticks == K_TICKS_FOREVER) {
		cyc = MAX_TICKS * CYC_PER_TICK;
	} else {
		/* Value of ticks can be zero or negative, what means "announce
		 * the next tick" (the same as ticks equal to 1).
		 */
		cyc = CLAMP(ticks, 3, (int32_t)MAX_TICKS);  // TODO: 
		cyc *= CYC_PER_TICK;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t unannounced = wch_rtc_timer_read() - last_count;

	if (unannounced >= RTC_CNT_HALF) {
		cyc = 0;
	}

	cyc += unannounced;
	cyc = DIV_ROUND_UP(cyc, CYC_PER_TICK) * CYC_PER_TICK;

	if (cyc > MAX_CYCLES)  {
		cyc = MAX_CYCLES;
	}

	uint64_t target_time = cyc + last_count;
	wch_rtc_set(target_time);
	k_spin_unlock(&lock, key);
}

static int sys_clock_init(void)
{
#if(CONFIG_WCH_RTC_LSI_32000 || CONFIG_WCH_RTC_LSI_32768)
    sys_safe_access_enable();
    R8_CK32K_CONFIG &= ~(RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON);
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_INT32K_PON;
    sys_safe_access_disable();
    Calibration_LSI(Level_64);
#else
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_OSC32K_XT | RB_CLK_INT32K_PON | RB_CLK_XT32K_PON;
    sys_safe_access_disable();
#endif

    sys_safe_access_enable();
    R32_RTC_TRIG = 0U;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO | RB_RTC_LOAD_HI;
    sys_safe_access_disable();

	last_count = 0ULL;
	overflow_cnt = 0ULL;

    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
    sys_safe_access_disable();   

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), 
		    wch_rtc_isr, 0, 0);
	irq_enable(DT_INST_IRQN(0));

	wch_rtc_set(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ?
		MAX_CYCLES : CYC_PER_TICK);

	return 0;
}

SYS_INIT(sys_clock_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
