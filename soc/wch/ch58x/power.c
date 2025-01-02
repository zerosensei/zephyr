/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/pm/pm.h>
#include <soc.h>

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	irq_unlock(MSTATUS_IEN);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
        LowPower_Idle();
		break;
	case PM_STATE_STANDBY:
		LowPower_Sleep(RB_PWR_RAM2K | RB_PWR_RAM30K | RB_PWR_EXTEND);
        DelayUs(1400);
        HSECFG_Current(HSE_RCur_100);
		break;
    case PM_STATE_SOFT_OFF:
        LowPower_Shutdown(0);
        break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);
}