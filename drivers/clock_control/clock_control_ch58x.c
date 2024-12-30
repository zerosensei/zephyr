/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch5xx_sysclk

#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>

#include <soc.h>

#if !DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(hse))
#error "HSE clock is required"
#endif

#define DT_CLK32K_SCR_NODE \
    DT_PHANDLE_BY_IDX(DT_NODELABEL(clk32k), clock_source, 0)

#define DT_CLK32K_FREQ()        \
    DT_PROP(DT_CLK32K_SCR_NODE, clock_frequency)

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(clk32k))
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lsi)) && \
        DT_SAME_NODE(DT_CLK32K_SCR_NODE, DT_NODELABEL(lsi))
#define CLK_32K_SRC_LSI_ENABLE 1
#elif DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lse)) && \
        DT_SAME_NODE(DT_CLK32K_SCR_NODE, DT_NODELABEL(lse))
#define CLK_32K_SRC_LSE_ENABLE 1
#else
#error "32K clock source is not supported"
#endif
#endif /* DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(clk32k)) */

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(pll))
#define CLK_PLL_ENABLE 1
#endif

struct ch58x_sysclk_config {
	uint32_t fsys;
};

static int ch58x_sysclk_on(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return 0;
}

static int ch58x_sysclk_off(const struct device *dev, clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return 0;
}

static int ch58x_sysclk_get_rate(const struct device *dev, clock_control_subsys_t sys,
				 uint32_t *rate)
{
	const struct ch58x_sysclk_config *cfg = dev->config;

	ARG_UNUSED(sys);

	*rate = cfg->fsys;

	return 0;
}

static enum clock_control_status ch58x_sysclk_get_status(const struct device *dev,
							 clock_control_subsys_t sys)
{
	return CLOCK_CONTROL_STATUS_ON;
}

static int ch58x_sysclk_init(const struct device *dev)
{
    uint32_t key;
	const struct ch58x_sysclk_config *cfg = dev->config;
    SYS_CLKTypeDef sys_clk;
	key = irq_lock();

#if CLK_32K_SRC_LSI_ENABLE
    sys_safe_access_enable();
    R8_CK32K_CONFIG &= ~(RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON);
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_INT32K_PON;
    sys_safe_access_disable();
    Calibration_LSI(Level_64);
#endif

#if CLK_32K_SRC_LSE_ENABLE
    sys_safe_access_enable();
    R8_CK32K_CONFIG |= RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON;
    sys_safe_access_disable();
#endif

	switch (cfg->fsys) {
#if CLK_PLL_ENABLE
    case MHZ(80):
        sys_clk = CLK_SOURCE_PLL_80MHz;
        break;
    case MHZ(60):
        sys_clk = CLK_SOURCE_PLL_60MHz;
        break;
    case MHZ(48):
        sys_clk = CLK_SOURCE_PLL_48MHz;
        break;
    case MHZ(40):
        sys_clk = CLK_SOURCE_PLL_40MHz;
        break;
    case KHZ(36900):
        sys_clk = CLK_SOURCE_PLL_36_9MHz;
        break;
    case MHZ(32):
        sys_clk = CLK_SOURCE_PLL_32MHz;
        break;
    case MHZ(30):
        sys_clk = CLK_SOURCE_PLL_30MHz;
        break;
    case MHZ(24):  
        sys_clk = CLK_SOURCE_PLL_24MHz;
        break;
    case MHZ(20):
        sys_clk = CLK_SOURCE_PLL_20MHz;
        break;
    case MHZ(15):
        sys_clk = CLK_SOURCE_PLL_15MHz;
        break;
#endif /* CLK_PLL_ENABLE */
    case MHZ(16):
        sys_clk = CLK_SOURCE_HSE_16MHz;
        break;
    case MHZ(8):
        sys_clk = CLK_SOURCE_HSE_8MHz;
        break;
    case KHZ(6400):
        sys_clk = CLK_SOURCE_HSE_6_4MHz;
        break;
    case MHZ(4):   
        sys_clk = CLK_SOURCE_HSE_4MHz;
        break;
    case MHZ(2):  
        sys_clk = CLK_SOURCE_HSE_2MHz;
        break;
    case MHZ(1):
        sys_clk = CLK_SOURCE_HSE_1MHz;
        break;
    case 32768:
#if CLK_32K_SRC_LSI_ENABLE
        sys_clk = CLK_SOURCE_LSI;
#elif CLK_32K_SRC_LSE_ENABLE
        sys_clk = CLK_SOURCE_LSE;
#else
        return -EINVAL;
#endif
        break;
#if CLK_32K_SRC_LSI_ENABLE
    case 32000:
        sys_clk = CLK_SOURCE_LSI;
        break;
#endif
    default:
        return -EINVAL;
    }

	SetSysClock(sys_clk);

	irq_unlock(key);

	return 0;
}

static const struct clock_control_driver_api ch58x_sysclk_api = {
	.on = ch58x_sysclk_on,
	.off = ch58x_sysclk_off,
	.get_rate = ch58x_sysclk_get_rate,
	.get_status = ch58x_sysclk_get_status,
};

#define CH58X_CLKMUX_INST(n)                                                                       \
	static const struct ch58x_sysclk_config ch58x_sysclk_cfg_##n = {                           \
		.fsys = DT_INST_PROP(n, clock_frequency),                                     \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ch58x_sysclk_init, NULL, NULL, &ch58x_sysclk_cfg_##n,             \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,                    \
			      &ch58x_sysclk_api);

DT_INST_FOREACH_STATUS_OKAY(CH58X_CLKMUX_INST)
