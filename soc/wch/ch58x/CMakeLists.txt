# Copyright (c) 2024 zerosensei
# SPDX-License-Identifier: Apache-2.0


zephyr_include_directories(.)
zephyr_library_sources(
    soc.c
    soc_irq.S
    vector.S
)

zephyr_linker_sources(RAMFUNC_SECTION highcode.ld)
zephyr_library_sources_ifdef(CONFIG_PM power.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
