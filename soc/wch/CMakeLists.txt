# Copyright (c) 2024 zerosensei
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

message(${SOC_SERIES})

add_subdirectory(${SOC_SERIES})