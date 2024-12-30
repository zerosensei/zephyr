# Copyright (c) 2024 zerosensei
# SPDX-License-Identifier: Apache-2.0

#include <zephyr/toolchain.h>

/* Exports */
GTEXT(__start)

/* Imports */
GTEXT(__initialize)


SECTION_FUNC(vectors, __start)
	# li 	a0, 3
	# csrw	mtvec, a0

	/*
	 * CLINT direct mode
	 *
	 * Set mtvec (Machine Trap-Vector Base-Address Register)
	 * to _isr_wrapper.
	 */
    la t0, _isr_wrapper
	csrw mtvec, t0

	/* Jump to __reset */
	tail	__reset
