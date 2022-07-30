/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <misc.h>
#include <console.h>

#define USR_MODE 0x10            // User mode
#define FIQ_MODE 0x11            // Fast Interrupt Request mode
#define IRQ_MODE 0x12            // Interrupt Request mode
#define SVC_MODE 0x13            // Supervisor mode
#define ABT_MODE 0x17            // Abort mode
#define UND_MODE 0x1B            // Undefined Instruction mode
#define SYS_MODE 0x1F            // System mode

/* Get CPSR Register */
uint32_t get_CPSR(void)
{
	uint32_t result;
	asm volatile("MRS %0, cpsr" : "=r" (result) );
	return(result);
}

/* Get mode */
uint32_t get_mode(void)
{
	return (get_CPSR() & 0x1FU);
}

uint32_t is_svc_mode(void)
{
	if(get_mode() == SVC_MODE)
		return 1;
	return 0;
}

/**
 * Initializes the MISC
 */
void misc_init(void)
{
	bm_printf_value_u32("svc mode:", is_svc_mode());
}
