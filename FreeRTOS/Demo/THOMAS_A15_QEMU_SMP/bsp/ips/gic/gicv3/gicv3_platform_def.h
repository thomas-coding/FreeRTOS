/*
 * GICv3.h - data types and function prototypes for GICv3 utility routines
 *
 * Refer to Armv8.2-A SMP bare-metal(fireworks_Cortex-A55_Cortex-A76)
 *
 * Copyright (c) 2014-2017 Arm Limited (or its affiliates). All rights reserved.
 * Use, modification and redistribution of this file is subject to your
 * possession of a valid End User License Agreement for the Arm Product of
 * which these examples are part of and your compliance with all applicable
 * terms and conditions of such licence agreement.
 */
#ifndef __GICV3__PLATFORM_DEF_H__
#define __GICV3__PLATFORM_DEF_H__

#include <stdint.h>
//#include "data-type.h"

/* Platform define, refer to qemu/hw/arm/thomas_a15.c */
#define GICD_BASE 0x60000000
#define GICR_BASE 0x61000000


/* Common define */
#define read_mreg64(addr)		(*(volatile uint64_t *)(addr))
#define write_mreg64(addr, val)		(*(volatile uint64_t *)(addr) = (uint64_t)(val))

#define read_mreg32(addr)		(*(volatile uint32_t *)(addr))
#define write_mreg32(addr, val)		(*(volatile uint32_t *)(addr) = (uint32_t)(val))

#define read_mreg16(addr)		(*(volatile u16 *)(addr))
#define write_mreg16(addr, val)		(*(volatile u16 *)(addr) = (u16)(val))

#define read_mreg8(addr)		(*(volatile uint8_t *)(addr))
#define write_mreg8(addr, val)		(*(volatile uint8_t *)(addr) = (uint8_t)(val))

/* ------------------------------------------------------------ */
/* Interrupts */
/* Enable/disables IRQs (not FIQs) */
void interrupts_enable(void);
void interrupts_disable(void);

uint32_t cpu_id_get(void);

#endif
