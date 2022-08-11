/*
 * gicv3_gicr.c - generic driver code for GICv3 redistributor
 *
 * Refer to Armv8.2-A SMP bare-metal(fireworks_Cortex-A55_Cortex-A76)
 *
 * Copyright (c) 2014-2018 Arm Limited (or its affiliates). All rights reserved.
 * Use, modification and redistribution of this file is subject to your
 * possession of a valid End User License Agreement for the Arm Product of
 * which these examples are part of and your compliance with all applicable
 * terms and conditions of such licence agreement.
 */
#include "gicv3.h"
#include "gicv3-reg.h"
//#include "platform.h"
//#include "common.h"

/*
 * utility functions to calculate base of a particular
 * Redistributor bank
 */
uint32_t gicr_rd_base(uint32_t gicr)
{
	return (GICR_BASE + GICV3_REDIST_SIZE * gicr + GICV3_REDIST_RD_OFF);
}

uint32_t gicr_sgi_base(uint32_t gicr)
{
	return (GICR_BASE + GICV3_REDIST_SIZE * gicr + GICV3_REDIST_SGI_OFF);
}

/**********************************************************************/

/* This function walks a block of RDs to find one with the matching affinity */
uint32_t gicr_get(uint32_t affinity)
{
	uint32_t base = 0;
	uint32_t index = 0;

	do {
		base = gicr_rd_base(index);

		if (read_mreg32(base + GICR_TYPER + 4) == affinity)
			return index;

		index++;
	} while ((read_mreg32(base + GICR_TYPER) & (1 << 4)) == 0);
	/* Keep looking until GICR_TYPER.Last reports no more RDs in block */

	return 0xFFFFFFFF; /* return -1 to signal not RD found */
}

void gicr_wakeup(uint32_t gicr)
{
	uint32_t base = gicr_rd_base(gicr);
	uint32_t val = 0;

	/* Power up Re-distributor for GIC-600 */
	write_mreg32((base + GICR_PWRR), 0x2);

	/*
	 * step 1 - ensure GICR_WAKER.ProcessorSleep is off
	 */
	val = read_mreg32(base + GICR_WAKER);
	val &= ~gicrwaker_processor_sleep;
	write_mreg32((base + GICR_WAKER), val);

	/*
	 * step 2 - wait for children asleep to be cleared
	 */
	while (read_mreg32(base + GICR_WAKER) & gicrwaker_children_as_sleep)
		continue;

	/*
	 * OK, GICR is go
	 */
	return;
}

void private_intr_enable(uint32_t gicr, uint32_t id)
{
	uint32_t base = gicr_sgi_base(gicr);

	id &= 0x1f;

	write_mreg32((base + GICR_ISENABLER0), 1 << id);
}

void private_intr_disable(uint32_t gicr, uint32_t id)
{
	uint32_t base = gicr_sgi_base(gicr);

	id &= 0x1f;

	write_mreg32((base + GICR_ICENABLER0), 1 << id);
}

void private_intr_priority_set(uint32_t gicr, uint32_t id, uint32_t priority)
{
	uint32_t base = gicr_sgi_base(gicr);

	/*
	 * GICD_IPRIORITYR has one byte-wide entry per interrupt
	 */
	id &= 0x1f;

	write_mreg8((base + GICR_IPRIORITYR0 + id), (uint8_t)priority);
}

uint32_t private_intr_priority_get(uint32_t gicr, uint32_t id)
{
	uint32_t base = gicr_sgi_base(gicr);

	/*
	 * GICD_IPRIORITYR has one byte-wide entry per interrupt
	 */
	id &= 0x1f;

	return read_mreg8(base + GICR_IPRIORITYR0 + id);
}

void private_intr_pending_set(uint32_t gicr, uint32_t id)
{
	uint32_t base = gicr_sgi_base(gicr);

	/*
	 * GICR_ISPENDR is one 32-bit register
	 */
	id &= 0x1f;

	write_mreg32((base + GICR_ISPENDR0), 1 << id);
}

void private_intr_pending_clear(uint32_t gicr, uint32_t id)
{
	uint32_t base = gicr_sgi_base(gicr);

	/*
	 * GICR_ICPENDR is one 32-bit register
	 */
	id &= 0x1f;

	write_mreg32((base + GICR_ICPENDR0), 1 << id);
}

uint32_t private_intr_pending_get(uint32_t gicr, uint32_t id)
{
	uint32_t base = gicr_sgi_base(gicr);

	/*
	 * GICR_ISPENDR is one 32-bit register
	 */
	id &= 0x1f;

	return (read_mreg32(base + GICR_ISPENDR0) >> id) & 0x01;
}

void private_intr_security_set(uint32_t gicr, uint32_t id, enum gicigroupr_bits_t group)
{
	uint32_t base = gicr_sgi_base(gicr);
	uint32_t groupmod = 0;
	uint32_t val = 0;

	/*
	 * GICR_IGROUPR0 is one 32-bit register
	 */
	id &= 0x1f;

	/*
	 * the single group argument is split into two separate
	 * registers, so filter out and remove the (new to gicv3)
	 * group modifier bit
	 */
	groupmod = (group >> 1) & 1;
	group &= 1;

	/*
	 * either set or clear the Group bit for the interrupt as appropriate
	 */
	val = read_mreg32(base + GICR_IGROUPR0);
	if (group)
		val |= 1 << id;
	else
		val &= ~(1 << id);
	write_mreg32((base + GICR_IGROUPR0), val);

	/*
	 * now deal with groupmod
	 */
	val = read_mreg32(base + GICR_IGRPMODR0);
	if (groupmod)
		val |= 1 << id;
	else
		val &= ~(1 << id);
	write_mreg32((base + GICR_IGRPMODR0), val);
}

void private_intr_security_block_set(uint32_t gicr, enum gicigroupr_bits_t group)
{
	uint32_t base = gicr_sgi_base(gicr);
	uint32_t groupmod = 0;
	uint32_t group_t = 0;

	/*
	 * get each bit of group config duplicated over all 32 bits
	 */
	groupmod = group  & 0x2 ? 0xffffffff : 0x00000000;
	group_t = group & 0x1 ? 0xffffffff : 0x00000000;

	/*
	 * set the security state for this block of SPIs
	 */
	write_mreg32((base + GICR_IGROUPR0), group_t);
	write_mreg32((base + GICR_IGRPMODR0), groupmod);
}

void private_intr_configure(uint32_t gicr, uint32_t id, enum gicricfgr_bits_t config)
{
	uint32_t base = gicr_sgi_base(gicr);
	uint32_t bank = 0, tmp = 0;

	/*
	 * GICR_ICFGR0/1 has 16 interrupts per register, i.e. 2-bits of
	 * configuration per register
	 */
	bank = (id >> 4) & 0x1;
	config &= 3;

	id = (id & 0xf) << 1;

	tmp = read_mreg32(base + GICR_ICFGR0 + bank * 4);
	tmp &= ~(3 << id);
	tmp |= config << id;
	write_mreg32((base + GICR_ICFGR0 + bank * 4), tmp);
}
