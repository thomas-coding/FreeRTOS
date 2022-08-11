/*
 * gicv3_gicd.c - generic driver code for GICv3 distributor
 *
 * Refer to Armv8.2-A SMP bare-metal(fireworks_Cortex-A55_Cortex-A76)
 *
 * Copyright (c) 2014-2017 Arm Limited (or its affiliates). All rights reserved.
 * Use, modification and redistribution of this file is subject to your
 * possession of a valid End User License Agreement for the Arm Product of
 * which these examples are part of and your compliance with all applicable
 * terms and conditions of such licence agreement.
 */
#include <stdint.h>

#include "gicv3.h"
#include "gicv3-reg.h"
//#include "data-type.h"
//#include "platform.h"
//#include "common.h"

void gicd_config(enum gicd_ctrl_flags_t flags)
{
	write_mreg32((GICD_BASE + GICD_CTLR), flags);
}

void gicd_enable(enum gicd_ctrl_flags_t flags)
{
	uint32_t val = read_mreg32(GICD_BASE + GICD_CTLR);

	val |= flags;
	write_mreg32((GICD_BASE + GICD_CTLR), val);
}

void gicd_disable(enum gicd_ctrl_flags_t flags)
{
	uint32_t val = read_mreg32(GICD_BASE + GICD_CTLR);

	val &= ~flags;
	write_mreg32((GICD_BASE + GICD_CTLR), val);
}

void gicd_are_sync(enum gicd_ctrl_flags_t flags, uint32_t dosync)
{
	if (dosync) {
		const uint32_t tmask = gicdctlr_ds;
		const uint32_t tval = flags & tmask;

		while ((read_mreg32(GICD_BASE + GICD_CTLR) & tmask) != tval)
			continue;
	} else {
		write_mreg32(GICD_BASE + GICD_CTLR, flags);
	}
}

void spi_enable(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_ISENABLER has 32 interrupts per register
	 */
	bank = (id >> 5) & 0x1f;
	id &= 32 - 1;

	write_mreg32(GICD_BASE + GICD_ISENABLER + bank * 4, 1 << id);

	return;
}

void spi_disable(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_ISENABLER has 32 interrupts per register
	 */
	bank = (id >> 5) & 0x1f;
	id &= 32 - 1;

	write_mreg32((GICD_BASE + GICD_ICENABLER + bank * 4), 1 << id);

	return;
}

void spi_priority_set(uint32_t id, uint32_t priority)
{
	uint32_t bank = 0;

	/*
	 * GICD_IPRIORITYR has one byte-wide entry per interrupt
	 */
	bank = id & 0x3ff;

	write_mreg32((GICD_BASE + GICD_IPRIORITYR + bank * 4), priority);
}

uint32_t spi_priority_get(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_IPRIORITYR has one byte-wide entry per interrupt
	 */
	bank = id & 0x3ff;

	return read_mreg32(GICD_BASE + GICD_IPRIORITYR + bank * 4);
}

void spi_route_set(uint32_t id, uint64_t affinity, enum gicdirouter_bits_t mode)
{
	uint32_t bank = 0;

	/*
	 * GICD_IROUTER has one doubleword-wide entry per interrupt
	 */
	bank = id & 0x3ff;

	/* CPU ID is aff1 */
	write_mreg64((GICD_BASE + GICD_IROUTER + bank * 8),
		affinity | (uint64_t)mode);
}

uint64_t spi_route_get(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_IROUTER has one doubleword-wide entry per interrupt
	 */
	bank = id & 0x3ff;

	return read_mreg64(GICD_BASE + GICD_IROUTER + bank * 8);
}

void spi_target_set(uint32_t id, uint8_t target)
{
	uint32_t bank = 0;

	/*
	 * GICD_ITARGETSR has one byte-wide entry per interrupt
	 */
	bank = id & 0x3ff;

	write_mreg8((GICD_BASE + GICD_ITARGETSR + bank), target);
}

uint8_t spi_target_get(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_ITARGETSR has one byte-wide entry per interrupt
	 *
	 * GICD_ITARGETSR has 4 interrupts per register, i.e. 8-bits of
	 * target bitmap per register
	 */
	bank = id & 0x3ff;

	return read_mreg8(GICD_BASE + GICD_ITARGETSR + bank);
}

void spi_configure(uint32_t id, enum gicdicfgr_bits_t config)
{
	uint32_t bank = 0, tmp = 0;

	/*
	 * GICD_ICFGR has 16 interrupts per register, i.e. 2-bits of
	 * configuration per register
	 */
	bank = (id >> 4) & 0x3f;
	config &= 3;

	id = (id & 0xf) << 1;

	tmp = read_mreg32(GICD_BASE + GICD_ICFGR + bank * 4);
	tmp &= ~(3 << id);
	tmp |= config << id;
	write_mreg32((GICD_BASE + GICD_ICFGR + bank * 4), tmp);
}

void spi_pending_set(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_ISPENDR has 32 interrupts per register
	 */
	bank = (id >> 5) & 0x1f;
	id &= 0x1f;

	write_mreg32((GICD_BASE + GICD_ISPENDR + bank * 4), 1 << id);
}

void spi_pending_clear(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_ICPENDR has 32 interrupts per register
	 */
	bank = (id >> 5) & 0x1f;
	id &= 0x1f;

	write_mreg32((GICD_BASE + GICD_ICPENDR + bank * 4), 1 << id);
}

uint32_t spi_pending_get(uint32_t id)
{
	uint32_t bank = 0;

	/*
	 * GICD_ICPENDR has 32 interrupts per register
	 */
	bank = (id >> 5) & 0x1f;
	id &= 0x1f;

	return (read_mreg32(GICD_BASE + GICD_ICPENDR + bank * 4) >> id) & 1;
}

void spi_security_set(uint32_t id, enum gicigroupr_bits_t group)
{
	uint32_t bank = 0, groupmod = 0, val = 0;

	/*
	 * GICD_IGROUPR has 32 interrupts per register
	 */
	bank = (id >> 5) & 0x1f;
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
	val = read_mreg32(GICD_BASE + GICD_IGROUPR + bank * 4);
	if (group)
		val |= 1 << id;
	else
		val &= ~(1 << id);

	write_mreg32((GICD_BASE + GICD_IGROUPR + bank * 4), val);

	/*
	 * now deal with groupmod
	 */
	val = read_mreg32(GICD_BASE + GICD_IGRPMODR + bank * 4);
	if (groupmod)
		val |= 1 << id;
	else
		val &= ~(1 << id);

	write_mreg32((GICD_BASE + GICD_IGRPMODR + bank * 4), val);
}

void spi_security_block_set(uint32_t block, enum gicigroupr_bits_t group)
{
	uint32_t groupmod = 0;
	uint32_t group_t = 0;

	/*
	 * GICD_IGROUPR has 32 interrupts per register
	 */
	block &= 0x1f;

	/*
	 * get each bit of group config duplicated over all 32-bits in a word
	 */
	groupmod = group & 0x2 ? 0xffffffff : 0x00000000;
	group_t = group & 0x1 ? 0xffffffff : 0x00000000;

	/*
	 * set the security state for this block of SPIs
	 */
	write_mreg32((GICD_BASE + GICD_IGROUPR + block * 4), group_t);
	write_mreg32((GICD_BASE + GICD_IGRPMODR + block * 4), groupmod);
}

void spi_security_all_set(enum gicigroupr_bits_t group)
{
	uint32_t block = 0, first = 0;

	/*
	 * GICD_TYPER.ITLinesNumber gives (No. SPIS / 32) - 1, and we
	 * want to iterate over all blocks excluding 0 (which are the
	 * SGI/PPI interrupts, and not relevant here)
	 */
	first = read_mreg32(GICD_BASE + GICD_TYPER) & ((1 << 5) - 1);
	for (block = first; block > 0; --block)
		spi_security_block_set(block, group);
}
