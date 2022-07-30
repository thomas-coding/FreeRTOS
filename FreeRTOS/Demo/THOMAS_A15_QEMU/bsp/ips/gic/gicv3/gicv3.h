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
#ifndef __GICV3_H__
#define __GICV3_H__

#include <stdint.h>
#include <gicv3_platform_def.h>

/*
 * extra flags for GICD enable
 */
enum gicd_ctrl_flags_t {
	gicdctlr_enable_grp0 = (1 << 0),
	gicdctlr_enable_grp1ns = (1 << 1),
	gicdctlr_enable_grp1a = (1 << 1),
	gicdctlr_enable_grp1s = (1 << 2),
	gicdctlr_enable_all = (1 << 2) | (1 << 1) | (1 << 0),
	gicdctlr_are_s = (1 << 4),	/* En Secure state affinity routing */
	gicdctlr_are_ns = (1 << 5),	/* En NSecure state affinity routing */
	gicdctlr_ds = (1 << 6),		/* Dis Security support */
	gicdctlr_e1nwf = (1 << 7)	/* En "1-of-N" wakeup model */
};

/*
 * modes for SPI routing
 */
enum gicdirouter_bits_t {
	gicdirouter_mode_specific = 0,
	gicdirouter_mode_any = (1 << 31)
};

enum gicdicfgr_bits_t {
	gicdicfgr_level = 0,
	gicdicfgr_edge = (1 << 1)
};

enum gicigroupr_bits_t {
	gicigroupr_g0s = 0,
	gicigroupr_g1ns = (1 << 0),
	gicigroupr_g1s = (1 << 1)
};

enum gicrwaker_bits_t {
	gicrwaker_processor_sleep = (1 << 1),
	gicrwaker_children_as_sleep = (1 << 2)
};

enum gicricfgr_bits_t {
	gicricfgr_level = 0,
	gicricfgr_edge = (1 << 1)
};

/**********************************************************************/

/*
 * Utility macros & functions
 */
#define RANGE_LIMIT(x)		((sizeof(x) / sizeof((x)[0])) - 1)

static inline uint64_t gicv3_pack_affinity(uint32_t aff3, uint32_t aff2,
					 uint32_t aff1, uint32_t aff0)
{
	/*
	 * only need to cast aff3 to get type promotion for all affinities
	 */
	return ((((uint64_t)aff3 & 0xff) << 32) |
		((aff2 & 0xff) << 16) |
		((aff1 & 0xff) << 8) | aff0);
}

/**********************************************************************/

/*
 * GIC Distributor Function Prototypes
 */

/*
 * gicd_config - configure GIC Distributor prior to enabling it
 *
 * Inputs:
 *
 *  control - control flags
 *
 * Returns:
 *
 *  <nothing>
 *
 * NOTE:
 *
 *  gicd_config() will set an absolute flags value, whereas
 *  gicd_{en,dis}able() will only {set,clear} the flag bits
 *  passed as a parameter
 */
void gicd_config(enum gicd_ctrl_flags_t flags);

/*
 * gicd_enable - top-level enable for GIC Distributor
 *
 * Inputs:
 *
 *  flags - new control flags to set
 *
 * Returns:
 *
 *  <nothing>
 *
 * NOTE:
 *
 *  gicd_config() will set an absolute flags value, whereas
 *  gicd_{en,dis}able() will only {set,clear} the flag bits
 *  passed as a parameter
 */
void gicd_enable(enum gicd_ctrl_flags_t flags);

/*
 * gicd_disable - top-level disable for GIC Distributor
 *
 * Inputs
 *
 *  flags - control flags to clear
 *
 * Returns
 *
 *  <nothing>
 *
 * NOTE:
 *
 *  gicd_config() will set an absolute flags value, whereas
 *  gicd_{en,dis}able() will only {set,clear} the flag bits
 *  passed as a parameter
 */
void gicd_disable(enum gicd_ctrl_flags_t flags);

/*
 * gicd_are_sync - synchronise GICD Address Routing Enable bits
 *
 * Inputs
 *
 *  flags - absolute flag bits to set in GIC Distributor
 *
 *  dosync - flag whether to wait for ARE bits to match passed
 *      flag field (dosync = true), or whether to set absolute
 *      flag bits (dosync = false)
 *
 * Returns
 *
 *  <nothing>
 *
 * NOTE:
 *
 * This function is used to resolve a race in an MP system whereby secondary
 * CPUs cannot reliably program all Redistributor registers until the
 * primary CPU has enabled Address Routing. The primary CPU will call this
 * function with dosync = false, while the secondaries will call it with
 * dosync = true.
 */
void gicd_are_sync(enum gicd_ctrl_flags_t flags, uint32_t dosync);

/*
 * spi_enable - enable a specific shared peripheral interrupt
 *
 * Inputs:
 *
 *  id - which interrupt to enable
 *
 * Returns:
 *
 *  <nothing>
 */
void spi_enable(uint32_t id);

/*
 * spi_disable - disable a specific shared peripheral interrupt
 *
 * Inputs:
 *
 *  id - which interrupt to disable
 *
 * Returns:
 *
 *  <nothing>
 */
void spi_disable(uint32_t id);

/*
 * spi_priority_set - configure the priority for a shared peripheral interrupt
 *
 * Inputs:
 *
 *  id - interrupt identifier
 *
 *  priority - 8-bit priority to program (see note below)
 *
 * Returns:
 *
 *  <nothing>
 *
 * Note:
 *
 *  The GICv3 architecture makes this function sensitive to the Security
 *  context in terms of what effect it has on the programmed priority: no
 *  attempt is made to adjust for the reduced priority range available
 *  when making Non-Secure accesses to the GIC
 */
void spi_priority_set(uint32_t id, uint32_t priority);

/*
 * spi_priority_get - determine the priority for a shared peripheral interrupt
 *
 * Inputs:
 *
 *  id - interrupt identifier
 *
 * Returns:
 *
 *  interrupt priority in the range 0 - 0xff
 */
uint32_t spi_priority_get(uint32_t id);

/*
 * spi_route_set - specify interrupt routing when gicdctlr_ARE is enabled
 *
 * Inputs:
 *
 *  id - interrupt identifier
 *
 *  affinity - prepacked "dotted quad" affinity routing. NOTE: use the
 *      gicv3_pack_affinity() helper routine to generate this input
 *
 *  mode - select routing mode (specific affinity, or any recipient)
 *
 * Returns:
 *
 *  <nothing>
 */
void spi_route_set(uint32_t id, uint64_t affinity, enum gicdirouter_bits_t mode);

/*
 * spi_route_get - read ARE-enabled interrupt routing information
 *
 * Inputs:
 *
 *  id - interrupt identifier
 *
 * Returns:
 *
 *  routing configuration
 */
uint64_t spi_route_get(uint32_t id);

/*
 * spi_target_set - configure the set of processor targets for an interrupt
 *
 * Inputs
 *
 *  id - interrupt identifier
 *
 *  target - 8-bit target bitmap
 *
 * Returns
 *
 *  <nothing>
 */
void spi_target_set(uint32_t id, uint8_t target);

/*
 * spi_target_get - read the set of processor targets for an interrupt
 *
 * Inputs
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *  8-bit target bitmap
 */
uint8_t spi_target_get(uint32_t id);

/*
 * spi_configure - setup an interrupt as edge- or level-triggered
 *
 * Inputs
 *
 *   id - interrupt identifier
 *
 *   config - desired configuration
 *
 * Returns
 *
 *   <nothing>
 */
void spi_configure(uint32_t id, enum gicdicfgr_bits_t config);

/*
 * spi_pending_set - mark an interrupt as pending
 *
 * Inputs
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *   <nothing>
 */
void spi_pending_set(uint32_t id);

/*
 * spi_pending_clear - mark an interrupt as not pending
 *
 * Inputs
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *   <nothing>
 */
void spi_pending_clear(uint32_t id);

/*
 * spi_pending_get - query whether an interrupt is pending
 *
 * Inputs
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *   pending status
 */
uint32_t spi_pending_get(uint32_t id);

/*
 * spi_security_set - mark a shared peripheral interrupt as
 *     security <group>
 *
 * Inputs
 *
 *  id - which interrupt to mark
 *
 *  group - the group for the interrupt
 *
 * Returns
 *
 *  <nothing>
 */
void spi_security_set(uint32_t id, enum gicigroupr_bits_t group);

/*
 * spi_security_block_set - mark a block of 32 shared peripheral
 *     interrupts as security <group>
 *
 * Inputs:
 *
 *  block - which block to mark (e.g. 1 = Ints 32-63)
 *
 *  group - the group for the interrupts
 *
 * Returns:
 *
 *  <nothing>
 */
void spi_security_block_set(uint32_t block, enum gicigroupr_bits_t group);

/*
 * spi_security_all_set - mark all shared peripheral interrupts
 *     as security <group>
 *
 * Inputs:
 *
 *  group - the group for the interrupts
 *
 * Returns:
 *
 *  <nothing>
 */
void spi_security_all_set(enum gicigroupr_bits_t group);

/**********************************************************************/

/*
 * GIC Re-Distributor Function Prototypes
 *
 * The model for calling Redistributor functions is that, rather than
 * identifying the target redistributor with every function call, the
 * SelectRedistributor() function is used to identify which redistributor
 * is to be used for all functions until a different redistributor is
 * explicitly selected
 */

/*
 * utility functions to calculate base of a particular
 * Redistributor bank
 */
uint32_t gicr_rd_base(uint32_t gicr);
uint32_t gicr_sgi_base(uint32_t gicr);

/*
 * gicr_wakeup - wake up a Redistributor
 *
 * Inputs:
 *
 *  gicr - which Redistributor to wakeup
 *
 * Returns:
 *
 *  <nothing>
 */
void gicr_wakeup(uint32_t gicr);

/*
 * private_intr_enable - enable a private (SGI/PPI) interrupt
 *
 * Inputs:
 *
 *  gicr - which Redistributor to program
 *
 *  id - which interrupt to enable
 *
 * Returns:
 *
 *  <nothing>
 */
void private_intr_enable(uint32_t gicr, uint32_t id);

/*
 * private_intr_disable - disable a private (SGI/PPI) interrupt
 *
 * Inputs:
 *
 *  gicr - which Redistributor to program
 *
 *  id - which interrupt to disable
 *
 * Returns:
 *
 *  <nothing>
 */
void private_intr_disable(uint32_t gicr, uint32_t id);

/*
 * private_intr_priority_set - configure the priority for a private
 *     (SGI/PPI) interrupt
 *
 * Inputs:
 *
 *  gicr - which Redistributor to program
 *
 *  id - interrupt identifier
 *
 *  priority - 8-bit priority to program (see note below)
 *
 * Returns:
 *
 *  <nothing>
 *
 * Note:
 *
 *  The GICv3 architecture makes this function sensitive to the Security
 *  context in terms of what effect it has on the programmed priority: no
 *  attempt is made to adjust for the reduced priority range available
 *  when making Non-Secure accesses to the GIC
 */
void private_intr_priority_set(uint32_t gicr, uint32_t id, uint32_t priority);

/*
 * private_intr_priority_get - configure the priority for a private
 *     (SGI/PPI) interrupt
 *
 * Inputs:
 *
 *  gicr - which Redistributor to program
 *
 *  id - interrupt identifier
 *
 * Returns:
 *
 *   Int priority
 */
uint32_t private_intr_priority_get(uint32_t gicr, uint32_t id);

/*
 * private_intr_pending_set - mark a private (SGI/PPI) interrupt as pending
 *
 * Inputs
 *
 *  gicr - which Redistributor to program
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *   <nothing>
 */
void private_intr_pending_set(uint32_t gicr, uint32_t id);

/*
 * private_intr_pending_clear - mark a private (SGI/PPI) interrupt as not
 *
 * pending Inputs
 *
 *  gicr - which Redistributor to program
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *   <nothing>
 */
void private_intr_pending_clear(uint32_t gicr, uint32_t id);

/*
 * private_intr_pending_get - query whether a private (SGI/PPI) interrupt is
 *
 * pending Inputs
 *
 *  gicr - which Redistributor to program
 *
 *  id - interrupt identifier
 *
 * Returns
 *
 *   pending status
 */
uint32_t private_intr_pending_get(uint32_t gicr, uint32_t id);

/*
 * private_intr_security_set - mark a private (SGI/PPI) interrupt as
 *     security <group>
 *
 * Inputs
 *
 *  gicr - which Redistributor to program
 *
 *  id - which interrupt to mark
 *
 *  group - the group for the interrupt
 *
 * Returns
 *
 *  <nothing>
 */
void private_intr_security_set(uint32_t gicr, uint32_t id, enum gicigroupr_bits_t group);

/*
 * private_intr_security_block_set - mark all 32 private (SGI/PPI)
 *     interrupts as security <group>
 *
 * Inputs:
 *
 *  gicr - which Redistributor to program
 *
 *  group - the group for the interrupt
 *
 * Returns:
 *
 *  <nothing>
 */
void private_intr_security_block_set(uint32_t gicr, enum gicigroupr_bits_t group);

/*
 * private_intr_configure - setup an SGI/PPI interrupt as edge- or
 * level-triggered
 *
 * Inputs
 *
 *   id - interrupt identifier
 *
 *   config - desired configuration
 *
 * Returns
 *
 *   <nothing>
 */
void private_intr_configure(uint32_t gicr, uint32_t id, enum gicricfgr_bits_t config);

#endif /* __GICV3_H__ */
