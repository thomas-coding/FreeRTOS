/*
 * Prototypes and inline functions for GICC system register operations
 *
 * Author: Kang Zhigang <Zhigang.Kang@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 */

#ifndef __GICV3_GICC_H__
#define __GICV3_GICC_H__

#include <gicv3_platform_def.h>
//#include "data-type.h"

/**********************************************************************/

enum icc_sre_bits_t {
	sre_sre = (1 << 0),
	sre_dfb = (1 << 1),
	sre_dib = (1 << 2),
	sre_enable = (1 << 3)
};

static inline void gicc_sre_set(enum icc_sre_bits_t mode)
{
	asm volatile("mcr p15, 0, %0, c12, c12, 5" : : "r" ((uint32_t)mode));
	asm volatile("isb");
}

static inline uint32_t gicc_sre_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c12, 5" : "=r" (ret));

	return ret;
}

static inline void gicc_hsre_set(enum icc_sre_bits_t mode)
{
	asm volatile("mcr p15, 4, %0, c12, c9, 5" : : "r" ((uint32_t)mode));
	asm volatile("isb");
}

static inline uint32_t gicc_hsre_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 4, %0, c12, c9, 5" : "=r" (ret));

	return ret;
}

static inline void gicc_msre_set(enum icc_sre_bits_t mode)
{
	asm volatile("msr CPSR_c, #0x16");
	asm volatile("mcr p15, 6, %0, c12, c12, 5" : : "r" ((uint32_t)mode));
	asm volatile("msr CPSR_c, #0x1f");
	asm volatile("isb");
}

static inline uint32_t gicc_msre_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 6, %0, c12, c12, 5" : "=r" (ret));

	return ret;
}

/**********************************************************************/

enum icc_igrp_bits {
	igrp_enable = (1 << 0),
	igrp_enable_grp1ns = (1 << 0),
	igrp_enable_grp1s = (1 << 1)
};

static inline void gicc_igrpen0_set(enum icc_igrp_bits mode)
{
	asm volatile("mcr p15, 0, %0, c12, c12, 6" : : "r" ((uint32_t)mode));
	asm volatile("isb");
}

static inline void gicc_igrpen1_set(enum icc_igrp_bits mode)
{
	asm volatile("mcr p15, 0, %0, c12, c12, 7" : : "r" ((uint32_t)mode));
	asm volatile("isb");
}

static inline void gicc_mgrpen1_set(enum icc_igrp_bits mode)
{
	asm volatile("msr CPSR_c, #0x16");
	asm volatile("mcr p15, 6, %0, c12, c12, 7" : : "r" ((uint32_t)mode));
	asm volatile("msr CPSR_c, #0x1f");
	asm volatile("isb");
}

/**********************************************************************/

enum icc_ctlr_bits_t {
	ctlr_cbpr = (1 << 0),
	ctlr_cbpr_el1s = (1 << 0),
	ctlr_eoi_mode = (1 << 1),
	ctlr_cbpr_el1ns = (1 << 1),
	ctlr_eoi_mode_el3 = (1 << 2),
	ctlr_eoi_mode_el1s = (1 << 3),
	ctlr_eoi_mode_el1ns = (1 << 4),
	ctlr_rm = (1 << 5),
	ctlr_pmhe = (1 << 6)
};

static inline void gicc_ctlr_set(enum icc_ctlr_bits_t mode)
{
	asm volatile("mcr p15, 0, %0, c12, c12, 4" : : "r" ((uint32_t)mode));
	asm volatile("isb");
}

static inline uint32_t gicc_ctlr_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c12, 4" : "=r" (ret));

	return ret;
}

static inline void gicc_mctlr_set(enum icc_ctlr_bits_t mode)
{
	asm volatile("mcr p15, 6, %0, c12, c12, 4" : : "r" ((uint32_t)mode));
	asm volatile("isb");
}

static inline uint32_t gicc_mctlr_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 6, %0, c12, c12, 4" : "=r" (ret));

	return ret;
}

/**********************************************************************/

static inline uint32_t gicc_iar0_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c8, 0" : "=r" (ret));
	asm volatile("dsb sy");

	return ret;
}

static inline uint64_t gicc_iar1_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c12, 0" : "=r" (ret));
	asm volatile("dsb sy");

	return ret;
}

static inline void gicc_eoir0_set(uint32_t intr)
{
	asm volatile("mcr p15, 0, %0, c12, c8, 1" : : "r" (intr));
	asm volatile("isb");
}

static inline void gicc_eoir1_set(uint32_t intr)
{
	asm volatile("mcr p15, 0, %0, c12, c12, 1" : : "r" (intr));
	asm volatile("isb");
}

static inline void gicc_dir_set(uint32_t intr)
{
	asm volatile("mcr p15, 0, %0, c12, c11, 1" : : "r" (intr));
	asm volatile("isb");
}

static inline void gicc_pmr_set(uint32_t priority)
{
	asm volatile("mcr p15, 0, %0, c4, c6, 0" : : "r" (priority));
	asm volatile("isb");
}

static inline void gicc_bpr0_set(uint32_t bp)
{
	asm volatile("mcr p15, 0, %0, c12, c8, 3" : : "r" (bp));
	asm volatile("isb");
}

static inline void gicc_bpr1_set(uint32_t bp)
{
	asm volatile("mcr p15, 0, %0, c12, c12, 3" : : "r" (bp));
	asm volatile("isb");
}

static inline uint32_t gicc_bpr0_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c8, 3" : "=r" (ret));

	return ret;
}

static inline uint32_t gicc_bpr1_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c12, 3" : "=r" (ret));

	return ret;
}

static inline uint32_t gicc_bpr_get(void)
{
	uint32_t ret = 0;

	asm volatile("mrc p15, 0, %0, c12, c11, 3" : "=r" (ret));

	return ret;
}

/**********************************************************************/

enum icc_sgir_bits_t
{
	sgir_irm_target = 0,
	sgir_irm_all = (1ull << 40)
};

static inline void gicc_sgi0r_set(uint8_t aff3, uint8_t aff2,
		uint8_t aff1, enum icc_sgir_bits_t irm,
		uint16_t targetlist, uint8_t intid)
{
	uint64_t packedbits = (((uint64_t)aff3 << 48) | ((uint64_t)aff2 << 32) | \
		((uint64_t)aff1 << 16) | irm | targetlist | \
		((uint64_t)(intid & 0x0f) << 24));
	uint32_t l_val = packedbits >> 0;
	uint32_t h_val = packedbits >> 32;

	asm volatile("mcrr p15, 2, %0, %1, c12" : : "r" (l_val), "r" (h_val));
	asm volatile("isb");
}

static inline void gicc_sgi1r_set(uint8_t aff3, uint8_t aff2,
		uint8_t aff1, enum icc_sgir_bits_t irm,
		uint16_t targetlist, uint8_t intid)
{
	uint64_t packedbits = (((uint64_t)aff3 << 48) | ((uint64_t)aff2 << 32) | \
		((uint64_t)aff1 << 16) | irm | targetlist | \
		((uint64_t)(intid & 0x0f) << 24));
	uint32_t l_val = packedbits >> 0;
	uint32_t h_val = packedbits >> 32;

	asm volatile("mcrr p15, 0, %0, %1, c12" : : "r" (l_val), "r" (h_val));
	asm volatile("isb");
}

static inline void gicc_asgi1r_set(uint8_t aff3, uint8_t aff2,
		uint8_t aff1, enum icc_sgir_bits_t irm,
		uint16_t targetlist, uint8_t intid)
{
	uint64_t packedbits = (((uint64_t)aff3 << 48) | ((uint64_t)aff2 << 32) | \
		((uint64_t)aff1 << 16) | irm | targetlist | \
		((uint64_t)(intid & 0x0f) << 24));
	uint32_t l_val = packedbits >> 0;
	uint32_t h_val = packedbits >> 32;

	asm volatile("mcrr p15, 2, %0, %1, c12" : : "r" (l_val), "r" (h_val));
	asm volatile("isb");
}

#endif /* __GICV3_GICC_H__ */
