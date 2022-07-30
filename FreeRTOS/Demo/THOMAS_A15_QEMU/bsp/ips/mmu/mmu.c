/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <mmu.h>
#include <console.h>

/* 4k items description in L1 table, each item is 1M mapping, total is 4k * 1M = 4G VA memory */
__attribute__ ((section (".ARM_TLB"))) unsigned int mmu_ttb[SIZE_4K] = {0};

#define FLASH_BASE		0x00000000
#define FLASH_SIZE		0x02000000 /* 32M */

#define SRAM_BASE		0x10000000
#define SRAM_SIZE		0x02000000 /* 32M */

#define PERIPH_BASE		0x40000000
#define PERIPH_SIZE		0x40000000 /* 1G */

#define DDR_BASE		0x20000000
#define DDR_SIZE		0x08000000 /* 128M */

struct mmu_section_entry mmu_section_entries[] = {
	{FLASH_BASE, FLASH_BASE, FLASH_SIZE, L1_COHERENT},
	{SRAM_BASE, SRAM_BASE, SRAM_SIZE, L1_COHERENT},
	{PERIPH_BASE, PERIPH_BASE, PERIPH_SIZE, L1_DEVICE},
	{DDR_BASE, DDR_BASE, DDR_SIZE, L1_DEVICE},
	{0xffffffff, 0xffffffff, 0x0, 0x0}
};

/*
 * Translation Table Base Register c2
 * we use short descriptor after reset
 * and TTBCR.N is 0 after reset
 * so the Translation table base 0 address bit field is 31:14
 */

static void mmu_set_ttbbase(unsigned int ttb_base)
{
	/*
	 * v7
	 * ; 31:14 - Base addr
	 * ; 13:5  - 0x0
	 * ; 4:3   - RGN 0x0 (Outer Noncachable)
	 * ; 2     - IMP 0x0
	 * ; 1     - S   0x0 (Non-shared)
	 * ; 0     - C   0x0 (Inner Noncachable)
	 */
	ttb_base &= 0xffffc000;
	__asm (
		"MCR	p15, 0, %0, c2, c0, 0"
		:
		: "r" (ttb_base)
		: "memory"
	);
}

void mmu_enable(void)
{
	unsigned int val32 = 0;
	__asm (
		"DSB\n"
		"MRC	p15, 0, %0, c1, c0, 0\n" /* Read control register */
		"ORR	%0, %0, #0x00000001\n" /* Set MMU enable bit 0 */
		"MCR	p15, 0, %0, c1, c0, 0" /* Write control register 1 */
		: "=r" (val32)
		:
		: "memory"
	);
}

void mmu_disable(void)
{
	unsigned int val32 = 0;
	__asm (
		"MRC	p15, 0, %0, c1, c0, 0\n" /* Read control register */
		"BIC	%0, %0, #0x00000001\n" /* Clear MMU enable bit 0 */
		"MCR	p15, 0, %0, c1, c0, 0" /* Write control register 1 */
		: "=r" (val32)
		:
		: "memory"
	);
}

/* Set DACR register for all domain access */
static void mmu_set_domain(unsigned int domain)
{
	__asm (
		"MCR	p15, 0, %0, c3, c0, 0"
		:
		: "r" (domain)
		: "memory"
	);
}

/* TLBIALLIS, Use cp15 c8 to invalidate TLB */
void invalidate_tlb(void)
{
	__asm (
		"mov r0, #0\n"
		"mcr p15, 0, r0, c8, c3, 0" /* TLBIALLIS Invalidate all TLB*/
	);
}

/**
 * Initializes the MMU
 */
void mmu_init(void)
{
    unsigned int ix = 0;
	unsigned int desc = 0;
	unsigned int ttb_entry = 0;
	unsigned int entry_cnt = 0;
	unsigned int domain_ctrl = 0;

	/* Set domain 0-15(all) can access */
	domain_ctrl = (DOMAIN_CTL_CLIENT(0) | DOMAIN_CTL_CLIENT(1) |
			DOMAIN_CTL_CLIENT(2) | DOMAIN_CTL_CLIENT(3) |
			DOMAIN_CTL_CLIENT(4) | DOMAIN_CTL_CLIENT(5) |
			DOMAIN_CTL_CLIENT(6) | DOMAIN_CTL_CLIENT(7) |
			DOMAIN_CTL_CLIENT(8) | DOMAIN_CTL_CLIENT(9) |
			DOMAIN_CTL_CLIENT(10) | DOMAIN_CTL_CLIENT(11) |
			DOMAIN_CTL_CLIENT(12) | DOMAIN_CTL_CLIENT(13) |
			DOMAIN_CTL_CLIENT(14) | DOMAIN_CTL_CLIENT(15));

	invalidate_tlb();

	mmu_set_domain(domain_ctrl);

    /* Set up TTB */
	struct mmu_section_entry *pentry = mmu_section_entries;
	while (pentry->phy_addr != 0xffffffff) {
		entry_cnt = pentry->region_size / SIZE_1M;
		if (entry_cnt == 0)
			entry_cnt = 1;
		for (ix = 0; ix < entry_cnt; ix++) {
			desc = (pentry->phy_addr + SIZE_1M * ix) &
				TTB_SECIION_MASK;
			desc |= pentry->attribute;
			ttb_entry = (pentry->virtual_addr + SIZE_1M * ix) >> 20;
            //bm_printf_value_u32("index: ", ttb_entry);
            //bm_printf_value_u32("value desc: ", desc);
			mmu_ttb[ttb_entry] = desc;
		}
		pentry++;
	}

	/* Set TTBR0 register */
	mmu_set_ttbbase((unsigned int)mmu_ttb);
	mmu_enable();
}
