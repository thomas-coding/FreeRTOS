/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <cache.h>
#include <console.h>

/* CSSELR */
unsigned int get_value_csselr(void)
{
	unsigned int val = 0;

	asm volatile("mrc p15, 2, %0, c0, c0, 0" : "=r" (val));

	return val;
}

void set_value_csselr(unsigned int val)
{
	asm volatile("mcr p15, 2, %0, c0, c0, 0" : : "r" (val));
	asm volatile("isb");
}

/* CCSIDR Get cache ways/words/sets
 * http://8.210.111.180/share/doc/arm/cortex-a/DDI0438I_cortex_a15_r4p0_trm.pdf
 * 4.3.21 Cache Size ID Register
 */
unsigned int get_value_ccsidr(void)
{
	unsigned int val = 0;

	asm volatile("mrc p15, 1, %0, c0, c0, 0" : "=r" (val));

	return val;
}

/* CLIDR Cache Level ID Register */
unsigned int get_value_clidr(void)
{
	unsigned int val = 0;

	asm volatile("mrc p15, 1, %0, c0, c0, 1" : "=r" (val));

	return val;
}

/*  SCTLR Get control Register */
unsigned int get_value_sctlr(void)
{
	unsigned int val = 0;

	asm volatile("mrc p15, 0, %0, c1, c0, 0" : "=r" (val));

	return val;
}

void get_cache_info(void)
{
	bm_printf_value_u32("clidr:", get_value_clidr());
	/* 0x00C50079, arm mode, mmu enable, cache disable*/
	bm_printf_value_u32("sctlr:", get_value_sctlr());

	/* L1 i-cache info */
	set_value_csselr(0x1);
	bm_printf_value_u32("L1 i-cache csselr:", get_value_csselr());
	/* 0x201FE00A : 32K, 16 words/line, 2 ways, 256 sets, support read-allocation */
	bm_printf_value_u32("L1 i-cache ccsidr:", get_value_ccsidr());

	/* L1 d-cache info */
	set_value_csselr(0x0);
	bm_printf_value_u32("L1 d-cache csselr:", get_value_csselr());
	/* 0x701FE00A : 32K, 16 words/line, 2 ways, 256 sets, support read-allocation/write-allocation/write-back */
	bm_printf_value_u32("L1 d-cache ccsidr:", get_value_ccsidr());

	/* L2 cache info */
	set_value_csselr(0x2);
	bm_printf_value_u32("L2 cache csselr:", get_value_csselr());
	/* 0x711FE07A : 2304K, 16 words/line, 16 ways, 2304 sets support read-allocation/write-allocation/write-back */
	bm_printf_value_u32("L2 cache ccsidr:", get_value_ccsidr());
}

void enable_dcache(void)
{
	__asm (
		"MRC	p15, 0, r0, c1, c0, 0\n" /* Read control register */
		"ORR	r0, r0, #0x00000004\n" /* Set Dcache enable bit 2 */
		"MCR	p15, 0, r0, c1, c0, 0" /* Write control register 1 */
	);
}

void disable_dcache(void)
{
	__asm (
		"MRC	p15, 0, r0, c1, c0, 0\n" /* Read control register */
		"BIC	r0, r0, #0x00000004\n" /* Clear Dcache enable bit 2 */
		"MCR	p15, 0, r0, c1, c0, 0" /* Write control register 1 */
	);
}

void enable_icache(void)
{
	__asm (
		"MRC	p15, 0, r0, c1, c0, 0\n" /* Read control register */
		"ORR	r0, r0, #0x00001000\n" /* Set Icache enable bit 12 */
		"MCR	p15, 0, r0, c1, c0, 0" /* Write control register 1 */
	);
}

void disable_icache(void)
{
	__asm (
		"MRC	p15, 0, r0, c1, c0, 0\n" /* Read control register */
		"BIC	r0, r0, #0x00001000\n" /* Clear Icache enable bit 12 */
		"MCR	p15, 0, r0, c1, c0, 0" /* Write control register 1 */
	);
}

/* Use cp15 c7 to invalidate i-cache */
void invalidate_icache(void)
{
	__asm (
		"mov r0, #0\n"
		"mcr p15, 0, r0, c7, c5, 0" /* ICIALLU Invalidate all instruction caches to PoU*/
	);
}

//extern void d_cache_invalidate_clean(void);
/* Use cp15 c7 to clean and invalidate d-cache */
void clean_and_invalidate_cache(void)
{
	invalidate_icache();
	d_cache_invalidate_clean();
}

void enable_cache(void)
{
	enable_dcache();
	enable_icache();
}

void disable_cache(void)
{
	disable_icache();
	disable_dcache();
}

void cache_test(void)
{
	get_cache_info();
	clean_and_invalidate_cache();
}

/**
 * Initializes the Cache
 */
void cache_init(void)
{
	cache_test();
	enable_cache();
}
