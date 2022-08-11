/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

	.global d_cache_invalidate_clean
	/* void d_cache_invalidate_clean(void) */
	.type d_cache_invalidate_clean, "function"
	.cfi_startproc
d_cache_invalidate_clean:
	PUSH	{r4-r12}

	/*
	 * Based on code example given in section 11.2.4 of Armv7-A/R
	 * Architecture Reference Manual (DDI 0406B)
	 */

	/* Read CLIDR */
	MRC	p15, 1, r0, c0, c0, 1
	ANDS	r3, r0, #0x700000
	/* Cache level value (naturally aligned) */
	MOV	r3, r3, LSR #23
	BEQ	d_cache_invalidate_clean_finished
	MOV	r10, #0

/* This loop for each cache, like L1 D-cache, L2 cache etc */
d_cache_invalidate_clean_loop1:
	/* Work out 3xcachelevel */
	ADD	r2, r10, r10, LSR #1
	/* bottom 3 bits are the Cache type for this level */
	MOV	r1, r0, LSR r2
	/* get those 3 bits alone */
	AND	r1, r1, #7
	CMP	r1, #2
	/* no cache or only instruction cache at this level */
	BLT	d_cache_invalidate_clean_skip
	/* write the Cache Size selection register */
	MCR	p15, 2, r10, c0, c0, 0
	/* ISB to sync the change to the CacheSizeID reg */
	ISB
	/* reads current Cache Size ID register */
	MRC	p15, 1, r1, c0, c0, 0
	/* extract the line length field */
	AND	r2, r1, #7
	/* add 4 for the line length offset (log2 16 bytes) */
	ADD	r2, r2, #4
	LDR	r4, =0x3ff
	/* R4 is the max number on the way size (right aligned) */
	ANDS	r4, r4, r1, LSR #3
	/* R5 is the bit position of the way size increment */
	CLZ	r5, r4
	LDR	r7, =0x00007fff
	/* R7 is the max number of the index size (right aligned) */
	ANDS	r7, r7, r1, LSR #13

/* This loop for all way in this cache */
d_cache_invalidate_clean_loop2:
	/* R9 working copy of the max way size (right aligned) */
	MOV	r9, R4
/* This loop for all sets in this way */
d_cache_invalidate_clean_loop3:
	/* factor in the way number and cache number into R11 */
	ORR	r11, r10, r9, LSL r5
	/* factor in the index number */
	ORR	r11, r11, r7, LSL r2
	/* This is actual clean and invalidate one cache line */
	/* DCCISW - clean and invalidate by set/way */
	MCR	p15, 0, r11, c7, c14, 2
	/* decrement the way number */
	SUBS	r9, r9, #1
	BGE	d_cache_invalidate_clean_loop3
	/* decrement the index */
	SUBS	r7, r7, #1
	BGE	d_cache_invalidate_clean_loop2

d_cache_invalidate_clean_skip:
	/* increment the cache number */
	ADD	r10, r10, #2
	CMP	r3, r10
	BGT	d_cache_invalidate_clean_loop1

d_cache_invalidate_clean_finished:
	POP	{r4-r12}

	BX	lr
	.cfi_endproc
