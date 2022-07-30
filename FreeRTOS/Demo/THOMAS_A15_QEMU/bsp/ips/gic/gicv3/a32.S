/*
 * A32 Cache and Branch Prediction Maintenance Operations
 *
 * Copyright (c) 2011-2017 Arm Limited (or its affiliates). All rights reserved.
 * Use, modification and redistribution of this file is subject to your
 * possession of a valid DS-5 end user licence agreement and your compliance
 * with all applicable terms and conditions of such licence agreement.
 *
 */

	.text
	/* put stack frame info into .debug_frame instead of .eh_frame */
	.cfi_sections .debug_frame

/* ------------------------------------------------------------ */
/* Interrupt enable/disable */

	/* Could use compiler intrinsics instead of these */

	.global interrupts_enable
	/* void interrupts_enable(void) */
	.type interrupts_enable, "function"
	.cfi_startproc
interrupts_enable:
	CPSIE	aif
	BX	lr
	.cfi_endproc

	.global interrupts_disable
	/* void interrupts_disable(void) */
	.type interrupts_disable, "function"
	.cfi_startproc
interrupts_disable:
	CPSID	aif
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* Cpu state save/restore */

	.global cpu_state_save
	/* void cpu_state_save(u32 *) */
	.type cpu_state_save, "function"
	.cfi_startproc
cpu_state_save:
	/* Read System Control Register */
	MRC	p15, 0, r1, c1, c0, 0
	/* State bit (bit0 | bit2 | bit11 | bit12) */
	LDR	r2, =#0x1805
	AND	r1, r1, r2
	/* Return state */
	STR	r1, [r0]
	BX	lr
	.cfi_endproc

	.global cpu_state_restore
	/* void cpu_state_restore(u32) */
	.type cpu_state_restore, "function"
	.cfi_startproc
cpu_state_restore:
	/* Read System Control Register */
	MRC	p15, 0, r1, c1, c0, 0
	/* Clear M */
	BIC	r1, r1, #(1 <<  0)
	/* Clear D */
	BIC	r1, r1, #(1 <<  2)
	/* Clear Z */
	BIC	r1, r1, #(1 << 11)
	/* Clear I */
	BIC	r1, r1, #(1 << 12)
	/* State restore */
	ORR	r1, r1, r0
	/* Write System Control Register */
	MCR	p15, 0, r1, c1, c0, 0
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* Cache Maintenance */

	.global caches_enable
	/* void caches_enable(void) */
	.type caches_enable, "function"
	.cfi_startproc
caches_enable:
	/* Read System Control Register */
	MRC	p15, 0, r0, c1, c0, 0
	/* Set D bit */
	ORR	r0, r0, #(1 << 2)
	/* Set I bit */
	ORR	r0, r0, #(1 << 12)
	/* Write System Control Register */
	MCR	p15, 0, r0, c1, c0, 0
	BX	lr
	.cfi_endproc

	.global caches_disable
	/* void caches_disable(void) */
	.type caches_disable, "function"
	.cfi_startproc
caches_disable:
	/* Read System Control Register */
	MRC	p15, 0, r0, c1, c0, 0
	/* Clear D bit */
	BIC	r0, r0, #(1 << 2)
	/* Clear I bit */
	BIC	r0, r0, #(1 << 12)
	/* Write System Control Register */
	MCR	p15, 0, r0, c1, c0, 0
	BX	lr
	.cfi_endproc

	.global d_cache_clean
	/* void d_cache_clean(void) */
	.type d_cache_clean, "function"
	.cfi_startproc
d_cache_clean:
	PUSH	{r4-r12}

	/*
	 * Based on code example given in section 11.2.4 of Armv7-A/R
	 * Architecture Reference Manual (DDI 0406B)
	 */

	/* Read CLIDR */
	MRC	p15, 1, r0, c0, c0, 1
	ANDS	r3, r0, #0x7000000

	/* Cache level value (naturally aligned) */
	MOV	r3, r3, LSR #23
	BEQ	clean_dcache_finished
	MOV	r10, #0

clean_dcache_loop1:
	/* Work out 3xcachelevel */
	ADD	r2, r10, r10, LSR #1
	/* bottom 3 bits are the Cache type for this level */
	MOV	r1, r0, LSR r2
	/* get those 3 bits alone */
	AND	r1, r1, #7
	CMP	r1, #2
	/* no cache or only instruction cache at this level */
	BLT	clean_dcache_skip
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

clean_dcache_loop2:
	/* R9 working copy of the max way size (right aligned) */
	MOV	r9, R4

clean_dcache_loop3:
	/* factor in the way number and cache number into R11 */
	ORR	r11, r10, r9, LSL r5
	/* factor in the index number */
	ORR	r11, r11, r7, LSL r2
	/* DCCSW - clean by set/way */
	MCR	p15, 0, r11, c7, c10, 2
	/* decrement the way number */
	SUBS	r9, r9, #1
	BGE	clean_dcache_loop3
	/* decrement the index */
	SUBS	r7, r7, #1
	BGE	clean_dcache_loop2

clean_dcache_skip:
	/* increment the cache number */
	ADD	r10, r10, #2
	CMP	r3, r10
	BGT	clean_dcache_loop1

clean_dcache_finished:
	POP	{r4-r12}

	BX	lr
	.cfi_endproc

	.global caches_invalidate
	/* void caches_invalidate(void) */
	.type caches_invalidate, "function"
	.cfi_startproc
caches_invalidate:
	PUSH	{r4-r12}

	/* Read Cache Level ID Register (CLIDR) */
	MRC	p15, 1, r0, c0, c0, 1
	/* Harvard Cache? */
	TST	r0, #0x3
	/* SBZ */
	MOV	r0, #0
	/*
	 * ICIALLU - Invalidate instruction cache and flush branch target cache
	 */
	MCRNE	p15, 0, r0, c7, c5, 0

	/* Read CLIDR */
	MRC	p15, 1, r0, c0, c0, 1
	ANDS	r3, r0, #0x7000000
	/* Cache level value (naturally aligned) */
	MOV	r3, r3, LSR #23
	BEQ	caches_invalidate_finished
	MOV	r10, #0

caches_invalidate_loop1:
	/* Work out 3xcachelevel */
	ADD	r2, r10, r10, LSR #1
	/* bottom 3 bits are the Cache type for this level */
	MOV	r1, r0, LSR r2
	/* get those 3 bits alone */
	AND	r1, r1, #7
	CMP	r1, #2
	/* no cache or only instruction cache at this level */
	BLT	caches_invalidate_skip
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

caches_invalidate_loop2:
	/* R9 working copy of the max way size (right aligned) */
	MOV	r9, R4

caches_invalidate_loop3:
	/* factor in the way number and cache number into R11 */
	ORR	r11, r10, r9, LSL r5
	/* factor in the index number */
	ORR	r11, r11, r7, LSL r2
	/* DCISW - invalidate by set/way */
	MCR	p15, 0, r11, c7, c6, 2
	/* decrement the way number */
	SUBS	r9, r9, #1
	BGE	caches_invalidate_loop3
	/* decrement the index */
	SUBS	r7, r7, #1
	BGE	caches_invalidate_loop2

caches_invalidate_skip:
	/* increment the cache number */
	ADD	r10, r10, #2
	CMP	r3, r10
	BGT	caches_invalidate_loop1

caches_invalidate_finished:
	POP	{r4-r12}
	BX	lr
	.cfi_endproc

	.global caches_invalidate_is
	/* void caches_invalidate_is(void) */
	.type caches_invalidate_is, "function"
	.cfi_startproc
caches_invalidate_is:
	PUSH	{r4-r12}

	MOV	r0, #0
	/* ICIALLUIS - Invalidate entire I Cache inner shareable */
	MCR	p15, 0, r0, c7, c1, 0

	/* Read CLIDR */
	MRC	p15, 1, r0, c0, c0, 1
	ANDS	r3, r0, #0x7000000
	/* Cache level value (naturally aligned) */
	MOV	r3, r3, LSR #23
	BEQ	caches_invalidate_is_finished
	MOV	r10, #0

caches_invalidate_is_loop1:
	/* Work out 3xcachelevel */
	ADD	r2, r10, r10, LSR #1
	/* bottom 3 bits are the Cache type for this level */
	MOV	r1, r0, LSR r2
	/* get those 3 bits alone */
	AND	r1, r1, #7
	CMP	r1, #2
	/* no cache or only instruction cache at this level */
	BLT	caches_invalidate_is_skip
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

caches_invalidate_is_loop2:
	/* R9 working copy of the max way size (right aligned) */
	MOV	r9, R4

caches_invalidate_is_loop3:
	/* factor in the way number and cache number into R11 */
	ORR	r11, r10, r9, LSL r5
	/* factor in the index number */
	ORR	r11, r11, r7, LSL r2
	/* DCISW - clean by set/way */
	MCR	p15, 0, r11, c7, c6, 2
	/* decrement the way number */
	SUBS	r9, r9, #1
	BGE	caches_invalidate_is_loop3
	/* decrement the index */
	SUBS	r7, r7, #1
	BGE	caches_invalidate_is_loop2

caches_invalidate_is_skip:
	/* increment the cache number */
	ADD	r10, r10, #2
	CMP	r3, r10
	BGT	caches_invalidate_is_loop1

caches_invalidate_is_finished:
	POP	{r4-r12}
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* TLB */

	.global unified_tlb_invalidate
	/* void unified_tlb_invalidate(void) */
	.type unified_tlb_invalidate, "function"
	.cfi_startproc
unified_tlb_invalidate:
	MOV	r0, #0
	/* TLBIALL - Invalidate entire unified TLB */
	MCR	p15, 0, r0, c8, c7, 0
	BX	lr
	.cfi_endproc


	.global unified_tlb_invalidate_is
	/* void unified_tlb_invalidate_is(void) */
	.type unified_tlb_invalidate_is, "function"
	.cfi_startproc
unified_tlb_invalidate_is:
	MOV	r0, #1
	/* TLBIALLIS - Invalidate entire unified TLB Inner Shareable */
	MCR	p15, 0, r0, c8, c3, 0
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* Branch Prediction */

	.global branch_prediction_enable
	/* void branch_prediction_enable(void) */
	.type branch_prediction_enable, "function"
	.cfi_startproc
branch_prediction_enable:
	/* Read SCTLR */
	MRC	p15, 0, r0, c1, c0, 0
	/* Set the Z bit (bit 11) */
	ORR	r0, r0, #(1 << 11)
	/* Write SCTLR */
	MCR	p15, 0,r0, c1, c0, 0
	BX	lr
	.cfi_endproc

	.global branch_prediction_disable
	/* void branch_prediction_disable(void) */
	.type branch_prediction_disable, "function"
	.cfi_startproc
branch_prediction_disable:
	/* Read SCTLR */
	MRC	p15, 0, r0, c1, c0, 0
	/* Clear the Z bit (bit 11) */
	BIC	r0, r0, #(1 << 11)
	/* Write SCTLR */
	MCR	p15, 0,r0, c1, c0, 0
	BX	lr
	.cfi_endproc

	.global branch_target_cache_flush
	/* void branch_target_cache_flush(void) */
	.type branch_target_cache_flush, "function"
	.cfi_startproc
branch_target_cache_flush:
	MOV	r0, #0
	/* BPIALL - Invalidate entire branch predictor array */
	MCR	p15, 0, r0, c7, c5, 6
	BX	lr
	.cfi_endproc

	.global branch_target_cache_flush_is
	/* void branch_target_cache_flush_is(void) */
	.type branch_target_cache_flush_is, "function"
	.cfi_startproc
branch_target_cache_flush_is:
	MOV	r0, #0

	/*
	 * BPIALLIS - Invalidate entire branch predictor array Inner Shareable
	 */
	MCR	p15, 0, r0, c7, c1, 6
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* High Vecs */

	.global high_vecs_enable
	/* void high_vecs_enable(void) */
	.type high_vecs_enable, "function"
	.cfi_startproc
high_vecs_enable:
	MRC	p15, 0, r0, c1, c0, 0 /* Read Control Register */
	ORR	r0, r0, #(1 << 13)    /* Set the V bit (bit 13) */
	MCR	p15, 0, r0, c1, c0, 0 /* Write Control Register */
	BX	lr
	.cfi_endproc

	.global high_vecs_disable
	/* void high_vecs_disable(void) */
	.type high_vecs_disable, "function"
	.cfi_startproc
high_vecs_disable:
	/* Read Control Register */
	MRC	p15, 0, r0, c1, c0, 0
	/* Clear the V bit (bit 13) */
	BIC	r0, r0, #(1 << 13)
	/* Write Control Register */
	MCR	p15, 0, r0, c1, c0, 0
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* Context ID */

	.global context_id_get
	/* u32 context_id_get(void) */
	.type context_id_get, "function"
	.cfi_startproc
context_id_get:
	/* Read Context ID Register */
	MRC	p15, 0, r0, c13, c0, 1
	BX	lr
	.cfi_endproc

	.global context_id_set
	/* void context_id_set(u32) */
	.type context_id_set, "function"
	.cfi_startproc
context_id_set:
	/* Write Context ID Register */
	MCR	p15, 0, r0, c13, c0, 1
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* ID registers */

	.global midr_get
	/* u32 midr_get(void) */
	.type midr_get, "function"
	.cfi_startproc
midr_get:
	MRC	p15, 0, r0, c0, c0, 0 /* Read Main ID Register (MIDR) */
	BX	lr
	.cfi_endproc

	.global mpidr_get
	/* u32 mpidr_get(void) */
	.type mpidr_get, "function"
	.cfi_startproc
mpidr_get:
	/* Read Multiprocessor ID register (MPIDR) */
	MRC	p15, 0, r0, c0 ,c0, 5
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
/* CP15 SMP related */

	.global base_addr_get
	/*
	 * u32 base_addr_get(void)
	 * Returns the value CBAR (base address of the private peripheral
	 * memory space)
	 */
	.type base_addr_get, "function"
	.cfi_startproc
base_addr_get:
	/* Read periph base address */
	MRC	p15, 4, r0, c15, c0, 0
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */
	.global cpu_id_get
	/*
	 * u32 cpu_id_get(void)
	 * Returns the CPU ID (0 to 3) of the CPU executed on
	 */
	.type cpu_id_get, "function"
	.cfi_startproc
cpu_id_get:
	/* Read CPU ID register */
	MRC	p15, 0, r0, c0, c0, 5
	/* Mask off, leaving the CPU ID field */
	AND	r0, r0, #0x03
	BX	lr
	.cfi_endproc

/* ------------------------------------------------------------ */

	.global goto_sleep
	/* void goto_sleep(void) */
	.type goto_sleep, "function"
	.cfi_startproc
goto_sleep:
	DSB				/* Clear all pending data accesses */
	WFI				/* Go into standby */
	B	goto_sleep		/* Catch in case of rogue events */
	BX	lr
	.cfi_endproc
