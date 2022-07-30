/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

	.section startup, "ax"
	.balign 0x20

/*----------------------------------------------------------------------------
  Definitions
 *----------------------------------------------------------------------------*/
#define USR_MODE 0x10            // User mode
#define FIQ_MODE 0x11            // Fast Interrupt Request mode
#define IRQ_MODE 0x12            // Interrupt Request mode
#define SVC_MODE 0x13            // Supervisor mode
#define ABT_MODE 0x17            // Abort mode
#define UND_MODE 0x1B            // Undefined Instruction mode
#define SYS_MODE 0x1F            // System mode
#define SMC_MODE 0x16            // SMC mode

/* when I bit is set, IRQ is disabled */
#define I_BIT  0x80
/* when F bit is set, FIQ is disabled */
#define F_BIT  0x40
/* when NS bit is set, current is non-secure world */
#define NS_BIT 0x01

Vectors:
	b	Reset_Handler
	b	undefined_handler
	b	FreeRTOS_SWI_Handler//svc_handler
	b	prefetch_handler
	b	abort_handler
	b	. //Reserved vector
	b	FreeRTOS_IRQ_Handler//irq_handler
	b	fiq_handler

smc_vectors:
	B	. //reset_handler
	B	. //undefined_handler
	B	smc_handler
	B	. //prefetch_handler
	B	. //abort_handler
	B	. //Reserved vector
	B	. //irq_handler
	B	. //fiq_handler

smc_handler:
	/*
	 * Save APCS corruptible registers to SMC mode stack (and maintain
	 * 8 byte alignment)
	 */
	PUSH	{r0-r4, r12}

	/* Which world have we come from */
	/* Read Secure Configuration Register data */
	MRC     p15, 0, r0, c1, c1, 0
	/* Toggle NS bit */
	EOR     r0, r0, #NS_BIT
	/* Write Secure Configuration Register data */
	MCR     p15, 0, r0, c1, c1, 0
	ISB

	LDR	r0, =Vectors
	/* Write Secure or Non-secure Vector Base Address */
	MCR	p15, 0, r0, c12, c0, 0

	/* Clear local monitor */
	/* Not strictly required in this example, as not using LDREX/STREX */
	/* However, architecturally should execute CLREX on a context switch */
	CLREX

	POP	{r0-r4, r12}        /* Restore stacked APCS registers */
	/* Perform exception return */
	MOVS    pc, lr

prefetch_handler:
	B	prefetch_handler


svc_handler:
	srsfd	sp!, #0x13              // Store SPSR_svc and LR_svc onto SVC stack
	/*
	* Save APCS corruptible registers to IRQ mode stack (and maintain
	* 8 byte alignment)
	*/
	PUSH	{r0-r8, r12}

	#ldr	r12, [lr,#-2]               // Thumb: load halfword
	#bic	r12, r12, #0xFF00           //        extract SVC number
	ldr		r12, [lr,#-4]               // ARM:   load word
	bic		r12, r12, #0xFF000000       //        extract SVC number
	cmp		r12, #8                    // Compare SVC number
	B		.
    /*
     * This example only uses (and enables) one.  At this point
     * you would normally check the ID, and clear the source.
     */

    POP		{r0-r8, r12}        /* Restore stacked APCS registers */
    RFEFD	sp!                 /* Return from exception */

irq_handler:
    /* Pre-adjust lr */
    SUB lr, lr, #4

    /* Save lr and SPSR to IRQ mode stack */
    SRSFD   sp!, #0x12

    /*
     * Save APCS corruptible registers to IRQ mode stack (and maintain
     * 8 byte alignment)
     */
    PUSH    {r0-r8, r12}

    /* Handle the interrupt */
    BL  gic_irq_handler

    /*
     * This example only uses (and enables) one.  At this point
     * you would normally check the ID, and clear the source.
     */

    POP {r0-r8, r12}        /* Restore stacked APCS registers */
    RFEFD   sp!                 /* Return from exception */

fiq_handler:
    /* Pre-adjust lr */
    SUB lr, lr, #4

    /* Save lr and SPSR to IRQ mode stack */
    SRSFD   sp!, #0x11

    /*
     * Save APCS corruptible registers to IRQ mode stack (and maintain
     * 8 byte alignment)
     */
    PUSH    {r0-r8, r12}

    /* Handle the interrupt */
    BL  gic_fiq_handler

    /*
     * This example only uses (and enables) one.  At this point
     * you would normally check the ID, and clear the source.
     */

    POP {r0-r8, r12}        /* Restore stacked APCS registers */
    RFEFD   sp!                 /* Return from exception */

/* Abort Handler, hardware will change lr and cpsr, save caller cpsr to spsr */
	.global	abort_handler
	.type abort_handler, "function"
abort_handler:
	/* Pre-adjust lr */
	/* If 0x6c instruction abort, hardware will save (+8)0x74 to lr, -4 is point to next instruction */
	SUB	lr, lr, #4

	/* SPSR is save caller CPSR */
	/* Save lr and SPSR to ABT mode stack */
	SRSFD	sp!, #0x17

	/*
	 * Save APCS corruptible registers to ABT mode stack (and maintain
	 * 8 byte alignment)
	 */
	PUSH	{r0-r8, r12}

	/* Do something here */

	POP	{r0-r8, r12}        /* Restore stacked APCS registers */
	RFEFD	sp!                 /* Return from exception, POP spsr and lr saved */

/* Undefined Handler */
	.global	undefined_handler
	.type undefined_handler, "function"
undefined_handler:
	/* Pre-adjust lr */
	SUB	lr, lr, #4

	/* Save lr and SPSR to ABT mode stack */
	SRSFD	sp!, #0x1b

	/*
	 * Save APCS corruptible registers to ABT mode stack (and maintain
	 * 8 byte alignment)
	 */
	PUSH	{r0-r4, r12}

	/* Do something here */

	POP	{r0-r4, r12}        /* Restore stacked APCS registers */
	RFEFD	sp!                 /* Return from exception */


	.globl   Reset_Handler
Reset_Handler:
	mov		r0, #1
	mov		r1, #2
	mov		r2, #3
	mov		r3, #4
	mov		r4, #5
	mov		r5, #6
	mov		r6, #7
	mov		r7, #8

	/*
	 * Set vector base address
	 */
	LDR	r0, =smc_vectors
	/* Write SMC Vector Base Address */
	MCR	p15, 0, r0, c12, c0, 1

	LDR	r0, =Vectors
	/* Write Secure or Non-secure Vector Base Address */
	MCR	p15, 0, r0, c12, c0, 0

	/* Setup stack */
	/* Read CPU ID register */
	MRC	p15, 0, r0, c0, c0, 5
	/* Mask off, leaving the CPU ID field */
	ANDS	r0, r0, #0x03

	/* Set abort stack address, the end of secure sram */
	msr		CPSR_c, #ABT_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$ABORT_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

	msr		CPSR_c, #UND_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$UNDEF_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

	msr		CPSR_c, #IRQ_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$IRQ_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

	msr		CPSR_c, #FIQ_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$FIQ_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

	msr		CPSR_c, #SMC_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$SMC_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

	/* sys mode and user mode use same sp */
	msr		CPSR_c, #SYS_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$USER_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

	/* Set stack address, the end of secure sram */
	msr		CPSR_c, #SVC_MODE | I_BIT | F_BIT
	LDR		r1, =Image$$SVC_STACK$$ZI$$Limit
	SUB		r1, r1, r0, LSL #10
	MOV		sp, r1

#	ldr		r3, =0x88888888 /* Test prefetch abort, bx set pc to 0x88888888, try to fetch from this address abort */
#	bx		r3
#	ldr		r3, =0x88888888 /* Test data abort, write value to 0x88888888 */
#	str		r0, [r3]

	/*
	 * Activate VFP/NEON, if required
	 */

#if defined (__ARM_NEON) || defined (__ARM_FP)
	/*
	 * Enable access to NEON/VFP by enabling access to Coprocessors 10 & 11.
	 * Enables Full Access i.e. in both privileged and non privileged modes
	 */

	/* Read Coprocessor Access Control Register (CPACR) */
	MRC	p15, 0, r0, c1, c0, 2
	/* Enable access to CP 10 & 11 */
	ORR	r0, r0, #(0xf << 20)
	/* Write Coprocessor Access Control Register (CPACR) */
	MCR	p15, 0, r0, c1, c0, 2

	/* Enable access to Coprocessors 10 and 11 from non-secure code too */

	/* Read Non-secure Access Control Register (NSACR) */
	MRC     p15, 0, r0, c1, c1, 2
	/* Enable access to CP 10 & 11 */
	ORR     r0, r0, #(0x3 << 10)
	/* Write Non-secure Access Control Register (NSACR) */
	MCR     p15, 0, r0, c1, c1, 2

	ISB

	/* Switch on the VFP and NEON hardware */
	MOV	r0, #0x40000000
	/* Write FPEXC register, EN bit set */
	VMSR	FPEXC, r0
#endif


	/*
	 * SMP initialization
	 */

	/* Read CPU ID register */
	mrc	p15, 0, r0, c0, c0, 5
	/* Mask off, leaving the CPU ID field */
	ands	r0, r0, #0x3
	beq	primary_cpu_init
	bne	secondary_cpus_init

/* ------------------------------------------------------------ */
/* Initialization for PRIMARY CPU */

	.global primary_cpu_init
	.type primary_cpu_init, "function"
primary_cpu_init:


	/* Copy .data from flash to sram */
	/* Read copy table, copy form r1 to r2, length is r3 */
	ldr		r0, = __copy_table_start__
	ldr		r1, [r0]
	ldr		r2, [r0, #4]
	ldr		r3, [r0, #8]

	/* r3-4, if result < 0, branch to done */
.L_loop0_0:
	subs	r3, #4
	blt		.L_loop0_0_done
	ldr		r0, [r1, r3]
	str		r0, [r2, r3]
	b		.L_loop0_0
.L_loop0_0_done:

	/* Clear bss */
	/* Read zero table, zero form r0, length is r2 */
	ldr		r0, = __zero_table_start__
	ldr		r1, [r0]
	ldr		r2, [r0, #4]
	/* The value of r0 will fill bss, for test, we can set it to 0xa5a5a5a5 */
	ldr		r0, =0

	/* r2-4, if result < 0, branch to done */
.L_loop0_1:
	subs	r2, #4
	blt		.L_loop0_1_done
	str		r0, [r1, r2]
	b		.L_loop0_1
.L_loop0_1_done:

	/* Init gic */
	/* Switch to SMC(Monitor) mode, this registers must be set in SMC mode */
	MSR	CPSR_c, #SMC_MODE

	/* Write MSRE, enable gic system register */
	MOV	r0, #0x0f
	MCR	p15, 6, r0, c12, c12, 5

	/* Write SRE, enable gic system register */
	MOV	r0, #0x07
	MCR	p15, 0, r0, c12, c12, 5

	/* ICC_MGRPEN1, set to igrp_enable_grp1ns | igrp_enable_grp1s */
	MOV	r0, #0x03
	MCR	p15, 6, r0, c12, c12, 7

	/* Switch to svc mode */
	MSR	CPSR_c, #SVC_MODE | I_BIT | F_BIT

	/* Read CPU ID register */
	MRC	p15, 0, r0, c0, c0, 5
	/* Mask off, leaving the CPU ID field */
	ANDS	r1, r0, #0x03
	/*gicdctlr_ds*/
	MOV	r0, #(1 << 5) | (1 << 4)
	BL	gicd_are_sync

	/*
	 * The Redistributor comes out of reset assuming the processor is
	 * asleep - correct that assumption
	 */
	MOV	r0, r1
	BL	gicr_wakeup

	/*
	 * Now we're ready to set security and other initialisations
	 * This is a per-CPU configuration for these interrupts
	 * for the first cluster, CPU number is the redistributor index
	 */
	MOV	r0, r1
	MOV	r1, #2			/* gicigroup_g1s */
	BL	private_intr_security_block_set

	/*
	 * While we're in the Secure World, set the priority mask low enough
	 * for it to be writable in the Non-Secure World
	 */
	MOV	r0, #0xff		/* for Non-Secure interrupts */
	MCR	p15, 0, r0, c4, c6, 0	/* ICC_PMR */

	bl		main
	b		.

/* ------------------------------------------------------------ */
/* Initialization for SECONDARY CPUs */

	.global secondary_cpus_init
	.type secondary_cpus_init, "function"
secondary_cpus_init:
	/* Init gic */
	/* Switch to SMC(Monitor) mode, this registers must be set in SMC mode */
	MSR	CPSR_c, #SMC_MODE

	/* Write MSRE, enable gic system register */
	MOV	r0, #0x0f
	MCR	p15, 6, r0, c12, c12, 5

	/* Write SRE, enable gic system register */
	MOV	r0, #0x07
	MCR	p15, 0, r0, c12, c12, 5

	/* ICC_MGRPEN1, set to igrp_enable_grp1ns | igrp_enable_grp1s */
	MOV	r0, #0x03
	MCR	p15, 6, r0, c12, c12, 7

	/* Switch to svc mode */
	MSR	CPSR_c, #SVC_MODE | I_BIT | F_BIT

	/* Read CPU ID register */
	MRC	p15, 0, r0, c0, c0, 5
	/* Mask off, leaving the CPU ID field */
	and	r1, r0, #0x03
	/*gicdctlr_ds*/
	//MOV	r0, #(1 << 5) | (1 << 4)
	//BL	gicd_are_sync

	/*
	 * The Redistributor comes out of reset assuming the processor is
	 * asleep - correct that assumption
	 */
	MOV	r0, r1
	BL	gicr_wakeup

	/*
	 * Now we're ready to set security and other initialisations
	 * This is a per-CPU configuration for these interrupts
	 * for the first cluster, CPU number is the redistributor index
	 */
	MOV	r0, r1
	MOV	r1, #2			/* gicigroup_g1s */
	BL	private_intr_security_block_set

	/*
	 * While we're in the Secure World, set the priority mask low enough
	 * for it to be writable in the Non-Secure World
	 */
	MOV	r0, #0xff		/* for Non-Secure interrupts */
	MCR	p15, 0, r0, c4, c6, 0	/* ICC_PMR */

	MOV	r0, #1
	MCR	p15, 0, r0, c12, c12, 6	  /* ICC_IGRPEN0 */
	MCR	p15, 0, r0, c12, c12, 7   /* ICC_IGRPEN1 */

	/* Read CPU ID register */
	MRC	p15, 0, r0, c0, c0, 5
	/* Mask off, leaving the CPU ID field */
	ANDS	r0, r0, #0x03

	MOV	r1, #0
	MOV	r2, #14 << 4
	BL	private_intr_priority_set

	/* Enable interrupts and fast interrupts */
	CPSIE	if

#if 0
	/* wait for primary core run ok, then we can use print */
	ldr r0, =0xffffff
1:
	sub r0, #1
	cmp r0, #0
	bne 1b

	bl second_main
#endif
		/*
	 * Holding Pen
	 */
holding_pen:

	DSB
	WFI

	NOP
	NOP

	B	holding_pen
