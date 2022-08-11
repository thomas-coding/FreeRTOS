/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/* Thomas-a15 memory map
 *
 *  0x00000000 .. 0x01ffffff : Flash(32M)
 *  0x10000000 .. 0x11ffffff : SRAM(32M)
 *  0x20000000 .. 0x27ffffff : DDR(128M)
 *  0x40000000 .. 0x40000fff : UART(4K)
 *  0x50000000 .. 0x50000fff : Thomas Test Device(4K)
 */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x02000000
	RAM (rwx)  : ORIGIN = 0x10000000, LENGTH = 0x02000000
	DDR (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x08000000
}

__HEAP_SIZE  = 1 * 1024 * 1024;

ENTRY(Reset_Handler)

SECTIONS
{
	.vectors :
	{
		KEEP(*(startup))
	} > FLASH

	.text :
	{
		*(.text*)
		*(.rodata*)
	} > FLASH

	.copy.table :
	{
		. = ALIGN(4);
		__copy_table_start__ = .;
		LONG (__etext)
		LONG (__data_start__)
		LONG (__data_end__ - __data_start__)
		__copy_table_end__ = .;
	} > FLASH

	.zero.table :
	{
		. = ALIGN(4);
		__zero_table_start__ = .;
		LONG (__bss_start__)
		LONG (__bss_end__ - __bss_start__)
		__zero_table_end__ = .;
	} > FLASH

	__etext = .;

	.ARM_TLB (NOLOAD):
	{
		/* 4K * 4 TTB */
		. = ALIGN(0x4000);
	} > RAM

	.data : AT(__etext)
	{
		. = ALIGN(4);
		__data_start__ = .;
		*(.data*)
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;

	} > RAM

	.bss ALIGN(16):
	{
		__bss_start__ = .;
		*(.bss*)
		__bss_end__ = .;
	} > RAM

	.heap :
	{
		. = ALIGN(64);
		__end__ = .;
		PROVIDE(end = .);
		_heap_bottom = .;
		. = . + __HEAP_SIZE;
		_heap_top = .;
		. = ALIGN(8);
		__HeapLimit = .;
	} > RAM

	.abort_stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$ABORT_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$ABORT_STACK$$ZI$$Limit = .;
	} > RAM

	.undef_stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$UNDEF_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$UNDEF_STACK$$ZI$$Limit = .;
	} > RAM

	.irq_stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$IRQ_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$IRQ_STACK$$ZI$$Limit = .;
	} > RAM

	.fiq_stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$FIQ_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$FIQ_STACK$$ZI$$Limit = .;
	} > RAM

	.smc_stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$SMC_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$SMC_STACK$$ZI$$Limit = .;
	} > RAM

	.user_stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$USER_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$USER_STACK$$ZI$$Limit = .;
	} > RAM

	.stack (NOLOAD):
	{
		. = ALIGN(8);
		Image$$SVC_STACK$$ZI$$Base = .;
		. += 0x00001000;
		Image$$SVC_STACK$$ZI$$Limit = .;
	} > RAM
}
