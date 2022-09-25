/*
 * Header for common code.
 *
 * Author: Kang Zhigang <Zhigang.Kang@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include "riscv_asm.h"
#include "riscv_cpu.h"

/* Is there a UART? */
#define UART_IS_OK
#ifdef UART_IS_OK
#include "console.h"
#include "board-ns16550.h"
#endif

#define read_mreg64(addr)		(*(volatile uint64_t *)(addr))
#define write_mreg64(addr, val)		(*(volatile uint64_t *)(addr) = (uint64_t)(val))

#define read_mreg32(addr)		(*(volatile uint32_t *)(addr))
#define write_mreg32(addr, val)		(*(volatile uint32_t *)(addr) = (uint32_t)(val))

#define read_mreg16(addr)		(*(volatile u16 *)(addr))
#define write_mreg16(addr, val)		(*(volatile u16 *)(addr) = (u16)(val))

#define read_mreg8(addr)		(*(volatile uint8_t *)(addr))
#define write_mreg8(addr, val)		(*(volatile uint8_t *)(addr) = (uint8_t)(val))

#define BIT(nr)				(1UL << (nr))

#define MIN(a, b) ({			\
	typeof(a)_min1 = (a);		\
	typeof(b)_min2 = (b);		\
	_min1 < _min2 ? _min1 : _min2; })
#define MAX(a, b) ({			\
	typeof(a)_max1 = (a);		\
	typeof(b)_max2 = (b);		\
	_max1 > _max2 ? _max1 : _max2; })

#define ARRAY_SIZE(x)			(sizeof(x) / sizeof((x)[0]))

/* MACRO for size values */
#define SIZE_4K				0x00001000
#define SIZE_16K			0x00004000
#define SIZE_256K			0x00040000
#define SIZE_128K			0x00020000
#define SIZE_1M				0x00100000
#define SIZE_8M				0x00800000
#define SIZE_16M			0x01000000
#define SIZE_KB(x)			(0x00000400 * (x))
#define SIZE_MB(x)			(0x00100000 * (x))
#define SIZE_GB(x)			(0x40000000 * (x))

/* Definition of ascii */
#define NANO_TO_MICRO			1000

#ifdef UART_IS_OK
#define vs_getc()			console_getc()
#define vs_getc_timeout(x)		console_getc_timeout(x)
#define vs_putc(x)			console_putc(x)
#define vs_printf(fmt, arg ...)		console_printf(fmt, ##arg)
#define printf(fmt, arg ...)		console_printf(fmt, ##arg)
#else
#define vs_getc()			'0'
#define vs_getc_timeout(x)		'0'
#define vs_putc(x)			do {} while (0)
#define vs_printf(fmt, arg ...)		do {} while (0)
#endif

/* Is there a TIMER? */
#define TIMER_IS_OK
#include "timer.h"
#ifdef TIMER_IS_OK
#define	udelay(x)			timer_udelay(x)
#define	mdelay(x)			timer_mdelay(x)
#define	delay(x)			timer_sdelay(x)
#else
#define	udelay(x)			do {} while (0)
#define	mdelay(x)			do {} while (0)
#define	delay(x)			do {} while (0)
#endif

#endif /* __COMMON_H__ */
