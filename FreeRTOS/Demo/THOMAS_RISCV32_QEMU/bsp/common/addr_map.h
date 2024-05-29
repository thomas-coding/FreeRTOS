/*
 * Header for address map
 *
 * Copyright (C) 2022 VeriSilicon Holdings Co., Ltd.
 *
 */

#ifndef __ADDR_MAP_H__
#define __ADDR_MAP_H__

#define UART0_BASE              0x10000000
#define ACLINT_MTIME_BASE       0x02004000
#define PLIC_BASE               0x0c000000

#ifdef __ASSEMBLER__
#define CONS(NUM, TYPE)NUM
#else
#define CONS(NUM, TYPE)NUM##TYPE
#endif /* __ASSEMBLER__ */

#define PRIM_HART			0

#define CLINT_ADDR			CONS(0x02000000, UL)
#define CLINT_MSIP			CONS(0x0000, UL)
#define CLINT_MTIMECMP		CONS(0x4000, UL)
#define CLINT_MTIME			CONS(0xbff8, UL)

#endif /* __ADDR_MAP_H__ */
