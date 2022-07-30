/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef MMU_H
#define MMU_H

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

#define DOMAIN_CTL_NOACCESS(x)	(((unsigned int)0x0L) << ((x) << 1L))
#define DOMAIN_CTL_CLIENT(x)	(((unsigned int)0x1L) << ((x) << 1L))
#define DOMAIN_CTL_MANAGER(x)	(((unsigned int)0x3L) << ((x) << 1L))

/*
 * TEX[2:0]=100, C=0, B=1
 * AP[2:0]=011, XN=0, PXN=0
 * Domain=0000, S=1, NG=0, NS=0
 * Template descriptor for:
 * normal memory
 * outer non-cacheable, inner write-back, write-allocate
 * read/write at any privilege level
 * executable, privileged executable
 * shareable, global
 */
#define L1_COHERENT		0x00014c06

/*
 * TEX[2:0]=000, C=1, B=1
 * AP[2:0]=011, XN=1, PXN=0
 * Domain=0000, S=0, NG=0, NS=0
 * Template descriptor for:
 * normal memory
 * outer and inner write-back, no write-allocate
 * read/write at any privilege level
 * execute-never, privileged execute-never
 * not shareable, global
 */
#define L1_NONCOHERENT		0x00000c1e

/*
 * TEX[2:0]=000, C=0, B=1
 * AP[2:0]=011, XN=0, PXN=0
 * Domain=0000, S=0, NG=0, NS=0
 * Template descriptor for:
 * shareable device memory,
 * read/write at any privilege level,
 * execute-never, privileged execute-never
 * global
 */
#define L1_DEVICE		0x00000c06

#define TTB_SECIION_MASK	(0xfff00000)

struct mmu_section_entry {
	unsigned int virtual_addr;   /* virtual address, [31:20] */
	unsigned int  phy_addr;       /* physic address, [31:20] */
	unsigned int  region_size;
	unsigned int  attribute;
};

void mmu_init(void);
#endif
