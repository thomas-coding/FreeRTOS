/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __ACLINT_H__
#define __ACLINT_H__
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define ACLINT_MTIMER_ALIGN		0x8
#define ACLINT_MTIMER_MAX_HARTS		4095

#define ACLINT_DEFAULT_MTIME_OFFSET	0x7ff8
#define ACLINT_DEFAULT_MTIME_SIZE	0x8
#define ACLINT_DEFAULT_MTIMECMP_OFFSET	0x0000
#define ACLINT_DEFAULT_MTIMECMP_SIZE	0x7ff8

#define CLINT_MTIMER_OFFSET		0x4000

struct aclint_mtimer_data {
    unsigned long mtime_freq;
    unsigned long mtime_addr;
    unsigned long mtimecmp_addr;

    uint64_t (*time_rd)(volatile uint32_t *addr);
    void (*time_wr)(bool timecmp, uint64_t value, uint32_t *addr);
};

uint64_t mtimer_value(void);
void mtimer_start(uint64_t next_count);
void mtimer_stop(void);
int aclint_mtimer_cold_init(struct aclint_mtimer_data *mt);

#endif /* #ifndef __ACLINT_H__ */
