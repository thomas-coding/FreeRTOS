/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Core-Local Interruptor (CLINT) driver
 *
 */

#include "common.h"
#include "aclint.h"

struct aclint_mtimer_data gp_mtimer;

static uint64_t mtimer_time_rd32(volatile uint32_t *addr)
{
    uint32_t lo, hi;

    do {
        hi = read_mreg32((uint32_t *)addr + 1);
        lo = read_mreg32((uint32_t *)addr);
    } while (hi != read_mreg32((uint32_t *)addr + 1));

    return ((uint64_t)hi << 32) | (uint64_t)lo;
}

static void mtimer_time_wr32(bool timecmp, uint64_t value, uint32_t *addr)
{
    write_mreg32((void *)(addr), (timecmp) ? -1U : 0U);
    write_mreg32((char *)(addr) + 0x04, (uint32_t)(value >> 32));
    write_mreg32((void *)(addr), (uint32_t)value);
}

uint64_t mtimer_value(void)
{
    uint32_t *timer_addr = (void *)gp_mtimer.mtime_addr;

    /* Read MTIMER Time Value */
    return gp_mtimer.time_rd(timer_addr);
}

void mtimer_stop(void)
{
    uint32_t *time_cmp = (void *)gp_mtimer.mtimecmp_addr;

    /* Clear MTIMER Time Compare */
    gp_mtimer.time_wr(true, -1ULL, time_cmp);
}

void mtimer_start(uint64_t next_count)
{
    uint32_t *time_cmp = (void *)gp_mtimer.mtimecmp_addr;

    /* Program MTIMER Time Compare */
    gp_mtimer.time_wr(true, next_count, time_cmp);
}

int aclint_mtimer_cold_init(struct aclint_mtimer_data *mt)
{
    gp_mtimer.mtime_addr = mt->mtime_addr;
    gp_mtimer.mtimecmp_addr = mt->mtimecmp_addr;
    gp_mtimer.time_rd = mtimer_time_rd32;
    gp_mtimer.time_wr = mtimer_time_wr32;
    return 0;
}
