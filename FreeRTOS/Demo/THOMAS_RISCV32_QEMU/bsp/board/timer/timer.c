/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Core-Local Interruptor (CLINT) driver
 *
 */

#include "common.h"
#include "timer.h"
#include "aclint.h"

struct aclint_mtimer_data board_timer = {
    .mtime_freq = TIMER_CLK,
    .mtime_addr = ACLINT_MTIME_BASE + ACLINT_DEFAULT_MTIME_OFFSET,
    .mtimecmp_addr = ACLINT_MTIME_BASE + ACLINT_DEFAULT_MTIMECMP_OFFSET,
};

int64_t get_ticks(void)
{
    uint32_t lo, hi, tmp;

    __asm__ __volatile__("1:\n"
                 "rdtimeh %0\n"
                 "rdtime %1\n"
                 "rdtimeh %2\n"
                 "bne %0, %2, 1b"
                 : "=&r"(hi), "=&r"(lo), "=&r"(tmp));
    return ((uint64_t)hi << 32) | lo;
}

void timer_udelay(uint32_t us)
{
    uint64_t start_val, delta;

    /* Save starting timer value */
    start_val = get_ticks();

    /* Compute desired timer value delta */
    delta = ((uint64_t)board_timer.mtime_freq * (uint64_t)us) / 1000000;

    /* Busy loop until desired timer value delta reached */
    while ((get_ticks() - start_val) < delta)
        asm volatile ("" : : : "memory");
}

void timer_mdelay(uint32_t ms)
{
    timer_udelay(1000 * ms);
}

void timer_sdelay(uint32_t s)
{
    timer_udelay(1000000 * s);
}

void timer_start(uint64_t next_tick)
{
    mtimer_start(next_tick);
    csr_set(CSR_MIE, MIP_MTIP);
}

void timer_start_ms(uint32_t ms)
{
    uint64_t next_tick;

    next_tick = get_ticks() + ((uint64_t)board_timer.mtime_freq * (uint64_t)ms) / 1000;
    mtimer_start(next_tick);
    csr_set(CSR_MIE, MIP_MTIP);
}

void timer_stop(void)
{
    mtimer_stop();
    csr_clear(CSR_MIP, MIP_STIP);
    csr_clear(CSR_MIE, MIP_MTIP);
}

void timer_interrupt_handler(void)
{
    vs_printf("timer interrupt handler called\n");
    timer_stop();
}

void timer_init(void)
{
    aclint_mtimer_cold_init(&board_timer);
}
