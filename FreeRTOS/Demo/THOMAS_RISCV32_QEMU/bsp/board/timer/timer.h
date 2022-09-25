/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __TIMER_H__
#define __TIMER_H__
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "addr_map.h"

#define TIMER_CLK		10000000 //10M for qemu

void timer_init(void);
int64_t get_ticks(void);

void timer_udelay(uint32_t us);
void timer_mdelay(uint32_t ms);
void timer_sdelay(uint32_t s);

void timer_start(uint64_t next_tick);
void timer_start_ms(uint32_t ms);
void timer_stop(void);

#endif /* #ifndef __TIMER_H__ */
