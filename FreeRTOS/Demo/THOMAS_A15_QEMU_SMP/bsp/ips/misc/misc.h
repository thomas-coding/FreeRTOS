/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef MISC_H
#define MISC_H

#include <stddef.h>
#include <stdint.h>

void misc_init(void);
uint32_t is_svc_mode(void);
uint32_t is_irq_mode(void);
#endif
