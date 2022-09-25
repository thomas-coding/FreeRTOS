/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PLIC_SOURCE_NUMBER          127
#define PLIC_DEFAULT_PRIORITY       2
#define PLIC_DEFAULT_THRESH         0

typedef void (*isr_t)(int irq_num, void *param);

int request_irq(int irq_num, isr_t isr, void *param);
int free_irq(int irq_num);

void interrupt_init(void);
void external_interrupt_handler(void);
#endif
