/* Copyright 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __RISCV_CPU_H
#define __RISCV_CPU_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

void interrupt_global_enable(void);
void interrupt_global_disable(void);

#endif
