/* Copyright 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

#include "riscv_cpu.h"
#include "riscv_asm.h"

void interrupt_global_enable(void)
{
    uintptr_t m;

    __asm__ volatile("csrrs %0, mstatus, %1"
                     : "=r"(m)
                     : "r"(MSTATUS_MIE));
}

void interrupt_global_disable(void)
{
    uintptr_t m;

    __asm__ volatile("csrrc %0, mstatus, %1"
                     : "=r"(m)
                     : "r"(MSTATUS_MIE));
}
