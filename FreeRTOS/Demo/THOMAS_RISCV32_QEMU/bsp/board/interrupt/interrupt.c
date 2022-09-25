/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include "common.h"
#include "interrupt.h"
#include "plic.h"

struct plic_data board_plic = {
    .addr = PLIC_BASE,
    .num_src = PLIC_SOURCE_NUMBER,
};

struct callback {
    isr_t    isr;
    void *param;
};

static struct callback callback[PLIC_SOURCE_NUMBER] = {0};

void external_interrupt_handler(void)
{
    struct callback *entry;
    uint32_t irq_num = plic_get_claim(&board_plic);

    vs_printf("external interrupt handler, pending irq: %d\n", irq_num);

    if (irq_num >= 0 && irq_num < PLIC_SOURCE_NUMBER) {
        entry = &callback[irq_num];
        entry->isr(irq_num, entry->param);
    }

    plic_set_claim(&board_plic, irq_num);
}

int request_irq(int irq_num, isr_t isr, void *param)
{
    struct callback *entry;
    uint32_t word_index, bit_index, value;

    if (irq_num < 0 || irq_num >= PLIC_SOURCE_NUMBER)
        return -1;

    entry = &callback[irq_num];
    entry->isr = isr;
    entry->param = param;

    /* Enable this irq */
    word_index = (irq_num) / 32;
    bit_index = irq_num - (word_index * 32);
    value = plic_get_ie(&board_plic, 0, word_index);
    value |= (1 << bit_index);
    plic_set_ie(&board_plic, 0, word_index, value);

    return 0;
}

int free_irq(int irq_num)
{
    struct callback *entry;
    uint32_t word_index, bit_index, value;

    if (irq_num < 0 || irq_num >= PLIC_SOURCE_NUMBER)
        return -1;

    entry = &callback[irq_num];
    entry->isr = 0;
    entry->param = 0;

    /* Disable this irq */
    word_index = (irq_num) / 32;
    bit_index = irq_num - (word_index * 32);
    value = plic_get_ie(&board_plic, 0, word_index);
    value &= ~(1 << bit_index);
    plic_set_ie(&board_plic, 0, word_index, value);

    return 0;
}

void interrupt_init(void)
{
    int i;

    /* Enable externel interrupt from PLIC */
    csr_set(CSR_MIE, MIP_MEIP);

	/* Configure default priorities of all IRQs */
	for (i = 1; i <= board_plic.num_src; i++)
		plic_set_priority(&board_plic, i, PLIC_DEFAULT_PRIORITY);

    /* Configure default threshold */
    plic_set_thresh(&board_plic, 0, PLIC_DEFAULT_THRESH);
}
