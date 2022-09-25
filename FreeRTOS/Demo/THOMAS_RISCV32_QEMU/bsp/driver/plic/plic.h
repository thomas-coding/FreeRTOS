/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#ifndef __IRQCHIP_PLIC_H__
#define __IRQCHIP_PLIC_H__

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

struct plic_data {
    unsigned long addr;
    unsigned long num_src;
};

/* So far, priorities on all consumers of these functions fit in 8 bits. */
void plic_priority_save(const struct plic_data *plic, uint8_t *priority);

void plic_priority_restore(const struct plic_data *plic, const uint8_t *priority);

void plic_context_save(const struct plic_data *plic, int context_id,
               uint32_t *enable, uint32_t *threshold);

void plic_context_restore(const struct plic_data *plic, int context_id,
              const uint32_t *enable, uint32_t threshold);

int plic_context_init(const struct plic_data *plic, int context_id,
              bool enable, uint32_t threshold);

int plic_warm_irqchip_init(const struct plic_data *plic,
               int m_cntx_id, int s_cntx_id);

int plic_cold_irqchip_init(const struct plic_data *plic);

void plic_set_ie(const struct plic_data *plic, uint32_t cntxid, uint32_t word_index, uint32_t val);
uint32_t plic_get_ie(const struct plic_data *plic, uint32_t cntxid, uint32_t word_index);
uint32_t plic_get_claim(const struct plic_data *plic);
void plic_set_claim(const struct plic_data *plic, uint32_t source);
uint32_t plic_get_priority(const struct plic_data *plic, uint32_t source);
void plic_set_priority(const struct plic_data *plic, uint32_t source, uint32_t val);
uint32_t plic_get_thresh(const struct plic_data *plic, uint32_t cntxid);
void plic_set_thresh(const struct plic_data *plic, uint32_t cntxid, uint32_t val);

#endif
