/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef GENERIC_TIMER_H
#define GENERIC_TIMER_H

#include <stddef.h>
#include <stdint.h>

/* Running on QEMU virt, I don't know freq, suitable for my machine */
#define PERIPH_CLK		60000000

#define G_TIMER_ENABLE	(1 << 0)
#define G_TIMER_MASK	(1 << 1)
#define G_TIMER_IRQ	(1 << 2)

/* From include/hw/arm/virt.h */
#define ARCH_TIMER_VIRT_IRQ   (11 + 16)
#define ARCH_TIMER_S_EL1_IRQ  (13 + 16)
#define ARCH_TIMER_NS_EL1_IRQ (14 + 16)
#define ARCH_TIMER_NS_EL2_IRQ (10 + 16)

void generic_timer_init(void);
void gtimer_udelay(uint32_t us);
void gtimer_mdelay(uint32_t ms);
void gtimer_sdelay(uint32_t s);

void gtimer_set_clk(void);
void gtimer_set_cntp_freq(uint32_t freq);
uint32_t gtimer_get_cntp_freq(void);
void gtimer_set_cntp_ctl(uint32_t ctl);
uint32_t gtimer_get_cntp_ctl(void);
uint64_t gtimer_get_cntp_ct(void);
void gtimer_set_cntp_tval(uint32_t t_val);
uint32_t gtimer_get_cntp_tval(void);
void gtimer_set_cntp_cval(uint64_t c_val);
uint64_t gtimer_get_cntp_cval(void);

void gtimer_enable(void);
void gtimer_disable(void);
uint64_t gtimer_get_ticks(void);

void gtimer_en_irq(void);
void gtimer_dis_irq(void);

void gtimer_udelay(uint32_t us);
void gtimer_mdelay(uint32_t ms);
void gtimer_sdelay(uint32_t s);

#endif
