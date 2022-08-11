/*
 * Generic driver code for interrupt
 *
 * Author: Kang Zhigang <Zhigang.Kang@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

//#include "common.h"
#include <stdint.h>
#include <stddef.h>
#include <gicv3_platform_def.h>
#include <gicv3-gicc.h>

#define INTR_ENTRY_NUM	151

//typedef void (*isr_handler_t)(uint32_t, void *);
typedef void (*isr_handler_t)(int irq_num, void *param);

enum isr_type_t {
	ISR_TYPE_IRQ = 0,
	ISR_TYPE_FIQ = 1
};

/* struct israction_s - per interrupt action descriptor */
struct israction_s {
	isr_handler_t handler;	/* interrupt handler function */
	enum isr_type_t	isr_type;	/* interrupt type */
	void *dev_id;	/* used to identify the device */
};

void gic_init(void);
void gic_irq_handler(void);
void gic_fiq_handler(void);

void gic_spi_conf(uint32_t spi_id, uint32_t target_cpu);
void gic_private_intr_conf(uint32_t cpu_id, uint32_t ppi_id);

void gic_isr_install(uint32_t irq_num, enum isr_type_t isr_type, uint32_t cpu_id,
	isr_handler_t cur_src_isr, void *dev_id);
void gic_isr_unstall(uint32_t irq_num, uint32_t cpu_id);

void request_irq_origin(uint32_t id, enum isr_type_t isr_type,
	isr_handler_t cur_src_isr, void *dev_id);
void free_irq(uint32_t id);

typedef void (*isr_handler_t)(int irq_num, void *param);
int request_irq(int irq_num, isr_handler_t isr, void *param);

void gic_set_interrupt_priority(uint32_t irq_num, uint32_t priority);
uint32_t gic_get_interrupt_priority(uint32_t irq_num);
void gic_set_interrupt_pending(uint32_t irq_num);
uint32_t gic_get_interrupt_pending(uint32_t irq_num);
#endif
