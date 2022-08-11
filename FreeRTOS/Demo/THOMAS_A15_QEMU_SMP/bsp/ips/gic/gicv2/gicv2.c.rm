/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <gicv2.h>
#include <console.h>

static struct israction_s intr_isr[INTR_ENTRY_NUM];

void enable_interrupt(void)
{
	asm volatile("CPSIE if");
	asm volatile("isb");
}

void disable_interrupt(void)
{
	asm volatile("CPSID if");
	asm volatile("isb");
}

void gic_fiq_handler(void)
{
	struct israction_s *action;

	unsigned int intr_ack = GICInterface->IAR & 0x3ff;

	bm_printf_value_u32("gic_fiq_handler number: ", intr_ack);
	action = &intr_isr[intr_ack];
	action->handler(intr_ack, action->dev_id);

	GICInterface->EOIR |= intr_ack;
}

void gic_irq_handler()
{
	struct israction_s *action;

	unsigned int intr_ack = GICInterface->IAR & 0x3ff;

	bm_printf_value_u32("gic_irq_handler number: ", intr_ack);
	action = &intr_isr[intr_ack];
	action->handler(intr_ack, action->dev_id);

	GICInterface->EOIR |= intr_ack;
}

void request_irq(uint32_t irq_num, enum isr_type_t isr_type,
	isr_handler_t cur_src_isr, void *dev_id)
{
	uint32_t reg_number = irq_num / 32;
	uint32_t offset = irq_num - (reg_number * 32);
	GICDistributor->ISENABLER[reg_number] |= (1 << offset);
	intr_isr[irq_num].handler = cur_src_isr;
	intr_isr[irq_num].isr_type = isr_type;
	intr_isr[irq_num].dev_id = dev_id;
}

void free_irq(uint32_t irq_num)
{
	uint32_t reg_number = irq_num / 32;
	uint32_t offset = irq_num - (reg_number * 32);
	GICDistributor->ICENABLER[reg_number] |= (1 << offset);
	intr_isr[irq_num].handler = 0;
}


void gicv2_dump_gicd(void)
{
	bm_printf_value_u32("GICD CTLR: ", GICDistributor->CTLR);
	bm_printf_value_u32("GICD TYPER: ", GICDistributor->TYPER);
}

void gicv2_dump_gicc(void)
{
	bm_printf_value_u32("GICC CTLR: ", GICInterface->CTLR);
	bm_printf_value_u32("GICC PMR: ", GICInterface->PMR);
	bm_printf_value_u32("GICC BPR: ", GICInterface->BPR);
	bm_printf_value_u32("GICC IAR: ", GICInterface->IAR);
}

#define PPI_TEST_IRQ_NUMBER 29

void gicv2_test2_isr(uint32_t irq, void *dev)
{
	bm_printf_value_u32("gicv2_test2_isr: ", irq);
}

void gicv2_test2(void)
{
	request_irq(PPI_TEST_IRQ_NUMBER, ISR_TYPE_FIQ, gicv2_test2_isr, 0);
	/* Trigger interrupt */
	GICDistributor->ISPENDR[0] = 1 << PPI_TEST_IRQ_NUMBER;
}

void gicv2_test(void)
{
	/* Trigger interrupt */
	GICDistributor->ISPENDR[0] = 1 << PPI_TEST_IRQ_NUMBER;
}


/**
 * Initializes the GICV2
 */
void gicv2_init(void)
{
	bm_printf("enter gicv2_init\n");

	/* Enable group0 and group1 */
	GICDistributor->CTLR = 0x3;

	/* Enable group0 and group1, enable fiq */
	GICInterface->CTLR = 0xB;

	/* Set priority mask */
	GICInterface->PMR = 0xff;

	/* Eenable interrupt */
	enable_interrupt();

	//gicv2_test2();
}
