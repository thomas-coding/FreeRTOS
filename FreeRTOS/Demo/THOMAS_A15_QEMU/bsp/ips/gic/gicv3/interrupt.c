/*
 * Generic driver code for interrupt
 *
 * Author: Kang Zhigang <Zhigang.Kang@verisilicon.com>
 *
 * Copyright (C) 2021 VeriSilicon Microelectronics (Shanghai) Co., Ltd.
 *
 */

#include "interrupt.h"
#include "gicv3.h"
#include "gicv3-gicc.h"

struct israction_s intr_isr[INTR_ENTRY_NUM] = {NULL};
void gic_test(void);

void gic_init(void)
{
	uint32_t cpu_id = cpu_id_get();

	/* Set the priority mask low enough */
	gicc_pmr_set(0xff);
	asm volatile("isb");

	gicd_enable(gicdctlr_enable_all);

	gicc_igrpen0_set(igrp_enable);
	gicc_igrpen1_set(igrp_enable);

	/* Already set in startup.s */
	/* gicc_mgrpen1_set(igrp_enable_grp1ns | igrp_enable_grp1s); */

	/* By defualt, configure all interrupts to group1 secure */
	private_intr_security_block_set(cpu_id, gicigroupr_g1s);
	spi_security_all_set(gicigroupr_g1s);

	interrupts_enable();

	//gic_test();
}

void gic_spi_conf(uint32_t spi_id, uint32_t target_cpu)
{
	uint32_t afft = target_cpu;

#if (CONFIG_NUM_CPUS > 1)	/* APS */
	afft |= 0x100;
#endif

	spi_route_set(spi_id, afft, gicdirouter_mode_specific);

	spi_priority_set(spi_id, 0);
	spi_configure(spi_id, gicdicfgr_level);
	spi_enable(spi_id);
}

void gic_private_intr_conf(uint32_t cpu_id, uint32_t ppi_id)
{
	private_intr_priority_set(cpu_id, ppi_id, 0);
	private_intr_enable(cpu_id, ppi_id);
}

void gic_isr_install(uint32_t irq_num, enum isr_type_t isr_type, uint32_t cpu_id,
	isr_handler_t cur_src_isr, void *dev_id)
{
	if (irq_num >= INTR_ENTRY_NUM) {
		//vs_printf("IntrID:%d is not supported\n", irq_num);
		return;
	}

#if 0 //only on secure
	if (get_secure_state() && (isr_type == ISR_TYPE_FIQ)) {
		/* Non-Secure only support IRQ */
		//vs_printf("Non-Secure only support IRQ, switch to IRQ!\n");
		isr_type = ISR_TYPE_IRQ;
	}
#endif

	if (isr_type == ISR_TYPE_FIQ) {
		if (irq_num < 32)
			private_intr_security_set(cpu_id, irq_num,
				gicigroupr_g0s);
		else if (irq_num < INTR_ENTRY_NUM)
			spi_security_set(irq_num, gicigroupr_g0s);
	} else {
#if 0 //only on secure
		if (get_secure_state()) {
			switch_2_secure();

			if (irq_num < 32)
				private_intr_security_set(cpu_id, irq_num,
					gicigroupr_g1ns);
			else if (irq_num < INTR_ENTRY_NUM)
				spi_security_set(irq_num, gicigroupr_g1ns);

			switch_2_non_secure();
		} else {
#endif
			if (irq_num < 32)
				private_intr_security_set(cpu_id, irq_num,
					gicigroupr_g1s);
			else if (irq_num < INTR_ENTRY_NUM)
				spi_security_set(irq_num, gicigroupr_g1s);
#if 0
		}
#endif
	}

	intr_isr[irq_num].handler = cur_src_isr;
	intr_isr[irq_num].isr_type = isr_type;
	intr_isr[irq_num].dev_id = dev_id;
}

void gic_isr_unstall(uint32_t irq_num, uint32_t cpu_id)
{
	if (irq_num >= INTR_ENTRY_NUM) {
		//vs_printf("IntrID:%d is not supported\n", irq_num);
		return;
	}

	if (irq_num < 32)
		private_intr_disable(cpu_id, irq_num);
	else if (irq_num < INTR_ENTRY_NUM)
		spi_disable(irq_num);

	intr_isr[irq_num].handler = NULL;
	intr_isr[irq_num].isr_type = ISR_TYPE_IRQ;
	intr_isr[irq_num].dev_id = NULL;
}

void request_irq_origin(uint32_t irq_num, enum isr_type_t isr_type,
	isr_handler_t cur_src_isr, void *dev_id)
{
	uint32_t cpu_id = cpu_id_get();

	gic_isr_install(irq_num, isr_type, cpu_id, cur_src_isr, dev_id);

	if (irq_num >= 32)
		gic_spi_conf(irq_num, cpu_id);
	else
		gic_private_intr_conf(cpu_id, irq_num);
}

int request_irq(int irq_num, isr_handler_t isr, void *param)
{
	if (irq_num < 0 || irq_num >= INTR_ENTRY_NUM)
		return -1;

	request_irq_origin(irq_num, ISR_TYPE_IRQ, isr, param);
	return 0;
}

void free_irq(uint32_t irq_num)
{
	uint32_t cpu_id = cpu_id_get();

	gic_isr_unstall(irq_num, cpu_id);
}

void gic_irq_handler(void)
{
	struct israction_s *action = NULL;
	uint32_t intr_ack = gicc_iar1_get() & 0x3ff;

	if ((intr_ack >= 1020) && (intr_ack <= 1023))
		return;

	action = &intr_isr[intr_ack];

	if (action->handler != NULL && action->isr_type == ISR_TYPE_IRQ)
		action->handler(intr_ack, action->dev_id);

	gicc_eoir1_set(intr_ack);
}

void gic_fiq_handler(void)
{
	uint32_t aliased = 0;
	struct israction_s *action = NULL;
	uint32_t intr_ack = gicc_iar0_get();

	if ((1020 <= intr_ack) && (intr_ack <= 1023)) {
		intr_ack = gicc_iar1_get();
		aliased = 1;

		if ((1020 <= intr_ack) && (intr_ack <= 1023))
			return;
	}

	action = &intr_isr[intr_ack];

	if (action->handler != NULL && action->isr_type == ISR_TYPE_FIQ)
		action->handler(intr_ack, action->dev_id);

	if (aliased == 0)
		gicc_eoir0_set(intr_ack);
	else
		gicc_eoir1_set(intr_ack);
}


void gic_set_interrupt_priority(uint32_t irq_num, uint32_t priority)
{
	uint32_t cpu_id = cpu_id_get();

	if (irq_num >= 32)
		spi_priority_set(irq_num, priority);
	else
		private_intr_priority_set(cpu_id, irq_num, priority);
}

uint32_t gic_get_interrupt_priority(uint32_t irq_num)
{
	uint32_t cpu_id = cpu_id_get();

	if (irq_num >= 32)
		return spi_priority_get(irq_num);
	else
		return private_intr_priority_get(cpu_id, irq_num);
}

void gic_set_interrupt_pending(uint32_t irq_num)
{
	uint32_t cpu_id = cpu_id_get();

	if (irq_num >= 32)
		spi_pending_set(irq_num);
	else
		private_intr_pending_set(cpu_id, irq_num);
}

uint32_t gic_get_interrupt_pending(uint32_t irq_num)
{
	uint32_t cpu_id = cpu_id_get();

	if (irq_num >= 32)
		return spi_pending_get(irq_num);
	else
		return private_intr_pending_get(cpu_id, irq_num);
}

#define TEST_IRQ_NUMBER (2 + 16)
#include <console.h>
void test_isr(int irq, void *dev)
{
	bm_printf_value_u32("test_isr: ", irq);
}

void gic_test(void)
{
	request_irq(TEST_IRQ_NUMBER, test_isr, 0);
	bm_printf("before set pending\n");
	gic_set_interrupt_pending(TEST_IRQ_NUMBER);
	bm_printf("after set pending\n");
}
