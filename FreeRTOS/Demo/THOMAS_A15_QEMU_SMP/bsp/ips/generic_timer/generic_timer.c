/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <generic_timer.h>
#include <console.h>
#include <interrupt.h>

void gtimer_set_cntp_freq(uint32_t freq)
{
	asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (freq));
	asm volatile("isb");
}

uint32_t gtimer_get_cntp_freq(void)
{
	uint32_t freq = 0;

	asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r" (freq));

	return freq;
}

void gtimer_set_cntp_ctl(uint32_t ctl)
{
	asm volatile("mcr p15, 0, %0, c14, c2, 1" : : "r" (ctl));
	asm volatile("isb");
}

uint32_t gtimer_get_cntp_ctl(void)
{
	uint32_t ctl = 0;

	asm volatile("mrc p15, 0, %0, c14, c2, 1" : "=r" (ctl));

	return ctl;
}

uint64_t gtimer_get_cntp_ct(void)
{
	uint64_t ct_val = 0;
	uint32_t l_val = 0;
	uint32_t h_val = 0;

	asm volatile("isb");
	asm volatile("mrrc p15, 0, %0, %1, c14" : "=r" (l_val), "=r" (h_val));

	ct_val = (uint64_t)h_val << 32 | (uint64_t)l_val;

	return ct_val;
}

void gtimer_set_cntp_tval(uint32_t t_val)
{
	asm volatile("mcr p15, 0, %0, c14, c2, 0" : : "r" (t_val));
	asm volatile("isb");
}

uint32_t gtimer_get_cntp_tval(void)
{
	uint32_t t_val = 0;

	asm volatile("mrc p15, 0, %0, c14, c2, 0" : "=r" (t_val));

	return t_val;
}

void gtimer_set_cntp_cval(uint64_t c_val)
{
	uint32_t l_val = 0;
	uint32_t h_val = 0;

	l_val = c_val >> 0;
	h_val = c_val >> 32;

	asm volatile("mcrr p15, 2, %0, %1, c14" : : "r" (l_val), "r" (h_val));
	asm volatile("isb");
}

uint64_t gtimer_get_cntp_cval(void)
{
	uint64_t c_val = 0;
	uint32_t l_val = 0;
	uint32_t h_val = 0;

	asm volatile("mrrc p15, 2, %0, %1, c14" : "=r" (l_val), "=r" (h_val));

	c_val = (uint64_t)h_val << 32 | (uint64_t)l_val;

	return c_val;
}

void gtimer_enable(void)
{
	uint32_t ctl = 0;

	ctl = gtimer_get_cntp_ctl();
	ctl |= G_TIMER_ENABLE;
	gtimer_set_cntp_ctl(ctl);
}

void gtimer_disable(void)
{
	uint32_t ctl = 0;

	ctl = gtimer_get_cntp_ctl();
	ctl &= ~G_TIMER_ENABLE;
	gtimer_set_cntp_ctl(ctl);
}

void gtimer_en_irq(void)
{
	uint32_t ctl = 0;

	ctl = gtimer_get_cntp_ctl();
	ctl &= ~G_TIMER_MASK;
	gtimer_set_cntp_ctl(ctl);
}

void gtimer_dis_irq(void)
{
	uint32_t ctl = 0;

	ctl = gtimer_get_cntp_ctl();
	ctl |= G_TIMER_MASK;
	gtimer_set_cntp_ctl(ctl);
}

uint64_t gtimer_usec2ticks(uint32_t usec)
{
	uint64_t ticks = 0;

	if (usec < 1000)
		ticks = ((usec * (gtimer_get_cntp_freq() / 1000)) + 500) / 1000;
	else
		ticks = ((usec / 10) * (gtimer_get_cntp_freq() / 100000));

	return ticks;
}

void gtimer_udelay(uint32_t us)
{
	uint64_t ticks = 0;

	gtimer_disable();
	gtimer_set_cntp_tval(10);
	gtimer_dis_irq();
	gtimer_enable();

	ticks = gtimer_get_cntp_ct();

	ticks += gtimer_usec2ticks(us);

	while (gtimer_get_cntp_ct() < ticks + 1)
		;

	gtimer_disable();
}

void gtimer_mdelay(uint32_t ms)
{
	gtimer_udelay(1000 * ms);
}

void gtimer_sdelay(uint32_t s)
{
	gtimer_udelay(1000000 * s);
}

static void soft_delay(void)
{
	volatile uint32_t i = 0;

	for (i = 0; i < 1000000; i++)
		;
}

void generic_timer_polling_test(void)
{
	/* -------------- tval test ------------------- */
	gtimer_disable();
	gtimer_dis_irq();
	gtimer_set_cntp_tval(100000);
	gtimer_enable();

	bm_printf_value_u32("generic timer ctl 1: ", gtimer_get_cntp_ctl());
	
	for (int i = 0; i < 10; i++) {
		uint64_t cnt = gtimer_get_cntp_ct();
		bm_printf_value_u32("cnt low 32bit : ", (uint32_t)cnt);
	}

	soft_delay();

	bm_printf_value_u32("generic timer ctl 2: ", gtimer_get_cntp_ctl());
	if (gtimer_get_cntp_ctl() & G_TIMER_IRQ) {
		bm_printf("timeout, tval test pass\n");
	}

	/* -------------- cval test ------------------- */

	gtimer_disable();
	gtimer_dis_irq();
	uint64_t tick = gtimer_get_cntp_ct() + 100000;
	gtimer_set_cntp_cval(tick);
	gtimer_enable();

	soft_delay();

	if (gtimer_get_cntp_ctl() & G_TIMER_IRQ) {
		bm_printf("timeout, cval test pass\n");
	} else {
		bm_printf("timeout, cval test fail\n");
	}

	gtimer_disable();
}

void generic_timer_isr_test(int irq, void *dev)
{
	bm_printf_value_u32("generic_timer_isr_test: ", irq);
	gtimer_disable();
}

void generic_timer_interrupt_test(void)
{
	gtimer_disable();
	request_irq(ARCH_TIMER_S_EL1_IRQ, generic_timer_isr_test, 0);
	/* Trigger interrupt */
	gtimer_en_irq();
	gtimer_set_cntp_tval(100000);
	gtimer_enable();
}

/**
 * Initializes the Generic Timer
 */
void generic_timer_init(void)
{
	bm_printf("enter generic_timer_init\n");
	gtimer_set_cntp_freq(PERIPH_CLK);

	//generic_timer_polling_test();
	//generic_timer_interrupt_test();

}
