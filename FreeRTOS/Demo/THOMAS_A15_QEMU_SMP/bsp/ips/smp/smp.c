/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#include <smp.h>
#include <console.h>
#include <interrupt.h>

void core_sgi_handle(int id, void *dev_id)
{
	uint32_t cpu_id = 0;

	cpu_id = cpu_id_get();
	bm_printf_value_u32("Hello IRQ from CPU:", cpu_id);
}

void smp_test(void)
{
    /* Registe handle for cpu1 irq, sgi interrupt number 0 */
	gic_isr_install(0, ISR_TYPE_IRQ, 1, core_sgi_handle, NULL);
	gic_private_intr_conf(1, 0);

    /* Generate SGI to core 1, interrupt number is 0 */
	gicc_sgi1r_set(0, 0, 0, 0, (1 << 1), 0);
}

int second_main(void)
{
	bm_printf("second main\n");
    /* Registe handle for cpu0 irq, sgi interrupt number 0 */
	gic_isr_install(0, ISR_TYPE_IRQ, 0, core_sgi_handle, NULL);
	gic_private_intr_conf(0, 0);

    bm_printf("generate sgi to primiray core\n");
    /* Generate SGI to core 0, interrupt number is 0 */
	gicc_sgi1r_set(0, 0, 0, 0, (1 << 0), 0);
	return 0;
}

/**
 * Initializes the SMP
 */
void smp_init(void)
{
	bm_printf("smp init\n");
    //smp_test();
}
