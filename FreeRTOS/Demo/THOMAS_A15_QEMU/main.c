/*
 * FreeRTOS V202104.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#include <FreeRTOS.h>
#include <task.h>
#include<FreeRTOSConfig.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "platform.h"
#include "console.h"
#include "console2.h"
#include "tiny_console.h"

#include "interrupt.h"

static void user_task( void *pvParameters )
{
	uint32_t task_number = (uint32_t)pvParameters;
	uint32_t count = 0;

	t_printf("\n%s %s\n", __DATE__, __TIME__);
	t_printf("task %d create succeed\n", task_number);
	while(1) {
		t_printf("core[%d]: task running\n", cpu_id_get());
		vTaskDelay( pdMS_TO_TICKS(1000 * task_number) );
		count++;
	}
}

void second_core_main(void)
{
	uint32_t count = 0;
	t_printf("enter second core main\n");
	while(1) {
		t_printf("core[%d]: second core running \n", cpu_id_get());
		for(int i = 0; i < 0xffffff; i++)
			count++;
	}
}

extern void smp_test(void);
/* The base address of mailbox */
#define SRAM_BASE		0x10000000
#define SRAM_SIZE		0x02000000 /* 32MB */
#define SMP_MAILBOXI_REG_OFFSET	(SRAM_SIZE - 0x40) //64 bytes for mailbox
#define SMP_MAILBOXI_REG	(SRAM_BASE + SMP_MAILBOXI_REG_OFFSET)
void start_second_core(void * enter)
{
	t_printf("start second core\n");
	//set entry to mailbox
	*(volatile uint32_t*)(SMP_MAILBOXI_REG) = (uint32_t)enter;

    /* Registe handle for cpu1 irq, sgi interrupt number 0 */
	//gic_isr_install(0, ISR_TYPE_IRQ, 1, NULL, NULL);
	//gic_private_intr_conf(1, 0);
	gic_private_intr_conf(1, 0);

    /* Generate SGI to core 1, interrupt number is 0 */
	gicc_sgi1r_set(0, 0, 0, 0, (1 << 1), 0);
}

int main(void)
{

	platform_init();
	bm_printf("enter freertos main\n");
	t_printf("t printf enter main....\n");
#ifdef CONFIG_LIBC_STUB
	printf("print enter main\n");
#endif

#ifdef CONFIG_SMP
	start_second_core((void *)second_core_main);
#endif

#ifdef CONFIG_TINY_CONSOLE
	tiny_uart_console();
#endif

	/* Create that task that handles the console itself. */
	xTaskCreate(user_task,  /* The task that implements the command console. */
		"Task1",    /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
		1000,       /* The size of the stack allocated to the task. */
		(void *)1,  /* The parameter is not used, so NULL is passed. */
		3,          /* The priority allocated to the task. */
		NULL );     /* A handle is not required, so just pass NULL. */

	xTaskCreate(user_task,  /* The task that implements the command console. */
		"Task2",    /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
		1000,       /* The size of the stack allocated to the task. */
		(void *)2,  /* The parameter is not used, so NULL is passed. */
		4,          /* The priority allocated to the task. */
		NULL );     /* A handle is not required, so just pass NULL. */

	vTaskStartScheduler();

	for ( ;; )
	;

	return 0;
}
