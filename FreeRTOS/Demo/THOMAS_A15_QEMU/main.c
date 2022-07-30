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

static void user_task( void *pvParameters )
{
	uint32_t task_number = (uint32_t)pvParameters;
	uint32_t count = 0;

	//vs_printf("\n%s %s\n", __DATE__, __TIME__);
	//vs_printf("task %d create succeed\n", task_number);
	while(1) {
		bm_printf_value_u32("task running :",task_number);
		vTaskDelay( pdMS_TO_TICKS(1000 * task_number) );
		count++;
	}
}

int main(void)
{

	platform_init();
	bm_printf("enter freertos main\n");

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
