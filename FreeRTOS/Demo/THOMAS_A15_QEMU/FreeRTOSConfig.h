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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See https://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configASSERT_DEFINED                     1
extern void vAssertCalled(void);
#define configASSERT(x) do {if ((x) == 0) vAssertCalled(); } while (0)
#define configQUEUE_REGISTRY_SIZE                20
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  2

#define configUSE_IDLE_HOOK                      1
#define configUSE_TICK_HOOK                      1
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0
#define configCPU_CLOCK_HZ                       (24000000UL)
#define configTICK_RATE_HZ                       ((TickType_t) 1000)
#define configMINIMAL_STACK_SIZE                 ((unsigned short) 2000)
#define configTOTAL_HEAP_SIZE                    ((size_t) (1*1024*1024))
#define configMAX_TASK_NAME_LEN                  (10)
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
//#define configIDLE_SHOULD_YIELD                  1
#define configUSE_CO_ROUTINES                    0
#define configSAFE_PRINT                         0
#define configSTACK_DEPTH_TYPE                   uint32_t

#define configMAX_PRIORITIES                     (10)
#define configMAX_CO_ROUTINE_PRIORITIES          (2)
#define configTIMER_QUEUE_LENGTH                 20
#define configTIMER_TASK_PRIORITY                (configMAX_PRIORITIES - 1)
#define configUSE_COUNTING_SEMAPHORES            1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION          1
#define  configNUM_TX_DESCRIPTORS                15
#define configSTREAM_BUFFER_TRIGGER_LEVEL_TEST_MARGIN 2

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define configUSE_POSIX_ERRNO                   1
#define configUSE_APPLICATION_TASK_TAG          1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_TIMERS                        1
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_uxTaskGetStackHighWaterMark2    1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1

unsigned long ulGetRunTimeCounterValue(void); /* Prototype of function that returns run time counter. */

#ifdef HEAP3
#define xPortGetMinimumEverFreeHeapSize (x)
#define xPortGetFreeHeapSize (x)
#endif

/* Alius define, address now not used, only define*/
#define configINTERRUPT_CONTROLLER_BASE_ADDRESS		(0xf5000000)
#define configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET (0x50000)

#define configUNIQUE_INTERRUPT_PRIORITIES				16
#define configMAX_API_CALL_INTERRUPT_PRIORITY	10

void vSetupTickInterrupt(void);
#define configSETUP_TICK_INTERRUPT() vSetupTickInterrupt()

#define configUSE_TASK_FPU_SUPPORT 2

#define portYIELD_WITHIN_API portYIELD

#define  configCHECK_FOR_STACK_OVERFLOW  2

/* This demo makes use of one or more example stats formatting functions.  These
 * format the raw data provided by the uxTaskGetSystemState() function in to
 * human readable ASCII form.  See the notes in the implementation of vTaskList()
 * within FreeRTOS/Source/tasks.c for limitations. */
#define configUSE_STATS_FORMATTING_FUNCTIONS			1

/* Dimensions a buffer that can be used by the FreeRTOS+CLI command interpreter.
 * See the FreeRTOS+CLI documentation for more information:
 * http://www.FreeRTOS.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/ */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE			2048

#define configUSE_NEWLIB_REENTRANT				1

#endif /* FREERTOS_CONFIG_H */
