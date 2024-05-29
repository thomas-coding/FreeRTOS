/* Kernel includes. */
#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#ifndef mainRUN_FREE_RTOS_ON_CORE
#define mainRUN_FREE_RTOS_ON_CORE 0
#endif

/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainSDK_MUTEX_USE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 3 )
#define mainSDK_SEMAPHORE_USE_TASK_PRIORITY ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY   ( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainQUEUE_SEND_PERIOD_MS            pdMS_TO_TICKS( 200 )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the pdMS_TO_TICKS() macro. */
#define mainSOFTWARE_TIMER_PERIOD_MS        pdMS_TO_TICKS( 1000 )

/* The number of items the queue can hold.  This is 1 as the receive task
has a higher priority than the send task, so will remove items as they are added,
meaning the send task should always find the queue empty. */
#define mainQUEUE_LENGTH                    ( 1 )

/*-----------------------------------------------------------*/

/*
 * Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*
 * The callback function assigned to the example software timer as described at
 * the top of this file.
 */
static void vExampleTimerCallback( TimerHandle_t xTimer );

/*
 * The event semaphore task as described at the top of this file.
 */
static void prvEventSemaphoreTask( void *pvParameters );

/**
 * A task that uses an SDK mutex
 */
static void prvSDKMutexUseTask( void *pvParameters );

/**
 * A task that uses an SDK semaphore
 */
static void prvSDKSemaphoreUseTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by the queue send and queue receive tasks. */
static QueueHandle_t xQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static SemaphoreHandle_t xEventSemaphore = NULL;

/* The counters used by the various examples.  The usage is described in the
 * comments at the top of this file.
 */
static volatile uint32_t ulCountOfTimerCallbackExecutions = 0;
static volatile uint32_t ulCountOfItemsSentOnQueue = 0;
static volatile uint32_t ulCountOfItemsReceivedOnQueue = 0;
static volatile uint32_t ulCountOfReceivedSemaphores = 0;
static volatile uint32_t ulCountOfSDKMutexEnters = 0;
static volatile uint32_t ulCountOfSDKSemaphoreAcquires = 0;

/*-----------------------------------------------------------*/

#include "pico/mutex.h"
#include "pico/sem.h"

auto_init_mutex(xSDKMutex);
static semaphore_t xSDKSemaphore;

static void prvNonRTOSWorker() {
    printf("Core %d: Doing regular SDK stuff\n", get_core_num());
    uint32_t counter;
    while (true) {
        mutex_enter_blocking(&xSDKMutex);
        printf("Core %d: Acquire SDK mutex\n", get_core_num());
        absolute_time_t end_time = make_timeout_time_ms(750);
        while (!time_reached(end_time)) {
            counter++;
            printf("Core %d: Busy work with mutex %d\n", get_core_num(), counter);
            busy_wait_us(137384);
        }
        printf("Core %d: Release SDK mutex\n", get_core_num());
        mutex_exit(&xSDKMutex);
        printf("Core %d: Starting SDK sleep\n", get_core_num());
        sleep_ms(1200);
        printf("Core %d: Finish SDK sleep; release SDK semaphore\n", get_core_num());
        sem_release(&xSDKSemaphore);
    }
}

static void prvLaunchRTOS() {
    printf("Core %d: Launching FreeRTOS scheduler\n", get_core_num());
    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    /* should never reach here */
    panic_unsupported();
}

static void prvCore1Entry() {
#if ( mainRUN_FREE_RTOS_ON_CORE == 1 )
    prvLaunchRTOS();
#else
    prvNonRTOSWorker();
#endif
}

static void user_task( void *pvParameters )
{
	uint32_t task_number = (uint32_t)pvParameters;
	uint32_t count = 0;

	printf("Core %d: \n%s %s\n", get_core_num(), __DATE__, __TIME__);
	printf("Core %d: task %d create succeed\n", get_core_num(), task_number);
	while(1) {
		//printf("Core %d: task[%d] running\n", get_core_num(), task_number);
		//vTaskDelay( pdMS_TO_TICKS(1000 * task_number) );
		count++;
        if (count == 0xffffff) {
            count = 0;
            printf("Core %d: task[%d] running\n", get_core_num(), task_number);
        }
	}
}


int main(void) {
    TimerHandle_t xExampleSoftwareTimer = NULL;

    /* Configure the system ready to run the demo.  The clock configuration
    can be done here if it was not done before main() was called. */
    prvSetupHardware();

    int core = get_core_num();

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
		3,          /* The priority allocated to the task. */
		NULL );     /* A handle is not required, so just pass NULL. */

	xTaskCreate(user_task,  /* The task that implements the command console. */
		"Task3",    /* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
		1000,       /* The size of the stack allocated to the task. */
		(void *)3,  /* The parameter is not used, so NULL is passed. */
		3,          /* The priority allocated to the task. */
		NULL );     /* A handle is not required, so just pass NULL. */

    printf("Core %d: Launching FreeRTOS scheduler\n", get_core_num());

	vTaskStartScheduler();

	for ( ;; )
	;
}
/*-----------------------------------------------------------*/

static void vExampleTimerCallback( TimerHandle_t xTimer )
{
    /* The timer has expired.  Count the number of times this happens.  The
    timer that calls this function is an auto re-load timer, so it will
    execute periodically. */
    ulCountOfTimerCallbackExecutions++;
}
/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
    TickType_t xNextWakeTime;
    const uint32_t ulValueToSend = 100UL;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        /* Place this task in the blocked state until it is time to run again.
        The block time is specified in ticks, the constant used converts ticks
        to ms.  The task will not consume any CPU time while it is in the
        Blocked state. */
        vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_PERIOD_MS );

        /* Send to the queue - causing the queue receive task to unblock and
        increment its counter.  0 is used as the block time so the sending
        operation will not block - it shouldn't need to block as the queue
        should always be empty at this point in the code. */
        ulCountOfItemsSentOnQueue++;
        printf("Core %d - Thread '%s': Queue send %d\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()), ulCountOfItemsSentOnQueue);
        xQueueSend( xQueue, &ulValueToSend, 0 );
    }
}
/*-----------------------------------------------------------*/


static void prvQueueReceiveTask( void *pvParameters )
{
    uint32_t ulReceivedValue;

    for( ;; )
    {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        FreeRTOSConfig.h. */
        xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );
        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, increment the counter. */
        if( ulReceivedValue == 100UL )
        {
            /* Count the number of items that have been received correctly. */
            ulCountOfItemsReceivedOnQueue++;
            printf("Core %d - Thread '%s': Queue receive %d\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()), ulCountOfItemsReceivedOnQueue);
        }
    }
}
/*-----------------------------------------------------------*/

static void prvEventSemaphoreTask( void *pvParameters )
{
    for( ;; )
    {
        /* Block until the semaphore is 'given'.  NOTE:
        A semaphore is used for example purposes.  In a real application it might
        be preferable to use a direct to task notification, which will be faster
        and use less RAM. */
        xSemaphoreTake( xEventSemaphore, portMAX_DELAY );

        /* Count the number of times the semaphore is received. */
        ulCountOfReceivedSemaphores++;
        printf("Core %d - Thread '%s': Semaphore taken %d\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()), ulCountOfReceivedSemaphores);
    }
}
/*-----------------------------------------------------------*/

static void prvSDKMutexUseTask( void *pvParameters )
{
    for( ;; )
    {
        mutex_enter_blocking(&xSDKMutex);
        ulCountOfSDKMutexEnters++;
        printf("Core %d - Thread '%s': SDK Mutex Entered, sleeping for a while %d\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()), ulCountOfSDKMutexEnters);
        vTaskDelay(3000);
        printf("Core %d - Thread '%s': Sleep finished; SDK Mutex releasing %d\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()), ulCountOfSDKMutexEnters);
        mutex_exit(&xSDKMutex);
    }
}
/*-----------------------------------------------------------*/

static void prvSDKSemaphoreUseTask( void *pvParameters )
{
    for( ;; )
    {
        absolute_time_t t = get_absolute_time();
        if (sem_acquire_timeout_us(&xSDKSemaphore, 250500)) {
            ulCountOfSDKSemaphoreAcquires++;
            printf("Core %d - Thread '%s': SDK Sem acquired %d\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()), ulCountOfSDKMutexEnters);
        } else {
            printf("Core %d - Thread '%s': SDK Sem wait timeout (ok) after %d us\n", get_core_num(), pcTaskGetName(xTaskGetCurrentTaskHandle()),
                   (int)absolute_time_diff_us(t, get_absolute_time()));
        }
    }
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{

}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
    panic("malloc failed");
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) xTask;

    /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeStackSpace;

    /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
    xFreeStackSpace = xPortGetFreeHeapSize();

    if( xFreeStackSpace > 100 )
    {
        /* By now, the kernel has allocated everything it is going to, so
        if there is a lot of heap remaining unallocated then
        the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
        reduced accordingly. */
    }
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    /* Want to be able to printf */
    stdio_init_all();
    /* And flash LED */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}