/*
 * FreeRTOS V202112.00
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


/******************************************************************************
 *
 * This file implements the code that is not demo specific, including the
 * hardware setup and FreeRTOS hook functions.
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdio.h>

/* HalCoGen includes. */
#include "system.h"
#include "het.h"
#include "sys_vim.h"
#include "sci.h"
#include "gio.h"

/* Demo include */
#include "mpu_demo.h"

/* Registers required to configure the RTI. */
#define portRTI_GCTRL_REG          ( *( ( volatile uint32_t * ) 0xFFFFFC00 ) )
#define portRTI_TBCTRL_REG         ( *( ( volatile uint32_t * ) 0xFFFFFC04 ) )
#define portRTI_COMPCTRL_REG       ( *( ( volatile uint32_t * ) 0xFFFFFC0C ) )
#define portRTI_CNT0_FRC0_REG      ( *( ( volatile uint32_t * ) 0xFFFFFC10 ) )
#define portRTI_CNT0_UC0_REG       ( *( ( volatile uint32_t * ) 0xFFFFFC14 ) )
#define portRTI_CNT0_CPUC0_REG     ( *( ( volatile uint32_t * ) 0xFFFFFC18 ) )
#define portRTI_CNT0_COMP0_REG     ( *( ( volatile uint32_t * ) 0xFFFFFC50 ) )
#define portRTI_CNT0_UDCP0_REG     ( *( ( volatile uint32_t * ) 0xFFFFFC54 ) )
#define portRTI_SETINTENA_REG      ( *( ( volatile uint32_t * ) 0xFFFFFC80 ) )
#define portRTI_CLEARINTENA_REG    ( *( ( volatile uint32_t * ) 0xFFFFFC84 ) )
#define portRTI_INTFLAG_REG        ( *( ( volatile uint32_t * ) 0xFFFFFC88 ) )

/* Are these needed? */
typedef void (* ISRFunction_t)( void );
#define portVIM_IRQINDEX     ( *( ( volatile uint32_t * ) 0xFFFFFE00 ) )
#define portVIM_IRQVECREG    ( *( ( volatile ISRFunction_t * ) 0xFFFFFE70 ) )

/** This project provides three demo applications. A simple blinky style demo
 * application, a MPU demo that triggers and clears data aborts, and a more comprehensive
 * test and demo application. The mainDEMO_TYPE setting is used to select between the three.
 *
 * If mainDEMO_TYPE is 1 then the blinky demo will be built.
 * The blinky demo is implemented and described in main_blinky.c.
 *
 * If mainDEMO_TYPE is 2 then the MPU demo will be built.
 * The MPU demo is implemented and described in mpu_demo.c
 *
 * If mainDEMO_TYPE is 3 then the full comprehensive test and
 * demo application will be built. The comprehensive test and demo application is
 * implemented and described in main_full.c. It also runs the MPU demo.
 */
#define BLINKY_DEMO          1
#define MPU_DEMO             2
#define FULL_DEMO            3

#ifndef mainDEMO_TYPE
    #define mainDEMO_TYPE    MPU_DEMO
#endif


/**
 * main_blinky() is used when mainDEMO_TYPE is set to BLINKY_DEMO.
 * vStartMPUDemo() is used when mainDEMO_TYPE is set to MPU_DEMO.
 * main_full() & vStartMPUDemo() is used when mainDEMO_TYPE is set to FULL_DEMO.
 */
extern void main_blinky( void );
extern void main_full( void );
extern void vStartMPUDemo( void );

/**
 * Only the comprehensive demo uses application hook (callback) functions.  See
 * https://www.FreeRTOS.org/a00016.html for more information.
 */
void vFullDemoTickHookFunction( void );
void vFullDemoIdleFunction( void );

/* Configure the hardware to run the demo. */
static void prvSetupHardware( void );

TaskStatus_t pxTaskStatusArray[ 100 ];

/*-----------------------------------------------------------*/

int main( void )
{
    prvSetupHardware();

    /* The mainDEMO_TYPE setting is described at the top of this file. */
    #if ( mainDEMO_TYPE == FULL_DEMO )
        {
            sci_print( "Launching Full Demo\r\n" );
            vStartMPUDemo();
            main_full();
        }
    #elif ( mainDEMO_TYPE == MPU_DEMO )
        {
            sci_print( "Launching MPU Demo\r\n" );
            vStartMPUDemo();
            vTaskStartScheduler();
        }
    #else /* if ( mainDEMO_TYPE == BLINKY_DEMO ) */
        {
            sci_print( "Launching Blinky Demo\r\n" );
            main_blinky();
        }
    #endif /* if ( mainDEMO_TYPE == FULL_DEMO ) */

    /* Should never reach here as the scheduler will be running. */
    return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    systemInit();
    gioInit();
    hetInit();
    sciInit();

    /* Setup gioPORTB for when using the RM46 Launchpad */
    gioPORTB->DIR |= ( 0x01 << 1 ); /*configure GIOB[1] as output */
    gioPORTB->DIR |= ( 0x01 << 2 ); /*configure GIOB[3] as output */

    /* Configure HET as master, pull functionality, and switch on. */
    hetREG1->GCR = 0x01000001;
    hetREG1->PULDIS = 0x00000000;

    /* Configure pins connected to LEDs NHET[0,2,4,5,25,16,17,18,20,27,29,31]
     * as output. */
    hetREG1->DIR = 0xAA178035;
    hetREG1->DOUT = 0x0;

    /* Enable notifications for the SCI register */
    /* Use a BAUD rate of 115200, 1 stopbit, and None Parity */
    sciEnableNotification( scilinREG, SCI_RX_INT );
}
/*-----------------------------------------------------------*/

void vToggleLED( uint32_t ulLED,
                 uint32_t ulGIO )
{
    /* Not a generic function - ok for the demo's use of LEDs zero and one. */
    taskENTER_CRITICAL();
    {
        if( ( hetREG1->DOUT & ulLED ) == 0 )
        {
            hetREG1->DOUT |= ulLED;
            gioPORTB->DOUT |= ulGIO;
        }
        else
        {
            hetREG1->DOUT &= ~ulLED;
            gioPORTB->DOUT &= ~ulGIO;
        }
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vMainSetupTimerInterrupt( void )
{
    /* Disable timer 0. */
    portRTI_GCTRL_REG &= 0xFFFFFFFEUL;

    /* Use the internal counter. */
    portRTI_TBCTRL_REG = 0x00000000U;

    /* COMPSEL0 will use the RTIFRC0 counter. */
    portRTI_COMPCTRL_REG = 0x00000000U;

    /* Initialise the counter and the prescale counter registers. */
    portRTI_CNT0_UC0_REG = 0x00000000U;
    portRTI_CNT0_FRC0_REG = 0x00000000U;

    /* Set Prescalar for RTI clock. */
    portRTI_CNT0_CPUC0_REG = 0x00000001U;
    portRTI_CNT0_COMP0_REG = ( configCPU_CLOCK_HZ / 2 ) / configTICK_RATE_HZ;
    portRTI_CNT0_UDCP0_REG = ( configCPU_CLOCK_HZ / 2 ) / configTICK_RATE_HZ;

    /* Clear interrupts. */
    portRTI_INTFLAG_REG = 0x0007000FU;
    portRTI_CLEARINTENA_REG = 0x00070F0FU;

    /* Enable the compare 0 interrupt. */
    portRTI_SETINTENA_REG = 0x00000001U;
    portRTI_GCTRL_REG |= 0x00000001U;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
     * configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
     * function that will get called if a call to pvPortMalloc() fails.
     * pvPortMalloc() is called internally by the kernel whenever a task, queue,
     * timer or semaphore is created using the dynamic allocation (as opposed to
     * static allocation) option.  It is also called by various parts of the
     * demo application.  If heap_1.c, heap_2.c or heap_4.c is being used, then the
     * size of the heap available to pvPortMalloc() is defined by
     * configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
     * API function can be used to query the size of free heap space that remains
     * (although it does not provide information on how the remaining heap might be
     * fragmented).  See http://www.freertos.org/a00111.html for more
     * information. */
    portDISABLE_INTERRUPTS();
    sci_print( "MallocFailed\r\n" );
    uxTaskGetSystemState( pxTaskStatusArray, sizeof( pxTaskStatusArray ) / sizeof( TaskStatus_t ), NULL );

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     * to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     * task.  It is essential that code added to this hook function never attempts
     * to block in any way (for example, call xQueueReceive() with a block time
     * specified, or call vTaskDelay()).  If application tasks make use of the
     * vTaskDelete() API function to delete themselves then it is also important
     * that vApplicationIdleHook() is permitted to return to its calling function,
     * because it is the responsibility of the idle task to clean up memory
     * allocated by the kernel to any task that has since deleted itself. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask,
                                    char * pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
     * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     * function is called if a stack overflow is detected. */
    portDISABLE_INTERRUPTS();
    sci_print( "StacKOverflow: " );
    sci_print( pcTaskName );
    sci_print( "\r\n" );
    uxTaskGetSystemState( pxTaskStatusArray, sizeof( pxTaskStatusArray ) / sizeof( TaskStatus_t ), NULL );

    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    * added here, but the tick hook is called from an interrupt context, so
    * code must not attempt to block, and only the interrupt safe FreeRTOS API
    * functions can be used (those that end in FromISR()). */

    #if ( mainDEMO_TYPE == FULL_DEMO )
        {
            extern void vFullDemoTickHookFunction( void );

            vFullDemoTickHookFunction();
        }
    #endif /* mainDEMO_TYPE */
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFileName,
                    uint32_t ulLine )
{
    volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

    /* Called if an assertion passed to configASSERT() fails.  See
     * http://www.freertos.org/a00110.html#configASSERT for more information. */
    char errorMessage[ 64 ];

    taskENTER_CRITICAL();
    {
        sprintf( errorMessage, "Assert Called at %s:%ld\r\n", pcFileName, ulLine );
        sci_print( errorMessage );

        uxTaskGetSystemState( pxTaskStatusArray, sizeof( pxTaskStatusArray ) / sizeof( TaskStatus_t ), NULL );

        /* You can step out of this function to debug the assertion by using
         * the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
         * value. */
        while( ulSetToNonZeroInDebuggerToContinue == 0 )
        {
            __asm volatile ( "NOP" );
            __asm volatile ( "NOP" );
        }
    }
    taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vApplicationIRQHandler( void )
{
    /* Not used on the RM46. */
    configASSERT( 0 );
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */

    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
 * function then they must be declared static - otherwise they will be allocated on
 * the stack and so not exists after this function exits. */

    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/
