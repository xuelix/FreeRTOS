/*
 * FreeRTOS V202212.00
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

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "sci.h"

/*
 * Utility function.
 */
extern void vToggleLED( uint32_t ulLED,
                        uint32_t ulGIO );


/**
 * @brief Size of the shared memory region.
 */
#define SHARED_MEMORY_SIZE    32

/**
 * @brief Memory region shared between two tasks.
 */
    static volatile uint8_t ucSharedMemory[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory1[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory2[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory3[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory4[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
#if ( configTOTAL_MPU_REGIONS == 16 )
    static volatile uint8_t ucSharedMemory5[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory6[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory7[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
    static volatile uint8_t ucSharedMemory8[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) );
#endif /* configTOTAL_MPU_REGIONS == 16 */

/**
 * @brief Memory region used to track Memory Fault intentionally caused by the
 * RO Access task.
 *
 * RO Access task sets ucROTaskFaultTracker[ 0 ] to 1 before accessing illegal
 * memory. Illegal memory access causes Memory Fault and the fault handler
 * checks ucROTaskFaultTracker[ 0 ] to see if this is an expected fault. We
 * recover gracefully from an expected fault by jumping to the next instruction.
 *
 * @note We are declaring a region of 32 bytes even though we need only one. The
 * reason is that the size of an MPU region must be aligned to the size of the region.
 * The minimal size of an MPU region on a CORTEX R series boards is 32 bytes.
 */
static volatile uint8_t ucROTaskFaultTracker[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( 32 ) ) ) = { 0 };
/*-----------------------------------------------------------*/

/**
 * @brief Implements the task which has Read Only access to the memory region
 * ucSharedMemory.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvROAccessTask( void * pvParameters );

/**
 * @brief Implements the task which has Read Write access to the memory region
 * ucSharedMemory.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvRWAccessTask( void * pvParameters );

/*-----------------------------------------------------------*/

static void prvROAccessTask( void * pvParameters )
{
    volatile uint8_t ucVal;

    /* Unused parameters. */
    ( void ) pvParameters;

    for( ; ; )
    {
        /* This task performs the following sequence for all the shared memory
         * regions:
         *
         * 1. Perform a read access to the shared memory. Since this task has
         *    RO access to the shared memory, the read operation is successful.
         *
         * 2. Set ucROTaskFaultTracker[ 0 ] to 1 before performing a write to
         *    the shared memory. Since this task has Read Only access to the
         *    shared memory, the write operation would result in a Memory Fault.
         *    Setting ucROTaskFaultTracker[ 0 ] to 1 tells the Memory Fault
         *    Handler that this is an expected fault. The handler recovers from
         *    the expected fault gracefully by jumping to the next instruction.
         *
         * 3. Perfrom a write to the shared memory resulting in a memory fault.
         *
         * 4. Ensure that the write access did generate MemFault and the fault
         *    handler did clear the ucROTaskFaultTracker[ 0 ].
         */
        /* Perform the above mentioned sequence on ucSharedMemory. */
        ucVal = ucSharedMemory[ 0 ];
        /* Silent compiler warnings about unused variables. */
        ( void ) ucVal;
        ucROTaskFaultTracker[ 0 ] = 1U;
        ucSharedMemory[ 0 ] = 0U;
        ucVal = ucROTaskFaultTracker[ 0 ];
        configASSERT( ucVal == 0U );

        /* Perform the above mentioned sequence on ucSharedMemory1. */
        ucVal = ucSharedMemory1[ 0 ];
        /* Silent compiler warnings about unused variables. */
        ( void ) ucVal;
        ucROTaskFaultTracker[ 0 ] = 1U;
        ucSharedMemory1[ 0 ] = 0U;
        ucVal = ucROTaskFaultTracker[ 0 ];
        configASSERT( ucVal == 0U );

        /* Perform the above mentioned sequence on ucSharedMemory2. */
        ucVal = ucSharedMemory2[ 0 ];
        /* Silent compiler warnings about unused variables. */
        ( void ) ucVal;
        ucROTaskFaultTracker[ 0 ] = 1U;
        ucSharedMemory2[ 0 ] = 0U;
        ucVal = ucROTaskFaultTracker[ 0 ];
        configASSERT( ucVal == 0U );

        /* Perform the above mentioned sequence on ucSharedMemory3. */
        ucVal = ucSharedMemory3[ 0 ];
        /* Silent compiler warnings about unused variables. */
        ( void ) ucVal;
        ucROTaskFaultTracker[ 0 ] = 1U;
        ucSharedMemory3[ 0 ] = 0U;
        ucVal = ucROTaskFaultTracker[ 0 ];
        configASSERT( ucVal == 0U );

        /* Perform the above mentioned sequence on ucSharedMemory4. */
        ucVal = ucSharedMemory4[ 0 ];
        /* Silent compiler warnings about unused variables. */
        ( void ) ucVal;
        ucROTaskFaultTracker[ 0 ] = 1U;
        ucSharedMemory4[ 0 ] = 0U;
        ucVal = ucROTaskFaultTracker[ 0 ];
        configASSERT( ucVal == 0U );

        #if ( configTOTAL_MPU_REGIONS == 16 )
            {
            /* Perform the above mentioned sequence on ucSharedMemory5. */
            ucVal = ucSharedMemory5[ 0 ];
            /* Silent compiler warnings about unused variables. */
            ( void ) ucVal;
            ucROTaskFaultTracker[ 0 ] = 1U;
            ucSharedMemory5[ 0 ] = 0U;
            ucVal = ucROTaskFaultTracker[ 0 ];
            configASSERT( ucVal == 0U );

            /* Perform the above mentioned sequence on ucSharedMemory6. */
            ucVal = ucSharedMemory6[ 0 ];
            /* Silent compiler warnings about unused variables. */
            ( void ) ucVal;
            ucROTaskFaultTracker[ 0 ] = 1U;
            ucSharedMemory6[ 0 ] = 0U;
            ucVal = ucROTaskFaultTracker[ 0 ];
            configASSERT( ucVal == 0U );

            /* Perform the above mentioned sequence on ucSharedMemory7. */
            ucVal = ucSharedMemory7[ 0 ];
            /* Silent compiler warnings about unused variables. */
            ( void ) ucVal;
            ucROTaskFaultTracker[ 0 ] = 1U;
            ucSharedMemory7[ 0 ] = 0U;
            ucVal = ucROTaskFaultTracker[ 0 ];
            configASSERT( ucVal == 0U );

            /* Perform the above mentioned sequence on ucSharedMemory8. */
            ucVal = ucSharedMemory8[ 0 ];
            /* Silent compiler warnings about unused variables. */
            ( void ) ucVal;
            ucROTaskFaultTracker[ 0 ] = 1U;
            ucSharedMemory8[ 0 ] = 0U;
            ucVal = ucROTaskFaultTracker[ 0 ];
            configASSERT( ucVal == 0U );
        }
        #endif /* configTOTAL_MPU_REGIONS == 16 */

        /* Wait for a second. */
        portDISABLE_INTERRUPTS();
        sci_print( "Read only task did a loop\n\r" );
        portENABLE_INTERRUPTS();
        vTaskDelay( pdMS_TO_TICKS( 4400 ) );
    }
}
/*-----------------------------------------------------------*/

static void prvRWAccessTask( void * pvParameters )
{
    /* Unused parameters. */
    ( void ) pvParameters;
    volatile uint8_t ucVal = 0U;
    ( void ) ucVal;
    for( ; ; )
    {
        /* This task has RW access to ucSharedMemory and therefore can write to
         * it. */
        ucSharedMemory[ 0 ] = 0U;
        ucSharedMemory1[ 0 ] = 0U;
        ucSharedMemory2[ 0 ] = 0U;
        ucSharedMemory3[ 0 ] = 0U;
        ucSharedMemory4[ 0 ] = 0U;
        #if ( configTOTAL_MPU_REGIONS == 16 )
        {            
            ucSharedMemory5[ 0 ] = 0U;
            ucSharedMemory6[ 0 ] = 0U;
            ucSharedMemory7[ 0 ] = 0U;
            ucSharedMemory8[ 0 ] = 0U;
        }
        #endif /* configTOTAL_MPU_REGIONS == 16 */

        /* This task has Privileged Read/Write access to ucROTaskFaultTracker.
         * Meaning that by raising the privilege it is possible to write to it.
         * Then after reseting the privilege a read will trigger a data abort.
         */
        portRAISE_PRIVILEGE();
        ucROTaskFaultTracker[ 1 ] = 1U;
        portRESET_PRIVILEGE();

        ucVal = ucROTaskFaultTracker[ 1 ];
        /* Wait for a second. */
        portDISABLE_INTERRUPTS();
        sci_print( "Read/Write task did a loop\n\r" );
        portENABLE_INTERRUPTS();
        vTaskDelay( pdMS_TO_TICKS( 9500 ) );
    }
}
/*-----------------------------------------------------------*/

void vStartMPUDemo( void )
{
    /* These tasks will use over 288 bytes as of time of writing.
     * Minimal Cortex R MPU region sizes are 32, 64, 128, 256, and 512 bytes. Regions must align to their size
     * Due to this limitation these regions declare 512, or 0x200, bytes and align to that size. */
    static StackType_t xROAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( 0x200 ) ) );
    static StackType_t xRWAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( 0x200 ) ) );
    TaskParameters_t xROAccessTaskParameters =
    {
        .pvTaskCode     = prvROAccessTask,
        .pcName         = "ROAccess",
        .usStackDepth   = configMINIMAL_STACK_SIZE,
        .pvParameters   = NULL,
        .uxPriority     = tskIDLE_PRIORITY,
        .puxStackBuffer = xROAccessTaskStack,
        .xRegions       =
        {
            { ucSharedMemory,       32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  },     /* First Configurable Region 5 */
            { ucSharedMemory1,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  },     /* Region 6 */
            { ucSharedMemory2,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  },     /* Region 7 */
            { ucSharedMemory3,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  },     /* Region 8 */
            { ucSharedMemory4,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  },     /* Region 9 */
            #if ( configTOTAL_MPU_REGIONS == 16 )
                { ucSharedMemory5,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  }, /* Region 10 */
                { ucSharedMemory6,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  }, /* Region 11 */
                { ucSharedMemory7,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  }, /* Region 12 */
                { ucSharedMemory8,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_ONLY | portMPU_NORMAL_OIWTNOWA_SHARED  }, /* Region 13 */
            #endif /* configTOTAL_MPU_REGIONS == 16 */
            { ucROTaskFaultTracker, 32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED },     /* Last Configurable Region */
        }
    };
    TaskParameters_t xRWAccessTaskParameters =
    {
        .pvTaskCode     = prvRWAccessTask,
        .pcName         = "RWAccess",
        .usStackDepth   = configMINIMAL_STACK_SIZE,
        .pvParameters   = NULL,
        .uxPriority     = tskIDLE_PRIORITY,
        .puxStackBuffer = xRWAccessTaskStack,
        .xRegions       =
        {
            { ucSharedMemory,       32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            },     /* First Configurable Region 5 */
            { ucSharedMemory1,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            },     /* Region 6 */
            { ucSharedMemory2,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            },     /* Region 7 */
            { ucSharedMemory3,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            },     /* Region 8 */
            { ucSharedMemory4,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            },     /* Region 9 */
            #if ( configTOTAL_MPU_REGIONS == 16 )
                { ucSharedMemory5,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            }, /* Region 10 */
                { ucSharedMemory6,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            }, /* Region 11 */
                { ucSharedMemory7,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            }, /* Region 12 */
                { ucSharedMemory8,      32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED            }, /* Region 13 */
            #endif /* configTOTAL_MPU_REGIONS == 16 */
            { ucROTaskFaultTracker, 32, portMPU_REGION_EXECUTE_NEVER | portMPU_REGION_PRIVILEGED_READ_WRITE | portMPU_NORMAL_OIWTNOWA_SHARED },     /* Last Configurable Region */
        }
    };

    /* Create an unprivileged task with RO access to ucSharedMemory. */
    xTaskCreateRestricted( &( xROAccessTaskParameters ), NULL );

    /* Create an unprivileged task with RW access to ucSharedMemory. */
    xTaskCreateRestricted( &( xRWAccessTaskParameters ), NULL );
}
/*-----------------------------------------------------------*/

portDONT_DISCARD void vHandleMemoryFault( uint32_t * pulFaultStackAddress )
{
    volatile uint32_t ulPC;
    volatile uint32_t ulOffendingInstruction;

    /* Is this an expected fault? */
    if( ( ucROTaskFaultTracker[ 0 ] == 1U ) || ( ucROTaskFaultTracker[ 1 ] == 1U ) )
    {
        /* Read program counter. */
        ulPC = pulFaultStackAddress[ 13 ];

        /* Read the offending instruction. */
        ulOffendingInstruction = *( uint32_t * ) ulPC;

        /* From ARM docs:
         * Bits [31:28] are the conditional field
         * Bits [27:24] are the operation code
         * If bits [31:28] are 0b1111, the instruction can only be executed unconditionally
         * If bits [31:28] are not 0b1111, the op code determines what the instruction is doing
         * If bits [27:24] are 0b01x0 it is a load/store word
         * If bits [27:24] are 0b0111 it is a media instruction
         */

        /* Extract bits[31:25] of the offending instruction. */
        ulOffendingInstruction = ulOffendingInstruction & 0xFF000000;
        ulOffendingInstruction = ( ulOffendingInstruction >> 24 );

        /* Check if we were called by a load/store word instruction */
        if( ( ulOffendingInstruction == 0x00E4 ) ||
            ( ulOffendingInstruction == 0x00E5 ) ||
            ( ulOffendingInstruction == 0x00E6 ) )
        {
            /* Increment the program counter to move to the next instruction */
            ulPC += 0x4;
        }
        else
        {
            sci_print( "Unexpected Instruction caused an MPU fault\n\r" );
            configASSERT( 0 );
        }

        /* Save the new program counter on the stack. */
        pulFaultStackAddress[ 13 ] = ulPC;

        /* Mark the fault as handled. */
        if( ucROTaskFaultTracker[ 0 ] == 1U )
        {
            ucROTaskFaultTracker[ 0 ] = 0U;
            sci_print( "Cleared an MPU Read Only Task Fault\n\r" );
        }
        else if( ucROTaskFaultTracker[ 1 ] == 1U )
        {
            sci_print( "Cleared an MPU Write Only Task Fault\n\r" );
            ucROTaskFaultTracker[ 1 ] = 0U;
        }
        else
        {
            sci_print( "TaskFaultTracker was high at first, now low. Unexpected MPU fault\n\r" );
            /* Sit in a loop forever */
            configASSERT( 0 );
        }
    }
    else
    {
        sci_print( "Neither TaskFaultTracker is high, Unexpected MPU fault\n\r" );

        /* This is an unexpected fault - loop forever. */
        configASSERT( 0 );
    }
}

/*-----------------------------------------------------------*/
