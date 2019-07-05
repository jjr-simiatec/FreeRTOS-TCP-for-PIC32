/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */


/*
 * Implementation of pvPortMalloc() and vPortFree() that relies on the
 * compilers own malloc() and free() implementations.
 *
 * This file can only be used if the linker is configured to to generate
 * a heap memory area.
 *
 * See heap_1.c, heap_2.c and heap_4.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */

#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
	#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/*-----------------------------------------------------------*/

#define HEAP_CHECK_OVERHEAD     0x06

#define ALLOC_START_MARKER1     0xFE
#define ALLOC_START_MARKER2     0xCA
#define ALLOC_END_MARKER1       0x0B
#define ALLOC_END_MARKER2       0xB1

#define HEAP_NEW_BLOCK          0xCD
#define HEAP_FREED_BLOCK        0xDD

void *pvPortMalloc( size_t xWantedSize )
{
    configASSERT(xWantedSize <= 0xFFFF);
    
    uint8_t *pvReturn;

	vTaskSuspendAll();
	{
		pvReturn = malloc( xWantedSize + HEAP_CHECK_OVERHEAD );
		traceMALLOC( pvReturn, xWantedSize );
	}
	( void ) xTaskResumeAll();

    if(pvReturn == NULL)
    {
        #if( configUSE_MALLOC_FAILED_HOOK == 1 )
        {
            extern void vApplicationMallocFailedHook( void );
            vApplicationMallocFailedHook();
        }
        #endif

        return NULL;
    }

    *pvReturn++ = (xWantedSize & 0xFF);
    *pvReturn++ = (xWantedSize >> 8);
    *pvReturn++ = ALLOC_START_MARKER1;
    *pvReturn++ = ALLOC_START_MARKER2;
        
    memset(pvReturn, HEAP_NEW_BLOCK, xWantedSize);
    
    *(pvReturn + xWantedSize)     = ALLOC_END_MARKER1;
    *(pvReturn + xWantedSize + 1) = ALLOC_END_MARKER2;
    
	return pvReturn;
}
/*-----------------------------------------------------------*/

void vPortFree( void *pv )
{
    if( !pv )
    {
        return;
    }
    
    uint8_t *p = (uint8_t *) pv - 4;
    
    // Underflow?
    configASSERT((p[2] == ALLOC_START_MARKER1) && (p[3] == ALLOC_START_MARKER2));
    
    uint16_t blockSize = p[0] | (p[1] << 8);
    
    // Overflow?
    configASSERT((p[blockSize + 4] == ALLOC_END_MARKER1) && (p[blockSize + 5] == ALLOC_END_MARKER2));
    
    memset(pv, HEAP_FREED_BLOCK, blockSize);
        
    vTaskSuspendAll();
    {
        free( p );
        traceFREE( p, 0 );
    }
    ( void ) xTaskResumeAll();
}
