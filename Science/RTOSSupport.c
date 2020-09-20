/*
 * RTOS/C Runtime Support
 *
 * Copyright (c) 2016 John Robertson
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// RTOS
#include <FreeRTOS.h>
#include <task.h>
// C Runtime
#include <sys/time.h>
#include <stdio.h>
#include <errno.h>

#define RUNTIME_COUNTER_HZ  10000U
#define TOD_RESOLUTION      100U

volatile uint32_t g_excepCause;
volatile uint32_t g_excepAddr;

static volatile unsigned long s_nRunTimeCounter = 0;

static volatile struct timeval s_tod = {0};

void __attribute__((interrupt(IPL0AUTO), vector(_TIMER_2_VECTOR))) vT2InterruptWrapper(void);

void vT2InterruptHandler(void)
{
    s_nRunTimeCounter++;

    s_tod.tv_usec += TOD_RESOLUTION;

    if(s_tod.tv_usec >= 1000000L)
    {
        s_tod.tv_usec = 0;
        s_tod.tv_sec++;
    }

    IFS0CLR = _IFS0_T2IF_MASK;
}

unsigned long ulGetRunTimeCounterValue(void)
{
    return s_nRunTimeCounter;
}

void vConfigureTimerForRunTimeStats(void)
{
    s_nRunTimeCounter = 0;

    T2CON = 0;
    TMR2 = 0;

    PR2 = (configPERIPHERAL_CLOCK_HZ / RUNTIME_COUNTER_HZ) - 1;

    // Setup timer 2 interrupt priority to be above the kernel priority so
    // the timer jitter is not effected by the kernel activity.
    IPC2bits.T2IP = (configMAX_SYSCALL_INTERRUPT_PRIORITY + 1);

    // Clear the interrupt as a starting condition.
    IFS0bits.T2IF = 0;

    // Enable the interrupt.
    IEC0bits.T2IE = 1;

    // Start the timer.
    T2CONbits.TON = 1;
}

time_t time(time_t *tod)
{
    uint32_t timeNow = s_tod.tv_sec;

    if( tod )
        *tod = timeNow;

    return timeNow;
}

int gettimeofday(struct timeval *tv, void *tz)
{
    if( tv )
        *tv = s_tod;

    return 0;
}

int settimeofday(const struct timeval *tv, void *tz)
{
    if( tv )
    {
        if((tv->tv_usec % TOD_RESOLUTION) != 0)
        {
            errno = EINVAL;
            return -1;
        }

        s_tod = *tv;
    }

    return 0;
}

#ifdef __MPLAB_DEBUGGER_SIMULATOR

#define portTIMER_PRESCALE	8
#define portPRESCALE_BITS	1

#define portCOMPARE_MATCH   (((configPERIPHERAL_CLOCK_HZ / portTIMER_PRESCALE) / configTICK_RATE_HZ) - 1)

void vApplicationSetupTickTimerInterrupt(void)
{
	T1CON = 0x0000;
	T1CONbits.TCKPS = portPRESCALE_BITS;
	PR1 = portCOMPARE_MATCH;
	IPC1bits.T1IP = configKERNEL_INTERRUPT_PRIORITY;

	// Clear the interrupt as a starting condition.
	IFS0bits.T1IF = 0;

	// Enable the interrupt.
	IEC0bits.T1IE = 1;

	// Start the timer.
	T1CONbits.TON = 1;
}

#endif

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    /* Run time task stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
    called if a task stack overflow is detected.  Note the system/interrupt
    stack is not checked. */
    taskDISABLE_INTERRUPTS();

#ifdef __DEBUG
    __builtin_software_breakpoint();
#endif // __DEBUG

    for( ; ; )
    {
        _wait();
    }
}

void vAssertCalled(const char *pcFile, unsigned long ulLine)
{
#ifdef __DEBUG
    __builtin_software_breakpoint();
#endif // __DEBUG

    printf("** Assertion failed: %s:%lu **\r\n", pcFile, ulLine);
}

void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
#ifdef __DEBUG
    __builtin_software_breakpoint();
#endif // __DEBUG

    taskDISABLE_INTERRUPTS();

    for( ; ; )
    {
        _wait();
    }
}

void vApplicationIdleHook(void)
{
    _wait();
}

void _general_exception_handler(void)
{
    g_excepCause = _CP0_GET_CAUSE();
    g_excepAddr = _CP0_GET_EPC();

    // Will be a _EXCCODE_* values as per cp0defs.h
    uint8_t _excep_code __attribute__((unused)) = (g_excepCause & _CP0_CAUSE_EXCCODE_MASK) >> _CP0_CAUSE_EXCCODE_POSITION;

    // Examine _excep_code to identify the type of exception.#
    // Examine g_excepAddr to find the address that the exception occurred
#ifdef __DEBUG
    __builtin_software_breakpoint();
#endif // __DEBUG

    for( ; ; )
    {
        _wait();
    }
}
