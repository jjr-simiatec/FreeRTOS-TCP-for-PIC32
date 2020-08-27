/*
 * Simple UART console driver for PIC32
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
#include <queue.h>
#include <task.h>
// C Runtime
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "UART2.h"

#define UART2_RX_QUEUE_SIZE 32U
#define UART2_TX_QUEUE_SIZE 32U

#define CHR_ESCAPE    '\x1B'
#define CHR_RETURN    '\x0D'
#define CHR_BACKSPACE '\x7F'

static QueueHandle_t s_UartRxQueue;
static QueueHandle_t s_UartTxQueue;
static uart_stats_t s_UartStats;

#if defined(__PIC32MZ__)

void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_FAULT_VECTOR) )) Uart2FaultInterruptWrapper(void);
void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_TX_VECTOR) )) Uart2TxInterruptWrapper(void);
void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_RX_VECTOR) )) Uart2RxInterruptWrapper(void);

#define ClearAllInterrupts()    IFS4CLR = _IFS4_U2EIF_MASK | _IFS4_U2RXIF_MASK | _IFS4_U2TXIF_MASK
#define ClearErrorInterrupt()   IFS4CLR = _IFS4_U2EIF_MASK
#define ClearRxInterrupt()      IFS4CLR = _IFS4_U2RXIF_MASK
#define EnableRxInterrupt()     IEC4SET = _IEC4_U2RXIE_MASK
#define DisableRxInterrupt()    IEC4CLR = _IEC4_U2RXIE_MASK
#define ClearTxInterrupt()      IFS4CLR = _IFS4_U2TXIF_MASK
#define EnableTxInterrupt()     IEC4SET = _IEC4_U2TXIE_MASK
#define DisableTxInterrupt()    IEC4CLR = _IEC4_U2TXIE_MASK

#elif defined(__PIC32MX__)

void __attribute__(( interrupt(IPL0AUTO), vector(_UART2_VECTOR) )) Uart2InterruptWrapper(void);

#define ClearAllInterrupts()    IFS1CLR = _IFS1_U2EIF_MASK | _IFS1_U2RXIF_MASK | _IFS1_U2TXIF_MASK
#define ClearErrorInterrupt()   IFS1CLR = _IFS1_U2EIF_MASK
#define ClearRxInterrupt()      IFS1CLR = _IFS1_U2RXIF_MASK
#define EnableRxInterrupt()     IEC1SET = _IEC1_U2RXIE_MASK
#define DisableRxInterrupt()    IEC1CLR = _IEC1_U2RXIE_MASK
#define ClearTxInterrupt()      IFS1CLR = _IFS1_U2TXIF_MASK
#define EnableTxInterrupt()     IEC1SET = _IEC1_U2TXIE_MASK
#define DisableTxInterrupt()    IEC1CLR = _IEC1_U2TXIE_MASK

#else

#error Unsupported processor!

#endif

static uint16_t CalcUartBaudRate(uint32_t fpb, uint32_t baud);

void Uart2Initialise(uint32_t baud)
{
    setbuf(stdin, NULL);
    setbuf(stdout, NULL);

    memset(&s_UartStats, 0, sizeof(s_UartStats));

    s_UartRxQueue = xQueueCreate(UART2_RX_QUEUE_SIZE, sizeof(uint8_t));
    s_UartTxQueue = xQueueCreate(UART2_TX_QUEUE_SIZE, sizeof(uint8_t));

    U2BRG = CalcUartBaudRate(configPERIPHERAL_CLOCK_HZ, baud);

    U2MODEbits.PDSEL = 0;   // 8-bit data, no parity
    U2MODEbits.STSEL = 0;   // 1 stop bit
    U2STAbits.UTXISEL = 0;

#if defined(__PIC32MZ__)

    IPC36bits.U2TXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2RXIP = configKERNEL_INTERRUPT_PRIORITY;
    IPC36bits.U2EIP = configKERNEL_INTERRUPT_PRIORITY;

#elif defined(__PIC32MX__)

    IPC8bits.U2IP = configKERNEL_INTERRUPT_PRIORITY;

#endif

    ClearAllInterrupts();
    EnableRxInterrupt();

    U2STASET = _U2STA_UTXEN_MASK | _U2STA_URXEN_MASK;
}

void Uart2Enable(void)
{
    U2MODESET = _U2MODE_ON_MASK;
}

void Uart2Flush(void)
{
    while( uxQueueMessagesWaiting(s_UartTxQueue) || !U2STAbits.TRMT )
    {
        taskYIELD();
    }
}

void Uart2FlushInput(void)
{
    xQueueReset(s_UartRxQueue);
}

void Uart2FaultInterruptHandler(void)
{
    ClearErrorInterrupt();
}

void Uart2TxInterruptHandler(void)
{
    ClearTxInterrupt();

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;

    uint8_t ch;

    while( !U2STAbits.UTXBF )
    {
        if( !xQueueReceiveFromISR(s_UartTxQueue, &ch, &bHigherPriorityTaskWoken) )
        {
            DisableTxInterrupt();
            break;
        }

        U2TXREG = ch;
    }

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}

void Uart2RxInterruptHandler(void)
{
    ClearRxInterrupt();

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;

    while( U2STAbits.URXDA )
    {
        bool bParityError = (U2STAbits.PERR != 0);
        bool bFramingError = (U2STAbits.FERR != 0);
        uint8_t rxchar = U2RXREG;

        if( bParityError || bFramingError )
        {
            if( bParityError )
                s_UartStats.badParity++;

            if( bFramingError )
                s_UartStats.framingErrors++;

            continue;
        }

        if( !xQueueSendFromISR(s_UartRxQueue, &rxchar, &bHigherPriorityTaskWoken) )
        {
            s_UartStats.queueFull++;
            break;
        }
    }

    if( U2STAbits.OERR )
    {
        U2STACLR = _U2STA_OERR_MASK;
        s_UartStats.fifoOverruns++;
    }

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}

#if defined(__PIC32MX__)

void Uart2InterruptHandler(void)
{
    if( IFS1bits.U2RXIF )
    {
        Uart2RxInterruptHandler();
    }

    if( IFS1bits.U2TXIF )
    {
        Uart2TxInterruptHandler();
    }
}

#endif

void Uart2GetStats(uart_stats_t *pStats)
{
    memcpy(pStats, &s_UartStats, sizeof(s_UartStats));
}

void _mon_putc(char c)
{
    xQueueSend(s_UartTxQueue, &c, portMAX_DELAY);
    EnableTxInterrupt();
}

int _mon_getc(int canblock)
{
    char c;

    if( xQueueReceive(s_UartRxQueue, &c, canblock ? portMAX_DELAY : 0) )
    {
        return c;
    }

    return -1;
}

uint16_t CalcUartBaudRate(uint32_t fpb, uint32_t baud)
{
    return ((fpb / (16.0f * baud)) - 0.5f);
}

/** User input helpers **/

int WaitForAKeyPress(TickType_t maxWait)
{
    uint8_t ch;
    if( !xQueueReceive(s_UartRxQueue, &ch, maxWait) )
    {
        return -1;
    }

    return ch;
}

int InputString(size_t bufSize, char *pBuffer)
{
    size_t nInputChars = 0;

    for( ; ; )
    {
        int ch = WaitForAKeyPress(portMAX_DELAY);

        switch(ch)
        {
        case CHR_ESCAPE:
            return -1;

        case CHR_RETURN:
            pBuffer[nInputChars] = 0;
            return nInputChars;

        case CHR_BACKSPACE:
            if(nInputChars > 0)
            {
                nInputChars--;
                printf("\x1B[D \x1B[D");
            }

            break;

        default:
            if((nInputChars < (bufSize - 1)) && isprint(ch))
            {
                printf("%c", ch);
                pBuffer[nInputChars++] = ch;
            }
        }
    }
}

bool InputHexValue(size_t bufSize, char *pBuffer, uint32_t *pValue)
{
    if( InputString(bufSize, pBuffer) <= 0 )
    {
        return false;
    }

    char *pEndPtr;
    *pValue = strtoul(pBuffer, &pEndPtr, 16);

    return (*pEndPtr == 0);
}

bool InputFloatValue(size_t bufSize, char *pBuffer, float *pValue)
{
    if( InputString(bufSize, pBuffer) <= 0 )
    {
        return false;
    }

    char *pEndPtr;
    *pValue = strtof(pBuffer, &pEndPtr);

    return (*pEndPtr == 0);
}

bool InputIntValue(size_t bufSize, char *pBuffer, int32_t *pValue)
{
    if( InputString(bufSize, pBuffer) <= 0 )
    {
        return false;
    }

    char *pEndPtr;
    *pValue = strtol(pBuffer, &pEndPtr, 10);

    return (*pEndPtr == 0);
}

bool InputIPv4Address(size_t bufSize, char *pBuffer, uint32_t *pAddrN)
{
    int result = InputString(bufSize, pBuffer);

    if((result <= 0) || (result > 15))
    {
        return false;
    }

    uint16_t a, b, c, d;
    result = sscanf(pBuffer, "%hu.%hu.%hu.%hu", &a, &b, &c, &d);

    if((result != 4) || (a > 255) || (b > 255) || (c > 255) || (d > 255))
    {
        return false;
    }

    *pAddrN = a | (b << 8U) | (c << 16UL) | (d << 24UL);

    return true;
}
