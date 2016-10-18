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

#include <xc.h>
#include <sys/asm.h>
#include "ISR_Support.h"

    .extern Uart2FaultInterruptHandler
    .extern Uart2TxInterruptHandler
    .extern Uart2RxInterruptHandler

    .extern xISRStackTop

#if defined(__PIC32MZ__)

    .global Uart2FaultInterruptWrapper
    .global Uart2TxInterruptWrapper
    .global Uart2RxInterruptWrapper

#elif defined(__PIC32MX__)    

    .global Uart2InterruptWrapper

#else
#error Unsupported processor!
#endif


    .set    nomips16
    .set    noreorder
    .set    noat

#if defined(__PIC32MZ__)

    .ent    Uart2FaultInterruptWrapper
Uart2FaultInterruptWrapper:
    portSAVE_CONTEXT
    jal	    Uart2FaultInterruptHandler
    nop
    portRESTORE_CONTEXT

    .end    Uart2FaultInterruptWrapper

    .ent    Uart2TxInterruptWrapper
Uart2TxInterruptWrapper:
    portSAVE_CONTEXT
    jal	    Uart2TxInterruptHandler
    nop
    portRESTORE_CONTEXT

    .end    Uart2TxInterruptWrapper

    .ent    Uart2RxInterruptWrapper
Uart2RxInterruptWrapper:
    portSAVE_CONTEXT
    jal	    Uart2RxInterruptHandler
    nop
    portRESTORE_CONTEXT

    .end    Uart2RxInterruptWrapper

#elif defined(__PIC32MX__)

    .ent    Uart2InterruptWrapper
Uart2InterruptWrapper:
    portSAVE_CONTEXT
    jal	    Uart2InterruptHandler
    nop
    portRESTORE_CONTEXT

    .end    Uart2InterruptWrapper

#endif
