/*
 * RAM Test Hook
 *
 * Copyright (c) 2017 John Robertson
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
#include <cp0defs.h>

#if defined(__DEBUG)

#define DATA_MEM_BASE 0
#define DATA_MEM_LENGTH 0

#else

#if defined(__PIC32MZ__)

// L1 Cache won't be enabled yet
#define DATA_MEM_BASE __KSEG0_DATA_MEM_BASE
#define DATA_MEM_LENGTH __KSEG0_DATA_MEM_LENGTH

#elif defined(__PIC32MX__)

#define DATA_MEM_BASE __KSEG1_DATA_MEM_BASE
#define DATA_MEM_LENGTH __KSEG1_DATA_MEM_LENGTH

#else

#error Unsupported processor!

#endif

#endif

    .set    nomips16
    .set    noreorder
    .set    noat

    .section .reset.startup, code, keep

    .extern _on_reset
    .extern MemTest_MarchingC

    .ent    _on_reset
    .type   _on_reset, @function
_on_reset:
    la	    a0, DATA_MEM_BASE
    li	    a1, DATA_MEM_LENGTH
    b	    MemTest_MarchingC
    nop

    .end _on_reset
