/*
 * Packet Rx/Tx Performance Test
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

    .extern vT3InterruptHandler
    .extern xISRStackTop
    
    .global vT3InterruptWrapper

    .set    nomips16
    .set    noreorder
    .set    noat
    
    .ent    vT3InterruptWrapper
vT3InterruptWrapper:
    portSAVE_CONTEXT
    jal	    vT3InterruptHandler
    nop
    portRESTORE_CONTEXT

    .end    vT3InterruptWrapper
