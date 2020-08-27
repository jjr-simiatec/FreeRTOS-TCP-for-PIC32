/*
 * PIC32 Architecture Helpers
 *
 * Copyright (c) 2016-2019 John Robertson
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

#ifndef PIC32_ARCH_H
#define PIC32_ARCH_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SYSKEY_UNLOCK_SEQ0  0xAA996655UL
#define SYSKEY_UNLOCK_SEQ1  0x556699AAUL

#define SYSKEY_UNLOCK() \
    SYSKEY = 0; \
    SYSKEY = SYSKEY_UNLOCK_SEQ0; \
    SYSKEY = SYSKEY_UNLOCK_SEQ1; \

#define SYSKEY_UNLOCK_RTOS() \
    portENTER_CRITICAL(); \
    SYSKEY_UNLOCK(); \
    portEXIT_CRITICAL(); \

#define SYSKEY_LOCK() \
    SYSKEY = 0; \

#define InterlockedCompareExchange(destination, exchange, comparand) __extension__({ \
    volatile BaseType_t *d = destination; \
    BaseType_t e = exchange, c = comparand, val = 0; \
    __asm__ volatile( \
                 ".set noreorder\n" \
                 "1:  ll    $t0, %0\n" \
                 "    bne   $t0, %2, 2f\n" \
                 "    move  %1, $t0\n" \
                 "    move  $t0, %3\n" \
                 "    sc    $t0, %0\n" \
                 "    beq   $t0, $zero, 1b\n" \
                 "    nop\n" \
                 "2:  \n" \
                 ".set reorder" \
                 : "+m"(*d), "=&r"(val) \
                 : "r"(c), "r"(e) \
                 : "t0", "cc"); \
    val; \
})

#ifdef	__cplusplus
}
#endif

#endif // PIC32_ATCH_H
