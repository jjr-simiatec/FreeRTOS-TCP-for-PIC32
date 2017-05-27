/*
 * Extended March C- Memory Test
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

/*
 * Detects AF, SAF, TF, CF and SOF faults.
 * {(w0);^(r0,w1,r1);^(r1,w0);!^(r0,w1);!^(r1,w0);(r0)}
 *
 * a0 = Start address of block to test
 * a1 = Length of block to test
 * ra = Program address to go to on success
 * t0 = Address under test
 * t1 = Counter
 * t2, t3, t4 = Temporary (value, tests, bit masks)
 *
 * Corrupts a0, t0-4, flags
 */

#include <xc.h>

    .set    nomips16
    .set    noreorder
    .set    noat

    .extern MemTest_MarchingC

    .text

    .ent    MemTest_MarchingC
    .type   MemTest_MarchingC, @function
MemTest_MarchingC:

/* (w0) */
    beqz    a1, memtest_pass

    move    t0, a0
    move    t1, a1

blank_loop:
    sb	    zero, 0(t0)
    subu    t1, t1, 1
    bnez    t1, blank_loop
    addu    t0, t0, 1

/* ^(r0,w1,r1) */
    move    t0, a0
    move    t1, a1

r0w1r1_innerstart:
    li	    t2, 1

r0w1r1_innerloop:
    lb	    t3, 0(t0)
    and	    t4, t3, t2
    bnez    t4, memtest_failure

    or	    t3, t3, t2
    sb	    t3, 0(t0)

    lb	    t3, 0(t0)
    and	    t4, t3, t2
    beqz    t4, memtest_failure

    sll	    t2, t2, 1
    and	    t2, t2, 0xFF
    bnez    t2, r0w1r1_innerloop
    nop

    subu    t1, t1, 1
    bnez    t1, r0w1r1_innerstart
    addu    t0, t0, 1

/* ^(r1,w0) */
    move    t0, a0
    move    t1, a1

r1w0_innerstart:
    li	    t2, 1

r1w0_innerloop:
    lb	    t3, 0(t0)
    and	    t4, t3, t2
    beqz    t4, memtest_failure

    xor	    t3, t3, t2
    sb	    t3, 0(t0)

    sll	    t2, t2, 1
    and	    t2, t2, 0xFF
    bnez    t2, r1w0_innerloop
    nop

    subu    t1, t1, 1
    bnez    t1, r1w0_innerstart
    addu    t0, t0, 1

/* !^(r0,w1) */
    subu    t0, t0, 1
    move    a0, t0
    move    t1, a1

rev_r0w1_innerstart:
    li	    t2, 0x80

rev_r0w1_innerloop:
    lb	    t3, 0(t0)
    and	    t4, t3, t2
    bnez    t4, memtest_failure

    or	    t3, t3, t2
    sb	    t3, 0(t0)

    srl	    t2, t2, 1
    bnez    t2, rev_r0w1_innerloop
    nop

    subu    t1, t1, 1
    bnez    t1, rev_r0w1_innerstart
    subu    t0, t0, 1

/* !^(r1,w0) */
    move    t0, a0
    move    t1, a1

rev_r1w0_innerstart:
    li	    t2, 0x80

rev_r1w0_innerloop:
    lb	    t3, 0(t0)
    and	    t4, t3, t2
    beqz    t4, memtest_failure

    xor	    t3, t3, t2
    sb	    t3, 0(t0)

    srl	    t2, t2, 1
    bnez    t2, rev_r1w0_innerloop
    nop

    subu    t1, t1, 1
    bnez    t1, rev_r1w0_innerstart
    subu    t0, t0, 1

/* (r0) */
    addu    t0, t0, 1
    move    t1, a1

r0_loop:
    lb	    t2, 0(t0)
    bnez    t2, memtest_failure

    subu    t1, t1, 1
    bnez    t1, r0_loop
    addu    t0, t0, 1

memtest_pass:
    jr	    ra
    nop

memtest_failure:
    wait
    b	    memtest_failure
    nop

    .end    MemTest_MarchingC
