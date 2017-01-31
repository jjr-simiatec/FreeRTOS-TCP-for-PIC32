/*
 * Test Harness Definitions
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

#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include "PIC32Arch.h"

#define TEST_BUFFER_SIZE    255

#define _countof(x)         (sizeof(x) / sizeof(x[0]))

extern uint8_t *g_pTestBuffer;

extern TaskHandle_t g_hTask1;
extern TaskHandle_t g_hTask2;
extern TaskHandle_t g_hPacketTask;

extern ePingReplyStatus_t g_tPingReplyStatus;
extern uint16_t g_nPingReplySequence;

extern void ShowTestTitle(const char *pTitle);
extern void RegisterTestHarnessCLICommands(void);
extern const char *FloatToStr(double d);

#endif // TEST_HARNESS_H
