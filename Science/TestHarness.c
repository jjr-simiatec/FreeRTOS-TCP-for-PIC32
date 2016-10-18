/*
 * Test Harness
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
// TCP/IP Stack
#include <FreeRTOS_IP.h>
// C Runtime
#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>

#include "UART2.h"
#include "LAN8740A.h"
#include "TestHarness.h"
#include "Ethernet.h"

#define TESTS_PER_PAGE      12

static const char pESC_SEQN_CLS[] = "\x1B[0m\x1B[2J\x1B[H";

static const uint8_t nVALID_PHY_REGS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 13, 14, 16, 17, 18, 24, 25, 26, 27, 28, 29, 30, 31};

static const char * const pPHY_REG_NAMES[_countof(nVALID_PHY_REGS)] = {
    "Basic Control",
    "Basic Status",
    "PHY Identifier 1",
    "PHY Identifier 2",
    "Auto Neg Advertisement",
    "Auto Neg Link Partner",
    "Auto Neg Expansion",
    "Auto Neg Next Page Tx",
    "Auto Neg Next Page Rx",
    "MMD Access Control",
    "MMD Access Address/Data",
    "EDPD/Crossover/EEE Config",
    "Mode Control/Status",
    "Special Modes",
    "TDR Patterns/Delay Control",
    "TDR Control/Status",
    "Symbol Error Counter",
    "Special Control/Status",
    "Cable Length",
    "Interrupt Source Flag",
    "Interrupt Mask",
    "PHY Special Control/Status"
};

static const char * const pETH_STATS[] = {
    "Frames transmitted",
    "Frames received",
    "Single collisions",
    "Multiple collisions",
    "Alignment errors",
    "FCS errors",
    "Rx controller overflows",
    "Rx buffer overflows",
    "Tx no DMA descriptors",
    "Link failures"
};

static const char pETH_STATS_FMT[] = "\x1B[%u;30H%u\x1B[K";

typedef struct {
    int menuChar;
    void (*pfnTest)(void);
    const char *pMenuText;
} test_info_t;

static void ViewRTOSRunTimeStats(void);
static void ViewPHYRegisters(void);
static void EthCableDiagnostic(void);
static void ViewEthernetStats(void);
static void PingAddress(void);
static void ResetBoard(void);

static const test_info_t s_TESTS[] = {
    {'1', &ViewRTOSRunTimeStats, "VIEW RTOS RUNTIME STATS"  },
    {'2', &ViewPHYRegisters,     "VIEW PHY REGISTERS"       },
    {'3', &EthCableDiagnostic,   "ETHERNET CABLE DIAGNOSTIC"},
    {'4', &ViewEthernetStats,    "ETHERNET STATISTICS"      },
    {'5', &PingAddress,          "PING ADDRESS"             },
    {'R', &ResetBoard,           "SOFT RESET"               }
};

static size_t s_nTestMenuTopIndx = 0;
uint8_t *g_pTestBuffer;

void vLoggingPrintf(const char *pcFormat, ...)
{
    va_list ap;
    va_start(ap, pcFormat);
    vprintf(pcFormat, ap);
    va_end(ap);
}

// Work around for the buggy printf support for outputting doubles
const char *FloatToStr(double d)
{
    double intPart;
    double fractPart = fabs( modf(d, &intPart) );

    snprintf((char *) g_pTestBuffer, TEST_BUFFER_SIZE, "%d.%u", (int) intPart, (unsigned int) (fractPart * 100.0f));

    return (char *) g_pTestBuffer;
}

void ShowTestTitle(const char *pTitle)
{
    printf(pESC_SEQN_CLS);
    printf("\x1B[4m%s\x1B[0m\r\n\n", pTitle);
}

static void ShowTime(void)
{
    time_t rawTime = time(NULL);
    const struct tm *timeNow = gmtime(&rawTime);

    printf("\x1B[s\x1B[0;38H%02u:%02u:%02u\x1B[u", timeNow->tm_hour, timeNow->tm_min, timeNow->tm_sec);
}

static void ShowMenuScreen(void)
{
    ShowTestTitle("TEST HARNESS MENU");

    printf("\tKey:\tFunction:\r\n");

#ifdef MULTI_PAGE_MENU
    if(s_nTestMenuTopIndx > 0)
        printf("\t(Page Up)\r\n");
#endif // MULTI_PAGE_MENU

    int i;
    for(i = s_nTestMenuTopIndx; i < (s_nTestMenuTopIndx + TESTS_PER_PAGE); i++)
    {
        if(i >= _countof(s_TESTS))
        {
            break;
        }

        printf("\t\'%c\'\t%s \r\n", s_TESTS[i].menuChar, s_TESTS[i].pMenuText);
    }

#ifdef MULTI_PAGE_MENU
    if(s_nTestMenuTopIndx < (_countof(s_TESTS) - TESTS_PER_PAGE))
        printf("\t(Page Down)\r\n");
#endif // MULTI_PAGE_MENU
}

portTASK_FUNCTION(Task1, pParams)
{
    portTASK_USES_FLOATING_POINT();

    g_pTestBuffer = pvPortMalloc(TEST_BUFFER_SIZE + 1);

    Uart2Enable();

    ShowMenuScreen();

    for( ; ; )
    {
        int m = toupper( WaitForAKeyPress( pdMS_TO_TICKS(1000) ) );

        if(m < 0)
        {
            ShowTime();
            continue;
        }

        int c;
        for(c = 0; c < _countof(s_TESTS); c++)
        {
            if(s_TESTS[c].menuChar != m)
            {
                continue;
            }

            (*(s_TESTS[c].pfnTest))();

            printf("\r\nPress a key to continue...\x1B[K");
            WaitForAKeyPress(portMAX_DELAY);

            break;
        }

        ShowMenuScreen();
    }
}

void ViewRTOSRunTimeStats(void)
{
    ShowTestTitle("RTOS RUNTIME STATS");

    printf("\x1B[4mTask List\x1B[0m\r\n");
    vTaskList((char *) g_pTestBuffer);
    printf((const char *) g_pTestBuffer);

    printf("\r\n\x1B[4mCPU Usage\x1B[0m\r\n");
    vTaskGetRunTimeStats((char *) g_pTestBuffer);
    printf((const char *) g_pTestBuffer);
}

void ViewPHYRegisters(void)
{
    bool bExit = false, bRegenDisplay = true;
    int c;

    do
    {
        if( bRegenDisplay )
        {
            ShowTestTitle("PHY REGISTERS");

            for(c = 0; c < _countof(pPHY_REG_NAMES); c++)
            {
                printf("\x1B[%d;%dH%s:", 3 + (c / 2), c & 1 ? 40 : 0, pPHY_REG_NAMES[c]);
            }

            bRegenDisplay = false;
       }

        for(c = 0; c < _countof(nVALID_PHY_REGS); c++)
        {
            printf("\x1B[%d;%dH0x%04X", 3 + (c / 2), c & 1 ? 68 : 30, PHYRead(nVALID_PHY_REGS[c]));
        }

        c = WaitForAKeyPress(0);

        if(c >= 0)
        {
            bRegenDisplay = true;

            switch( toupper(c) )
            {
            case 'R':
#if defined(__PIC32MZ__)
                // Hardware reset the PHY
                LAN8740_ASSERT_HW_RESET();
                printf("\r\nReset asserted, press a key to continue.");
                WaitForAKeyPress(portMAX_DELAY);
                LAN8740_CLEAR_HW_RESET();
#endif
                break;

            case 'W':
                // Write to a PHY register
                {
                    int addr; uint32_t value;

                    printf("\r\nRegister? ");
                    if( !InputIntValue(3, (char *) g_pTestBuffer, &addr) )
                    {
                        break;
                    }

                    printf("\r\nValue? ");
                    if( !InputHexValue(5, (char *) g_pTestBuffer, &value) )
                    {
                        break;
                    }

                    PHYWrite(addr, value);
                }

                break;

            case 'M':
                // Read MMD register
                {
                    int devad, index;

                    printf("\r\nMMD device address? ");
                    if( !InputIntValue(3, (char *) g_pTestBuffer, &devad) )
                    {
                        break;
                    }

                    printf("\r\nMMD index? ");
                    if( !InputIntValue(6, (char *) g_pTestBuffer, &index) )
                    {
                        break;
                    }

                    PHY_MMDRead(devad, index);
                }
                break;

#if 0
            case 'N':
                // Write MMD register
                c = PHY_MMDRead(LAN8740_MMD_DEVAD_PCS, 5);
                printf("\r\nValue is %04X", c);

                _mon_getc(1);

                bRegenDisplay = true;

                break;
#endif

            case 'X':
                bExit = true;
                continue;
            }
        }
        else
        {
            vTaskDelay( pdMS_TO_TICKS(250) );
        }
    }
    while( !bExit );
}

static const char * const pETH_CABLE_STATES[] = {
    "Good", "Failed", "Shorted", "Open"
};

void EthCableDiagnostic(void)
{
    ShowTestTitle("ETHERNET CABLE DIAGNOSTIC");

    printf("Cable type: (0-Unknown, 1-CAT5, 2-CAT5e, 3-CAT6)? ");

    int cableType;

    if( !InputIntValue(2, (char *) g_pTestBuffer, &cableType) )
    {
        return;
    }

    printf("\r\nRunning diagnostic...");

    float fLenEstimate;
    phy_tdr_state_t state = PHYCableDiagnostic(cableType, &fLenEstimate);

    if(state >= PHY_TDR_STATE_GOOD)
        printf("\r\nState: %s, Length: %sm", pETH_CABLE_STATES[state], FloatToStr(fLenEstimate));
    else
        printf("\r\nTest not completed, result code is %d.", state);
}

void ViewEthernetStats(void)
{
    ShowTestTitle("ETHERNET STATISTICS");

    int c;
    for(c = 0; c < _countof(pETH_STATS); c++)
    {
        printf("%s\r\n", pETH_STATS[c]);
    }

    eth_stats_t tStats;
    bool bExit = false;

    do
    {
        EthernetGetStats(&tStats);

        printf(pETH_STATS_FMT, 3, tStats.framesTransmitted);
        printf(pETH_STATS_FMT, 4, tStats.framesReceived);
        printf(pETH_STATS_FMT, 5, tStats.singleCollisions);
        printf(pETH_STATS_FMT, 6, tStats.multipleCollisions);
        printf(pETH_STATS_FMT, 7, tStats.alignmentErrors);
        printf(pETH_STATS_FMT, 8, tStats.fcsErrors);
        printf(pETH_STATS_FMT, 9, tStats.receiveOverflows);
        printf(pETH_STATS_FMT, 10, tStats.rxNoBuffers);
        printf(pETH_STATS_FMT, 11, tStats.txNoDMADescriptors);
        printf(pETH_STATS_FMT, 12, tStats.linkFailures);

        c = WaitForAKeyPress(0);

        if(c >= 0)
        {
            switch( toupper(c) )
            {
            case 'X':
                bExit = true;
                break;

            case 'R':
                EthernetResetStats();
                break;
            }
        }
        else
        {
            vTaskDelay( pdMS_TO_TICKS(200) );
        }
    }
    while( !bExit );
}

void PingAddress(void)
{
    ShowTestTitle("PING ADDRESS");

    printf("Enter address: ");

    uint32_t addr;
    if( !InputIPv4Address(16, (char *) g_pTestBuffer, &addr) )
    {
        return;
    }

    xTaskNotifyStateClear(NULL);

    BaseType_t result = FreeRTOS_SendPingRequest(addr, 16, pdMS_TO_TICKS(1000));

    if(result == pdFAIL)
    {
        printf("\r\nPing send request failed.");
        return;
    }

    printf("\r\nPing sent with sequence no. %d, waiting for reply...", result);

    if( ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 )
    {
        printf("\r\nNo response.");
        return;
    }

    switch(g_tPingReplyStatus)
    {
    case eSuccess:
        if(result == g_nPingReplySequence)
            printf("\r\nGood response received.");
        else
            printf("\r\nResponse received but sequence no. doesn't match.");

        break;

    case eInvalidChecksum:
        printf("\r\nReply has a bad checksum.");
        break;

    case eInvalidData:
        printf("\r\nReceived data does not match the request.");
        break;

    default:
        printf("\r\nDon't know what happened...");
    }
}

void ResetBoard(void)
{
    // Works as a reset in non-debug builds
    __builtin_software_breakpoint();
}
