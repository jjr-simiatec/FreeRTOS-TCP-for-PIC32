/*
 * Startup and Hardware Configuration
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
#include <queue.h>
#include <semphr.h>
// C Runtime
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
// TCP/IP Stack
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

#include "UART2.h"
#include "LAN8740A.h"
#include "TestHarness.h"
#include "PIC32Arch.h"

#if defined(__PIC32MX__)

#define PIC32MX_MAX_FLASH_SPEED   30000000UL

#define CHECON_PREFEN_CACHEABLE     1
#define CHECON_PREFEN_ALL_REGIONS   3
#define CP0_CONFIG_K0_UNCACHED      2
#define CP0_CONFIG_K0_CACHEABLE     3

#endif

#ifndef __MPLAB_DEBUGGER_SIMULATOR
#define UART2_BAUD_RATE     115200UL
#else
#define UART2_BAUD_RATE     460800UL
#endif

#if defined(__32MZ2048EFM144__)
static const uint8_t pIP_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {10, 10, 10, 10};
#else
static const uint8_t pIP_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {10, 10, 10, 11};
#endif

static const uint8_t pNET_MASK[ipIP_ADDRESS_LENGTH_BYTES] = {255, 255, 255, 0};
static const uint8_t pGATEWAY_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {10, 10, 10, 1};
static const uint8_t pDNS_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {10, 10, 10, 1};

static void HardwareConfigurePerformance(void);
static void HardwareUseMultiVectoredInterrupts(void);
static void HardwareConfigPeripherals(void);

#if defined(__PIC32MZ__) && (__PIC32_FEATURE_SET1 == 'F')
static uint32_t TRNGRead32(void);
static uint64_t TRNGRead64(void);
#endif

portTASK_FUNCTION_PROTO(Task1, pParams);
portTASK_FUNCTION_PROTO(Task2, pParams);
portTASK_FUNCTION_PROTO(PacketTask, pParams);

TaskHandle_t g_hTask1;
TaskHandle_t g_hTask2;
TaskHandle_t g_hPacketTask;

/*
 *
 */
int main(int argc, char *argv[])
{
    // Disable interrupts - note taskDISABLE_INTERRUPTS() cannot be used here as
    // FreeRTOS does not globally disable interrupts
    __builtin_disable_interrupts();

    HardwareConfigurePerformance();
    HardwareUseMultiVectoredInterrupts();
    HardwareConfigPeripherals();

    portDISABLE_INTERRUPTS();

    xTaskCreate(&Task1, "Task1", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &g_hTask1);
    xTaskCreate(&Task2, "Task2", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, &g_hTask2);
    xTaskCreate(&PacketTask, "PacketTx", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 4, &g_hPacketTask);

    uint8_t tMacAddr[ipMAC_ADDRESS_LENGTH_BYTES] = {EMAC1SA2bits.STNADDR1, EMAC1SA2bits.STNADDR2,
                                                    EMAC1SA1bits.STNADDR3, EMAC1SA1bits.STNADDR4,
                                                    EMAC1SA0bits.STNADDR5, EMAC1SA0bits.STNADDR6};

    FreeRTOS_IPInit(pIP_ADDRESS, pNET_MASK, pGATEWAY_ADDRESS, pDNS_ADDRESS, tMacAddr);

    vTaskStartScheduler();

    return EXIT_SUCCESS;
}

/*-----------------------------------------------------------*/

#if defined(__PIC32MZ__)

void HardwareConfigurePerformance(void)
{
    SYSKEY_UNLOCK();

    // Disable unused peripherals (REFCLKs not disabled due to erratum #5)
    PMD1SET = _PMD1_CVRMD_MASK;
    PMD2SET = _PMD2_CMP1MD_MASK | _PMD2_CMP2MD_MASK;
    PMD3SET = (_PMD3_IC1MD_MASK | _PMD3_IC2MD_MASK | _PMD3_IC3MD_MASK
               | _PMD3_IC4MD_MASK | _PMD3_IC5MD_MASK | _PMD3_IC6MD_MASK
               | _PMD3_IC7MD_MASK | _PMD3_IC8MD_MASK | _PMD3_IC9MD_MASK
               | _PMD3_OC1MD_MASK | _PMD3_OC2MD_MASK | _PMD3_OC3MD_MASK
               | _PMD3_OC4MD_MASK | _PMD3_OC5MD_MASK | _PMD3_OC6MD_MASK
               | _PMD3_OC7MD_MASK | _PMD3_OC8MD_MASK | _PMD3_OC9MD_MASK);
    PMD5SET = (_PMD5_U1MD_MASK | _PMD5_U3MD_MASK | _PMD5_U4MD_MASK
               | _PMD5_U5MD_MASK | _PMD5_U6MD_MASK | _PMD5_SPI1MD_MASK
               | _PMD5_SPI2MD_MASK | _PMD5_SPI3MD_MASK | _PMD5_SPI4MD_MASK
               | _PMD5_SPI5MD_MASK | _PMD5_SPI6MD_MASK | _PMD5_I2C1MD_MASK
               | _PMD5_I2C2MD_MASK | _PMD5_I2C3MD_MASK | _PMD5_I2C5MD_MASK
               | _PMD5_CAN1MD_MASK | _PMD5_CAN2MD_MASK );
    PMD6SET = (_PMD6_RTCCMD_MASK | _PMD6_PMPMD_MASK | _PMD6_EBIMD_MASK);

    // Disable analogue for UART2 pins (B14, G6) and I2C4 pins (G7, G8)
    ANSELBCLR = _ANSELB_ANSB14_MASK;
    ANSELGCLR = _ANSELG_ANSG6_MASK | _ANSELG_ANSG7_MASK | _ANSELG_ANSG8_MASK;

    // PPS and I/O configuration for UART2
    TRISGSET = _TRISG_TRISG6_MASK;
    U2RXRbits.U2RXR = 0b0001;   // RPG6
    RPB14Rbits.RPB14R = 0b0010; // U2TX

    // Disable analogue for Ethernet RMII pins (H4, H5, J8, J9, J11)
    ANSELHCLR = _ANSELH_ANSH4_MASK | _ANSELH_ANSH5_MASK;
    ANSELJCLR = _ANSELJ_ANSJ8_MASK | _ANSELJ_ANSJ9_MASK | _ANSELJ_ANSJ11_MASK;

    // PPS and I/O for Ethernet PHY
    LAN8740_ASSERT_HW_RESET();
    TRISHCLR = _TRISH_TRISH11_MASK; // nRST
    TRISCSET = _TRISC_TRISC13_MASK; // nINT
    INT4Rbits.INT4R = 0b0111;       // RPC13

    INTCONCLR = _INTCON_INT4EP_MASK;    // nINT active low so falling edge

    // I/O configuration for LEDs/switches
    // Note: Erratum #7 on the PIC32MZ EC means that LEDs 1 and 2 won't work
    //       while the Ethernet Controller is enabled
    ANSELBCLR = _ANSELB_ANSB12_MASK | _ANSELB_ANSB13_MASK;  // SW1, SW2
    ANSELHCLR = _ANSELH_ANSH0_MASK | _ANSELH_ANSH1_MASK;    // LED1, LED2
    TRISHCLR = 0x07;

    // Configure peripheral busses

    // Set PBCLK2 to deliver 40Mhz clock for PMP/I2C/UART/SPI
    // 200MHz / 5 = 40MHz
    PB2DIVbits.PBDIV = 0b0000100;

    // Timers use clock PBCLK3, set this to 40MHz
    PB3DIVbits.PBDIV = 0b0000100;

    // Ports use PBCLK4, max allowed frequency is 100MHz
    PB4DIVbits.PBDIV = 0b0000001;

    // Set PBCLK5 to 100MHz for Ethernet
    PB5DIVbits.PBDIV = 0b0000001;

    // Enable prefetch module
    PRECONbits.PFMWS = 2;   // 2 wait state
    PRECONbits.PREFEN = 3;  // Enable predictive prefetch for any address

    SYSKEY_LOCK();
}

void HardwareUseMultiVectoredInterrupts(void)
{
    _CP0_BIS_CAUSE(_CP0_CAUSE_IV_MASK);
    INTCONSET = _INTCON_MVEC_MASK;
}

#elif defined(__PIC32MX__)

static void PIC32MXConfigureWaitStates(void)
{
    uint32_t nSystemClock = configCPU_CLOCK_HZ - 1;

    uint8_t nWaitStates = 0;

    while(nSystemClock > PIC32MX_MAX_FLASH_SPEED)
    {
        nWaitStates++;
        nSystemClock -= PIC32MX_MAX_FLASH_SPEED;
    }

    uint32_t tmp = CHECON & ~(_CHECON_PFMWS_MASK | _CHECON_PREFEN_MASK);

    if(nWaitStates > 0)
    {
        tmp |= (CHECON_PREFEN_ALL_REGIONS << _CHECON_PREFEN_POSITION);
    }

    // Set flash wait states and enabled prefetch for all regions
    CHECON = (tmp | (nWaitStates << _CHECON_PFMWS_POSITION));
}

void HardwareConfigurePerformance(void)
{
    // Set analog pins to digital mode
    AD1PCFG = _AD1PCFG_PCFG_MASK;

    // I/O for Ethernet PHY
    TRISASET = _TRISA_TRISA14_MASK;
    INTCONCLR = _INTCON_INT3EP_MASK;    // nINT active low so falling edge

    // I/O configuration for LEDs/switches
    TRISDCLR = 0x07;

    // Disable SRAM wait states
    BMXCONCLR = _BMXCON_BMXWSDRM_MASK;

    // Configure flash wait states and prefetch caches
    PIC32MXConfigureWaitStates();

    // Make Kseg0 cacheable
    uint32_t cfg0 = _CP0_GET_CONFIG();
    cfg0 = (cfg0 & ~_CP0_CONFIG_K0_MASK) | (CP0_CONFIG_K0_CACHEABLE << _CP0_CONFIG_K0_POSITION);
    _mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, cfg0);
}

void HardwareUseMultiVectoredInterrupts(void)
{
    extern uint32_t _ebase_address[];

    uint32_t status = _CP0_GET_STATUS();
    status |= _CP0_STATUS_BEV_MASK;
    _CP0_SET_STATUS(status);

    // Setup EBase
    _CP0_SET_EBASE((uintptr_t) _ebase_address);

    // Space vectors by 0x20 bytes
    _CP0_XCH_INTCTL(0x20);

    uint32_t cause = _CP0_GET_CAUSE();
    cause |= _CP0_CAUSE_IV_MASK;
    _CP0_SET_CAUSE(cause);

    status &= ~(_CP0_STATUS_BEV_MASK | _CP0_STATUS_EXL_MASK);
    _CP0_SET_STATUS(status);

    INTCONSET = _INTCON_MVEC_MASK;
}

#else

#error Unsupported processor

#endif

/*-----------------------------------------------------------*/

void HardwareConfigPeripherals(void)
{
#if defined(__PIC32MZ__)

#if (__PIC32_FEATURE_SET1 == 'F')
    // Enable TRNG
    RNGCONbits.TRNGEN = 1;

    // Seed the C Runtime pseudo random sequence using the TRNG
    srand( TRNGRead32() );
#endif

    LATHCLR = 0x07;

#elif defined(__PIC32MX__)
    LATDCLR = 0x07;
#endif

    Uart2Initialise(UART2_BAUD_RATE);
}

time_t FreeRTOS_time(time_t *pxTime)
{
    return time(pxTime);
}

#if defined(__PIC32MZ__) && (__PIC32_FEATURE_SET1 == 'F')

uint32_t TRNGRead32(void)
{
#ifndef __MPLAB_DEBUGGER_SIMULATOR
    while(RNGCNTbits.RCNT < 32);
#endif

    return RNGSEED1;
}

uint64_t TRNGRead64(void)
{
#ifndef __MPLAB_DEBUGGER_SIMULATOR
    while(RNGCNTbits.RCNT < 64);
#endif

    return ((uint64_t) RNGSEED2 << 32U) | RNGSEED1;
}

#endif
