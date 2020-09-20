/*
 * Startup and Hardware Configuration
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
#include "TestHarness.h"

#if defined(__PIC32MX__)

#define PIC32MX_MAX_FLASH_SPEED     30000000UL

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
static const uint8_t pIP_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {172, 16, 1, 201};
#else
static const uint8_t pIP_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {172, 16, 1, 200};
#endif

static const uint8_t pNET_MASK[ipIP_ADDRESS_LENGTH_BYTES] = {255, 255, 255, 0};
static const uint8_t pGATEWAY_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {172, 16, 1, 1};
static const uint8_t pDNS_ADDRESS[ipIP_ADDRESS_LENGTH_BYTES] = {172, 16, 1, 1};

static const uint8_t pDEVEL_MAC_ADDR[ipMAC_ADDRESS_LENGTH_BYTES] = {0x02, 'W', 'o', 'o', 'f', '!'};

static void HardwareConfigurePerformance(void);
static void HardwareUseMultiVectoredInterrupts(void);
static void HardwareConfigPeripherals(void);

#if defined(PIC32_HAS_WORKING_TRNG)

extern uint32_t TRNGRead32(void);
extern uint64_t TRNGRead64(void);

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

    xTaskCreate(&Task1, "Task1", 350, NULL, tskIDLE_PRIORITY + 1, &g_hTask1);
    xTaskCreate(&Task2, "Task2", 620, NULL, tskIDLE_PRIORITY + 1, &g_hTask2);
    xTaskCreate(&PacketTask, "Packet", 200, NULL, tskIDLE_PRIORITY + 4, &g_hPacketTask);

#if defined(__PIC32MZ__) && (__PIC32_FEATURE_SET0 == 'D')
    FreeRTOS_IPInit(pIP_ADDRESS, pNET_MASK, pGATEWAY_ADDRESS, pDNS_ADDRESS, pDEVEL_MAC_ADDR);
#else
    uint8_t tMacAddr[ipMAC_ADDRESS_LENGTH_BYTES] = {EMAC1SA2bits.STNADDR1, EMAC1SA2bits.STNADDR2,
                                                    EMAC1SA1bits.STNADDR3, EMAC1SA1bits.STNADDR4,
                                                    EMAC1SA0bits.STNADDR5, EMAC1SA0bits.STNADDR6};

    FreeRTOS_IPInit(pIP_ADDRESS, pNET_MASK, pGATEWAY_ADDRESS, pDNS_ADDRESS, tMacAddr);
#endif

    vTaskStartScheduler();

    return EXIT_SUCCESS;
}

/*-----------------------------------------------------------*/

#if defined(__PIC32MZ__)

#if (__PIC32_FEATURE_SET0 == 'E')

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
    ipconfigPIC32_PHY_CLEAR_HW_RESET();
    TRISHCLR = _TRISH_TRISH11_MASK; // nRST
    TRISCSET = _TRISC_TRISC13_MASK; // nINT
    INT4Rbits.INT4R = 0b0111;       // RPC13

    INTCONCLR = _INTCON_INT4EP_MASK;    // nINT active low so falling edge
    IPC5bits.INT4IP = configKERNEL_INTERRUPT_PRIORITY;

    // I/O configuration for LEDs/switches
    // Note: Erratum #7 on the PIC32MZ EC means that LEDs 1 and 2 won't work
    //       while the Ethernet Controller is enabled
    ANSELBCLR = _ANSELB_ANSB12_MASK | _ANSELB_ANSB13_MASK;  // SW1, SW2
    CNPUBSET = _CNPUB_CNPUB12_MASK | _CNPUB_CNPUB13_MASK | _CNPUB_CNPUB14_MASK;
    ANSELHCLR = _ANSELH_ANSH0_MASK | _ANSELH_ANSH1_MASK;    // LED1, LED2
    TRISHCLR = _TRISH_TRISH0_MASK | _TRISH_TRISH1_MASK | _TRISH_TRISH2_MASK;

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

#elif (__PIC32_FEATURE_SET0 == 'D')

#define QNSEC(f) (((uint16_t) f << 2) | (uint8_t)((f - (int) f) * 4.0f))

typedef struct {
    uint16_t tRFC;
    uint8_t tWR;
    uint8_t tRP;
    uint8_t tRCD;
    uint8_t tRRD;
    uint8_t tWTR;
    uint8_t tRTP;
    uint8_t DLLK;
    uint8_t tRAS;
    uint8_t tRC;
    uint8_t tFAW;
    uint8_t MRD;
    uint8_t XP;
    uint8_t CKE;
    uint8_t RL;
    uint16_t tRFI;
    uint8_t WL;
    uint8_t BL;
} sdram_timing_params_t;

typedef struct {
    uint8_t minlimit;
    uint8_t rqper;
    uint8_t mincmd;
} sdram_arb_params_t;

// FBGA marking: D9SBC == MT47H64M16NF-25E XIT:M
static const sdram_timing_params_t tMT47H64M16NF_25E = {
    QNSEC(127.5f),  // tRFC Idd
    QNSEC(15.0f),   // tWR
    QNSEC(12.5f),   // tRP Idd
    QNSEC(12.5f),   // tRCD Idd
    QNSEC(10.0f),   // tRRD Idd
    QNSEC( 7.5f),   // tWTR
    QNSEC( 7.5f),   // tRTP
    200,            // DLLK
    QNSEC(45.0f),   // tRAS Idd
    QNSEC(57.5f),   // tRC Idd
    QNSEC(45.0f),   // tFAW
    2,              // MRD
    2,              // XP
    3,              // CKE
    5,              // RL/CL Idd
    QNSEC(7800.0f), // tRFI
    4,              // WL
    2               // BL
};

static const sdram_arb_params_t tDDR_ARB_PARAMS[] = {
    {0x1F, 0xFF, 0x04},
    {0x1F, 0xFF, 0x10},
    {0x1F, 0xFF, 0x10},
    {0x04, 0xFF, 0x04},
    {0x04, 0xFF, 0x04}
};

#define DDR_DRAM_CLK_PERIOD QNSEC(2.5f)
#define DDR_CTRL_CLK_PERIOD (DDR_DRAM_CLK_PERIOD * 2)

#define MPLL_IDIV           3
#define MPLL_MULT           50
#define MPLL_ODIV1          2
#define MPLL_ODIV2          1

// DDR addressing scheme: CS, ROW, BA, COL
#define DDR_CS_BITS         1
#define DDR_ROW_BITS        13
#define DDR_BA_BITS         3
#define DDR_COL_BITS        10

#define QNSEC_DIV(n, d) (((uint16_t)(n) + (d) - QNSEC(1.0f)) / (d))
#define QNSEC_TICKS(q)  (QNSEC_DIV(q, DDR_CTRL_CLK_PERIOD))

typedef enum {
    DRAM_CMD_DSELECT = 0xFFFU,
    DRAM_CMD_PCALL   = 0x401U,
    DRAM_CMD_REF     = 0x801U,
    DRAM_CMD_LDM     = 0x001U
} dram_cmd_t;

#define DRAM_CMD_LDREG(ba)   (DRAM_CMD_LDM | (ba << 12))

#define DRAM_CMD_PCALL_ALL_BANKS    0x0400

typedef enum {
    DRAM_MR_BURST_LEN_POSITION = 0,
    DRAM_MR_BURST_LEN_4        = 0x02 << DRAM_MR_BURST_LEN_POSITION,
    DRAM_MR_BURST_LEN_8        = 0x03 << DRAM_MR_BURST_LEN_POSITION,
    DRAM_MR_BURST_INTERLEAVED  = 1 << 3,
    DRAM_MR_CL_POSITION        = 4,
    DRAM_MR_DLL_RESET          = 1 << 8,
    DRAM_MR_WR_POSITION        = 9,
    DRAM_MR_SLOW_EXIT          = 1 << 12
} dram_mr_t;

typedef enum {
    DRAM_EMR_DLL_DISABLED     = 1 << 0,
    DRAM_EMR_ODS_REDUCED      = 1 << 1,
    DRAM_EMR_CAL_POSITION     = 3,
    DRAM_EMR_TERM_R150        = (0 << 2) | (1 << 6),
    DRAM_EMR_OCD_POSITION     = 7,
    DRAM_EMR_OCD_EXIT         = 0x00 << DRAM_EMR_OCD_POSITION,
    DRAM_EMR_OCD_DEFAULTS     = 0x07 << DRAM_EMR_OCD_POSITION,
    DRAM_EMR_DQS_DISABLE      = 1 << 10,
    DRAM_EMR_RDQS_ENABLE      = 1 << 11,
    DRAM_EMR_OUTPUTS_DISABLED = 1 << 12
} dram_emr_t;

void DRAM_Initialise(const sdram_timing_params_t *p)
{
    uint16_t MR = DRAM_MR_BURST_LEN_8 | (p->RL << DRAM_MR_CL_POSITION)
                  | ((QNSEC_DIV(p->tWR, DDR_DRAM_CLK_PERIOD) - 1) << DRAM_MR_WR_POSITION);
    uint16_t EMR = DRAM_EMR_TERM_R150;

    struct {
        uint16_t cmd;
        uint16_t modeAddr;
        uint16_t timing;
    } tDRAM_INIT_CMDS[] = {
        {DRAM_CMD_DSELECT, 0x0000, QNSEC(400)},

        {DRAM_CMD_PCALL, DRAM_CMD_PCALL_ALL_BANKS, p->tRP + DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_LDREG(2), 0x0000, p->MRD * DDR_DRAM_CLK_PERIOD},
        {DRAM_CMD_LDREG(3), 0x0000, p->MRD * DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_LDREG(1), EMR, p->MRD * DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_LDREG(0), MR | DRAM_MR_DLL_RESET, p->MRD * DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_PCALL, DRAM_CMD_PCALL_ALL_BANKS, p->tRP + DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_REF, 0x0000, p->tRFC},
        {DRAM_CMD_REF, 0x0000, p->tRFC},

        {DRAM_CMD_LDREG(0), MR, p->MRD * DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_LDREG(1), EMR | DRAM_EMR_OCD_DEFAULTS, p->MRD * DDR_DRAM_CLK_PERIOD},

        {DRAM_CMD_LDREG(1), EMR, p->DLLK * DDR_DRAM_CLK_PERIOD}
    };

    volatile uint32_t *pCmd1Array = &DDRCMD10;
    volatile uint32_t *pCmd2Array = &DDRCMD20;

    size_t c;
    for(c = 0; c < _countof(tDRAM_INIT_CMDS); c++)
    {
        uint32_t delay = max(QNSEC_DIV(tDRAM_INIT_CMDS[c].timing, DDR_DRAM_CLK_PERIOD), 2) - 2;
        pCmd1Array[c] = (tDRAM_INIT_CMDS[c].cmd & 0xFFF) | (DRAM_CMD_DSELECT << 12)
                        | ((tDRAM_INIT_CMDS[c].modeAddr & 0xFF) << 24);
        pCmd2Array[c] = (tDRAM_INIT_CMDS[c].modeAddr >> 8)
                        | ((tDRAM_INIT_CMDS[c].cmd & 0x7000) >> 4)
                        | (delay << 11);
    }

    DDRCMDISSUE = (_countof(tDRAM_INIT_CMDS) << _DDRCMDISSUE_NUMHOSTCMDS_POSITION)
                  | _DDRCMDISSUE_VALID_MASK;

    DDRMEMCON = _DDRMEMCON_STINIT_MASK;

    while(DDRCMDISSUE & _DDRCMDISSUE_VALID_MASK);
}

void DDR2_Configure(const sdram_timing_params_t *p)
{
    CFGMPLL &= ~_CFGMPLL_MPLLVREGDIS_MASK;
    while((CFGMPLL & _CFGMPLL_MPLLVREGRDY_MASK) == 0);

    // Configure MPLL
    CFGMPLL &= ~(_CFGMPLL_MPLLIDIV_MASK | _CFGMPLL_MPLLMULT_MASK
                 | _CFGMPLL_MPLLODIV1_MASK | _CFGMPLL_MPLLODIV2_MASK);

    CFGMPLL |= (MPLL_IDIV << _CFGMPLL_MPLLIDIV_POSITION)
                | (MPLL_MULT << _CFGMPLL_MPLLMULT_POSITION)
                | (MPLL_ODIV1 << _CFGMPLL_MPLLODIV1_POSITION)
                | (MPLL_ODIV2 << _CFGMPLL_MPLLODIV2_POSITION);

    CFGMPLL &= ~_CFGMPLL_MPLLDIS_MASK;
    while((CFGMPLL & _CFGMPLL_MPLLRDY_MASK) == 0);

    PMD7CLR = _PMD7_DDR2CMD_MASK;

    // Initialise PHY using recommended settings from DS60001321C
    DDRSCLCFG0 = _DDRSCLCFG0_BURST8_MASK | _DDRSCLCFG0_DDR2_MASK
                 | (p->RL << _DDRSCLCFG0_RCASLAT_POSITION)
                 | _DDRSCLCFG0_ODTCSW_MASK;
    DDRSCLCFG1 = _DDRSCLCFG1_SCLCSEN_MASK
                 | (p->WL << _DDRSCLCFG1_WCASLAT_POSITION);

    DDRPHYPADCON = _DDRPHYPADCON_ODTSEL_MASK | _DDRPHYPADCON_ODTEN_MASK
                   | (2 << _DDRPHYPADCON_ODTPDCAL_POSITION)
                   | (3 << _DDRPHYPADCON_ODTPUCAL_POSITION)
                   | _DDRPHYPADCON_NOEXTDLL_MASK | _DDRPHYPADCON_WRCMDDLY_MASK
                   | _DDRPHYPADCON_HALFRATE_MASK
                   | (14 << _DDRPHYPADCON_DRVSTRNFET_POSITION)
                   | (14 << _DDRPHYPADCON_DRVSTRPFET_POSITION)
                   | _DDRPHYPADCON_RCVREN_MASK
                   | (2 << _DDRPHYPADCON_PREAMBDLY_POSITION);

    DDRSCLLAT = (3 << _DDRSCLLAT_CAPCLKDLY_POSITION)
                | (4 << _DDRSCLLAT_DDRCLKDLY_POSITION);

    DDRPHYDLLR = (0x10 << _DDRPHYDLLR_RECALIBCNT_POSITION)
                 | (0 << _DDRPHYDLLR_DISRECALIB_POSITION)
                 | (3 << _DDRPHYDLLR_DLYSTVAL_POSITION);

    // Initialise the DDR Controller
    DDRMEMWIDTH = _DDRMEMWIDTH_HALFRATE_MASK;

    size_t c;
    for(c = 0; c < _countof(tDDR_ARB_PARAMS); c++)
    {
        DDRTSEL = c * _DDRMINLIM_MINLIMIT_LENGTH;
        DDRMINLIM = tDDR_ARB_PARAMS[c].minlimit;

        DDRTSEL = c * _DDRRQPER_RQPER_LENGTH;
        DDRRQPER = tDDR_ARB_PARAMS[c].rqper;

        DDRTSEL = c * _DDRMINCMD_MINCMD_LENGTH;
        DDRMINCMD = tDDR_ARB_PARAMS[c].mincmd;
    }

    // Column address bits <= 13 so column high mask/position not programmed
    // Only one chip select so chip select mask/position not programmed
    DDRMEMCFG0 = ((DDR_COL_BITS + DDR_BA_BITS) << _DDRMEMCFG0_RWADDR_POSITION)
                 | (DDR_COL_BITS << _DDRMEMCFG0_BNKADDR_POSITION);
    DDRMEMCFG1 = (1 << DDR_ROW_BITS) - 1;
    DDRMEMCFG2 = 0;
    DDRMEMCFG3 = (1 << DDR_COL_BITS) - 1;
    DDRMEMCFG4 = (1 << DDR_BA_BITS) - 1;

    DDRREFCFG = ((QNSEC_TICKS(p->tRFI) - 2) << _DDRREFCFG_REFCNT_POSITION)
                | ((QNSEC_TICKS(p->tRFC) - 2) << _DDRREFCFG_REFDLY_POSITION)
                | (7 << _DDRREFCFG_MAXREFS_POSITION);

    DDRPWRCFG = (8 << _DDRPWRCFG_PWRDNDLY_POSITION)
                | (17 << _DDRPWRCFG_SLFREFDLY_POSITION);

    uint8_t nxtdatavdly = p->WL;
    uint8_t w2rdly = QNSEC_TICKS(p->tWTR) + p->WL + p->BL;
    uint8_t w2rcsdly = max(w2rdly - 1, 3);
    uint8_t w2pchrgdly = QNSEC_TICKS(p->tWR) + p->WL + p->BL;
    uint16_t slfrefexdly = ((p->DLLK + 1) / 2) - 2;

    DDRDLYCFG0 = ((w2rdly & 0x0F) << _DDRDLYCFG0_W2RDLY_POSITION)
                 | ((w2rcsdly & 0x0F) << _DDRDLYCFG0_W2RCSDLY_POSITION)
                 | ((p->BL - 1) << _DDRDLYCFG0_R2RDLY_POSITION)
                 | (p->BL << _DDRDLYCFG0_R2RCSDLY_POSITION)
                 | ((p->BL - 1) << _DDRDLYCFG0_W2WDLY_POSITION)
                 | ((p->BL - 1) << _DDRDLYCFG0_W2WCSDLY_POSITION)
                 | ((p->BL + 2) << _DDRDLYCFG0_R2WDLY_POSITION)
                 | ((p->RL - p->WL + 3) << _DDRDLYCFG0_RMWDLY_POSITION);
    DDRDLYCFG1 = ((p->CKE - 1) << _DDRDLYCFG1_SLFREFMINDLY_POSITION)
                 | ((slfrefexdly & 0xFF) << _DDRDLYCFG1_SLFREFEXDLY_POSITION)
                 | ((p->CKE - 1) << _DDRDLYCFG1_PWRDNMINDLY_POSITION)
                 | (max(p->XP - 1, p->CKE - 1) << _DDRDLYCFG1_PWRDNEXDLY_POSITION)
                 | ((w2pchrgdly >> _DDRDLYCFG2_W2PCHRGDLY_LENGTH) << _DDRDLYCFG1_W2PCHRGDLY4_POSITION)
                 | ((w2rdly >> _DDRDLYCFG0_W2RDLY_LENGTH) << _DDRDLYCFG1_W2RDLY4_POSITION)
                 | ((w2rcsdly >> _DDRDLYCFG0_W2RCSDLY_LENGTH) << _DDRDLYCFG1_W2RCSDLY4_POSITION)
                 | ((nxtdatavdly >> _DDRXFERCFG_NXTDATAVDLY_LENGTH) << _DDRDLYCFG1_NXTDATAVDLY4_POSITION)
                 | ((slfrefexdly >> 8) << _DDRDLYCFG1_SLFREFEXDLY8_POSITION);
    DDRDLYCFG2 = (QNSEC_TICKS(p->tRP) << _DDRDLYCFG2_PCHRGALLDLY_POSITION)
                 | ((QNSEC_TICKS(p->tRTP) + p->BL - 2) << _DDRDLYCFG2_R2PCHRGDLY_POSITION)
                 | ((w2pchrgdly & 0x0F) << _DDRDLYCFG2_W2PCHRGDLY_POSITION)
                 | ((QNSEC_TICKS(p->tRRD) - 1) << _DDRDLYCFG2_RAS2RASDLY_POSITION)
                 | ((QNSEC_TICKS(p->tRCD) - 1) << _DDRDLYCFG2_RAS2CASDLY_POSITION)
                 | ((QNSEC_TICKS(p->tRP) - 1) << _DDRDLYCFG2_PCHRG2RASDLY_POSITION)
                 | ((p->RL + 3) << _DDRDLYCFG2_RBENDDLY_POSITION);
    DDRDLYCFG3 = ((QNSEC_TICKS(p->tRAS) - 1) << _DDRDLYCFG3_RAS2PCHRGDLY_POSITION)
                 | ((QNSEC_TICKS(p->tRC) - 1) << _DDRDLYCFG3_RAS2RASSBNKDLY_POSITION)
                 | ((QNSEC_TICKS(p->tFAW) - 1) << _DDRDLYCFG3_FAWTDLY_POSITION);

    DDRODTCFG = ((p->RL - 3) << _DDRODTCFG_ODTRDLY_POSITION)
                | ((p->WL - 3) << _DDRODTCFG_ODTWDLY_POSITION)
                | (2 << _DDRODTCFG_ODTRLEN_POSITION)
                | (3 << _DDRODTCFG_ODTWLEN_POSITION);
    DDRODTENCFG = _DDRODTENCFG_ODTWEN_MASK;

    DDRXFERCFG = ((p->WL - 2) << _DDRXFERCFG_NXTDATRQDLY_POSITION)
                 | ((nxtdatavdly & 0x0F) << _DDRXFERCFG_NXTDATAVDLY_POSITION)
                 | ((p->RL - 3) << _DDRXFERCFG_RDATENDLY_POSITION)
                 | (3 << _DDRXFERCFG_MAXBURST_POSITION);

    DRAM_Initialise(p);

    DDRMEMCON = _DDRMEMCON_STINIT_MASK | _DDRMEMCON_INITDN_MASK;

    DDRSCLSTART = _DDRSCLSTART_SCLEN_MASK | _DDRSCLSTART_SCLSTART_MASK;

    while((DDRSCLSTART & _DDRSCLSTART_SCLUBPASS_MASK) == 0);
}

void HardwareConfigurePerformance(void)
{
    SYSKEY_UNLOCK();

    // Disable unused peripherals (REFCLKs not disabled due to erratum #5)
    PMD1SET = _PMD1_CTMUMD_MASK | _PMD1_CVRMD_MASK | _PMD1_LVDMD_MASK;
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
               | _PMD5_I2C2MD_MASK | _PMD5_I2C4MD_MASK | _PMD5_I2C5MD_MASK
               | _PMD5_CAN1MD_MASK | _PMD5_CAN2MD_MASK );
    PMD6SET = (_PMD6_PMPMD_MASK | _PMD6_EBIMD_MASK | _PMD6_GPUMD_MASK
               | _PMD6_GLCDMD_MASK);

    // I/O configuration for UART2 [pins B0, G9] and I2C3 [pins F2, F8]
    ANSELBCLR = _ANSELB_ANSB0_MASK;
    ANSELGCLR = _ANSELG_ANSG9_MASK;

    TRISBSET = _TRISB_TRISB0_MASK;
    U2RXRbits.U2RXR = 0b0101;   // RPB0
    RPG9Rbits.RPG9R = 0b0010;   // U2TX

    // I/O configuration for Ethernet PHY
    ipconfigPIC32_PHY_CLEAR_HW_RESET();
    ANSELBCLR = _ANSELB_ANSB11_MASK;
    TRISJCLR = _TRISJ_TRISJ15_MASK; // nRST
    TRISBSET = _TRISB_TRISB11_MASK; // nINT

    CNCONBSET = _CNCONB_ON_MASK | _CNCONB_EDGEDETECT_MASK;
    CNNEBSET = _CNNEB_CNNEB11_MASK;
    IPC29bits.CNBIP = configKERNEL_INTERRUPT_PRIORITY;

    // I/O configuration for LEDs/switches
    ANSELBCLR = _ANSELB_ANSB12_MASK | _ANSELB_ANSB13_MASK | _ANSELB_ANSB14_MASK;
    TRISHCLR = _TRISH_TRISH0_MASK | _TRISH_TRISH1_MASK | _TRISH_TRISH2_MASK;

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

    DDR2_Configure(&tMT47H64M16NF_25E);
}

#else

#error Unsupported PIC32MZ class processor

#endif

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
    IPC3bits.INT3IP = configKERNEL_INTERRUPT_PRIORITY;

    // I/O configuration for LEDs/switches
    CNPUE = _CNPUE_CNPUE15_MASK | _CNPUE_CNPUE16_MASK | _CNPUE_CNPUE19_MASK;
    TRISDCLR = _TRISD_TRISD0_MASK | _TRISD_TRISD1_MASK | _TRISD_TRISD2_MASK;

    // Disable SRAM wait states
    BMXCONCLR = _BMXCON_BMXWSDRM_MASK;

    // Configure flash wait states and prefetch caches
    PIC32MXConfigureWaitStates();

    // Make Kseg0 cacheable
    _bcsc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, _CP0_CONFIG_K0_MASK, CP0_CONFIG_K0_CACHEABLE << _CP0_CONFIG_K0_POSITION);
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
#if defined(PIC32_HAS_WORKING_TRNG)
    // Enable TRNG
    RNGCONbits.TRNGEN = 1;

    // Seed the C Runtime pseudo random sequence using the TRNG
    srand( TRNGRead32() );
#endif

#if defined(__PIC32MZ__)
    LATHCLR = _LATH_LATH0_MASK | _LATH_LATH1_MASK | _LATH_LATH2_MASK;
#elif defined(__PIC32MX__)
    LATDCLR = _LATD_LATD0_MASK | _LATD_LATD1_MASK | _LATD_LATD2_MASK;
#endif

    Uart2Initialise(UART2_BAUD_RATE);
}

time_t FreeRTOS_time(time_t *pxTime)
{
    return time(pxTime);
}

BaseType_t xApplicationGetRandomNumber(uint32_t *pulNumber)
{
    *pulNumber = rand();
    return pdTRUE;
}

#if defined(PIC32_HAS_WORKING_TRNG)

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
