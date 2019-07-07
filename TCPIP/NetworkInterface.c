/*
 * PIC32 Ethernet Driver for FreeRTOS+TCP
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
#include <list.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
// C Runtime
#include <limits.h>
#include <stdbool.h>
#include <sys/kmem.h>
// TCP/IP stack
#include <FreeRTOS_IP.h>
#include <FreeRTOS_IP_Private.h>
#include <NetworkBufferManagement.h>

#include "Ethernet.h"
#include "PHYGeneric.h"
#include "EthernetPrivate.h"
#include "PIC32Arch.h"

#if (ipconfigZERO_COPY_TX_DRIVER != 1) || (ipconfigZERO_COPY_RX_DRIVER != 1)
#error ipconfigZERO_COPY_TX_DRIVER/ipconfigZERO_COPY_RX_DRIVER must be set to 1
#endif

#if (ipconfigUSE_LINKED_RX_MESSAGES != 1)
#error ipconfigUSE_LINKED_RX_MESSAGES must be set to 1
#endif

#if !defined(ipconfigPIC32_TX_DMA_DESCRIPTORS)
#define ipconfigPIC32_TX_DMA_DESCRIPTORS    10
#endif

#if !defined(ipconfigPIC32_RX_DMA_DESCRIPTORS)
#define ipconfigPIC32_RX_DMA_DESCRIPTORS    20
#endif

#if !defined(ipconfigPIC32_DRV_TASK_BLOCK_TICKS)
#define ipconfigPIC32_DRV_TASK_BLOCK_TICKS  portMAX_DELAY
#endif

#if !defined(ipconfigPIC32_VERIFY_BUFFER_DESCRIPTOR_LINK)
#define ipconfigPIC32_VERIFY_BUFFER_DESCRIPTOR_LINK 0
#endif

#if !defined(ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ)
#define ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ    2500000UL
#endif // ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ

#if defined(__PIC32MX__)

#if defined(ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ)
#error The clock source is not configurable on the PIC32MX
#endif // ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ

#define ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ   configCPU_CLOCK_HZ

#elif defined(__PIC32MZ__)

#if !defined(ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ)
#define ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ   100000000UL
#endif // ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ

#endif

#if defined(__PIC32MZ__)
void __attribute__(( interrupt(IPL0AUTO), vector(_ETHERNET_VECTOR) )) EthernetInterruptWrapper(void);
#define IPCbits IPC38bits
#define CLEAR_INTERRUPT_FLAG()  IFS4CLR = _IFS4_ETHIF_MASK
#define ENABLE_INTERRUPT()      IEC4SET = _IEC4_ETHIE_MASK
#define DISABLE_INTERRUPT()     IEC4CLR = _IEC4_ETHIE_MASK
#elif defined(__PIC32MX__)
void __attribute__(( interrupt(IPL0AUTO), vector(_ETH_VECTOR) )) EthernetInterruptWrapper(void);
#define IPCbits IPC12bits
#define CLEAR_INTERRUPT_FLAG()  IFS1CLR = _IFS1_ETHIF_MASK
#define ENABLE_INTERRUPT()      IEC1SET = _IEC1_ETHIE_MASK
#define DISABLE_INTERRUPT()     IEC1CLR = _IEC1_ETHIE_MASK
#else
#error Unsupported processor
#endif

static const uint8_t pMIIM_CLOCK_DIVIDERS[] = {
    4, 6, 8, 10, 14, 20, 28, 40, 48, 50
};

#define NET_BUFFER_SIZE (ipTOTAL_ETHERNET_FRAME_SIZE + ipBUFFER_PADDING)
#define NET_BUFFER_SIZE_ROUNDED_UP ((NET_BUFFER_SIZE + 3) & ~0x03UL)
#define SELF_TEST_RETRY_COUNT   (5)

static volatile uint8_t s_tNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS][NET_BUFFER_SIZE_ROUNDED_UP] __attribute((aligned(4), coherent));

static volatile eth_dma_descriptor_t s_tTxDMADescriptors[ipconfigPIC32_TX_DMA_DESCRIPTORS] __attribute__((coherent));
static volatile eth_dma_descriptor_t s_tRxDMADescriptors[ipconfigPIC32_RX_DMA_DESCRIPTORS] __attribute__((coherent));

static volatile eth_dma_descriptor_t *s_pCurrentRxDMADescKVA;
static volatile eth_dma_descriptor_t *s_pCurrentTxDMADescKVA;
static volatile eth_dma_descriptor_t *s_pPendingTxDMADescKVA;

static eth_stats_t s_tStats;
static TaskHandle_t s_hTaskWaitingForTestResult = NULL;

TaskHandle_t g_hEthernetTask = NULL;
static SemaphoreHandle_t s_hTxDMABufCountSemaphore = NULL;
static SemaphoreHandle_t s_hTxDMABufMutex = NULL;

SemaphoreHandle_t g_hLinkUpSemaphore = NULL;
volatile BaseType_t g_interfaceState = ETH_NORMAL;

portTASK_FUNCTION_PROTO(EthernetTask, pParams);

static void ControllerInitialise(void);
static void MACConfigure(phy_speed_t speed, bool fullDuplex);
static void InitialiseDMADescriptorLists(void);
static void InitiateWOLandWait(void);
static void PowerdownEthernet(void);
static bool ExecuteSelfTests(void);

#define TEST_IF_TRUE_WITH_RETRY(test, retry) ({ \
    uint8_t count = (retry); bool result = false; \
    while( count-- && !(result = ((test) != 0)) ) { \
        vTaskDelay(1); \
    } \
    result; \
}) \

#if ipconfigPIC32_VERIFY_BUFFER_DESCRIPTOR_LINK == 1

static NetworkBufferDescriptor_t *pxDebugGetNetworkBufferWithDescriptor(size_t xRequestedSizeBytes, TickType_t xBlockTimeTicks)
{
    NetworkBufferDescriptor_t *p = pxGetNetworkBufferWithDescriptor(xRequestedSizeBytes, xBlockTimeTicks);

    if( !p )
        return NULL;

    NetworkBufferDescriptor_t *q = pxPacketBuffer_to_NetworkBuffer(p->pucEthernetBuffer);

    if(p != q)
        __builtin_software_breakpoint();

    return p;
}

#define pxGetNetworkBufferWithDescriptor pxDebugGetNetworkBufferWithDescriptor

#endif

static inline void SwapPacketBuffers(NetworkBufferDescriptor_t *pFirst, NetworkBufferDescriptor_t *pSecond)
{
    uint8_t *pTmp = pFirst->pucEthernetBuffer;
    pFirst->pucEthernetBuffer = pSecond->pucEthernetBuffer;
    pSecond->pucEthernetBuffer = pTmp;

    *((NetworkBufferDescriptor_t **)(pFirst->pucEthernetBuffer - ipBUFFER_PADDING)) = pFirst;
    *((NetworkBufferDescriptor_t **)(pSecond->pucEthernetBuffer - ipBUFFER_PADDING)) = pSecond;
}

portTASK_FUNCTION(EthernetTask, pParams)
{
    for( ; ; )
    {
        uint32_t nTaskEvents;

        xTaskNotifyWait(0, ULONG_MAX, &nTaskEvents, portMAX_DELAY);

        IPStackEvent_t tStackEvent;

        if(nTaskEvents & _ETHIRQ_RXDONE_MASK)
        {
            tStackEvent.eEventType = eNetworkRxEvent;
            tStackEvent.pvData = NULL;

            NetworkBufferDescriptor_t *pTailDescriptor = NULL;

            while((s_pCurrentRxDMADescKVA->hdr.control & ETH_DESC_EOWN) == 0)
            {
                // Prepare packet to be sent to the stack for processing
                NetworkBufferDescriptor_t *pStackRxDescriptor = pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, 0);

                if( pStackRxDescriptor )
                {
                    NetworkBufferDescriptor_t *pRxDescriptor = pxPacketBuffer_to_NetworkBuffer( PA_TO_KVA1((uintptr_t) s_pCurrentRxDMADescKVA->pBufferPA) );

                    SwapPacketBuffers(pStackRxDescriptor, pRxDescriptor);

                    // The Rx buffer also contains the Frame Check Sequence which is not needed
                    pStackRxDescriptor->xDataLength = s_pCurrentRxDMADescKVA->hdr.Count - FCS_LENGTH;
                    pStackRxDescriptor->pxNextBuffer = NULL;

                    s_pCurrentRxDMADescKVA->pBufferPA = (uint8_t *) KVA_TO_PA(pRxDescriptor->pucEthernetBuffer);

                    if( !tStackEvent.pvData )
                        tStackEvent.pvData = pStackRxDescriptor;
                    else
                        pTailDescriptor->pxNextBuffer = pStackRxDescriptor;

                    pTailDescriptor = pStackRxDescriptor;

                    iptraceNETWORK_INTERFACE_RECEIVE();
                }
                else
                {
                    s_tStats.rxNoBuffers++;
                    iptraceETHERNET_RX_EVENT_LOST();
                }

                // Prepare the DMA descriptor to receive again
                s_pCurrentRxDMADescKVA->statusVectorLow = s_pCurrentRxDMADescKVA->statusVectorHigh = 0;
                s_pCurrentRxDMADescKVA->hdr.control = ETH_DESC_NPV | ETH_DESC_EOWN;

                ETHCON1SET = _ETHCON1_BUFCDEC_MASK;

                s_pCurrentRxDMADescKVA = PA_TO_KVA1((uintptr_t) s_pCurrentRxDMADescKVA->pNextDescriptorPA);
            }

            if( tStackEvent.pvData )
            {
                if( !xSendEventStructToIPTask(&tStackEvent, ipconfigPIC32_DRV_TASK_BLOCK_TICKS) )
                {
                    do
                    {
                        NetworkBufferDescriptor_t *pPacketToDump = tStackEvent.pvData;
                        tStackEvent.pvData = pPacketToDump->pxNextBuffer;

                        vReleaseNetworkBufferAndDescriptor(pPacketToDump);
                    }
                    while( tStackEvent.pvData );

                    iptraceETHERNET_RX_EVENT_LOST();
                }
            }
        }

        if(nTaskEvents & _ETHIRQ_TXDONE_MASK)
        {
            xSemaphoreTake(s_hTxDMABufMutex, portMAX_DELAY);

            while( s_pPendingTxDMADescKVA->pBufferPA && ((s_pPendingTxDMADescKVA->hdr.control & ETH_DESC_EOWN) == 0) )
            {
                if( (s_pPendingTxDMADescKVA->statusVectorLow & ETH_TSVL_TRANSMIT_DONE) == 0 )
                {
                    s_tStats.txFailures++;
                }

                uint8_t *pBuffer = PA_TO_KVA1((uintptr_t) s_pPendingTxDMADescKVA->pBufferPA);

                s_pPendingTxDMADescKVA->pBufferPA = NULL;
                s_pPendingTxDMADescKVA->statusVectorLow = s_pCurrentRxDMADescKVA->statusVectorHigh = 0;
                s_pPendingTxDMADescKVA = PA_TO_KVA1((uintptr_t) s_pPendingTxDMADescKVA->pNextDescriptorPA);

                // Release the mutex while the buffer is released and reacquire afterwards
                xSemaphoreGive(s_hTxDMABufMutex);

                xSemaphoreGive(s_hTxDMABufCountSemaphore);
                vReleaseNetworkBufferAndDescriptor( pxPacketBuffer_to_NetworkBuffer(pBuffer) );

                xSemaphoreTake(s_hTxDMABufMutex, portMAX_DELAY);
            }

            xSemaphoreGive(s_hTxDMABufMutex);
        }

        if(nTaskEvents & ETH_TASK_PHY_INTERRUPT)
        {
            PHYDeferredInterruptHandler();
        }

        s_tStats.alignmentErrors += ETHALGNERR;
        s_tStats.fcsErrors += ETHFCSERR;
        s_tStats.framesReceived += ETHFRMRXOK;
        s_tStats.framesTransmitted += ETHFRMTXOK;
        s_tStats.multipleCollisions += ETHMCOLFRM;
        s_tStats.receiveOverflows += ETHRXOVFLOW;
        s_tStats.singleCollisions += ETHSCOLFRM;
    }
}

void EthernetInterruptHandler(void)
{
    unsigned int irqs = ETHIRQ;
    ETHIRQCLR = irqs;

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(g_hEthernetTask, irqs, eSetBits, &bHigherPriorityTaskWoken);

    CLEAR_INTERRUPT_FLAG();

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}

void ControllerInitialise(void)
{
    // Procedure taken from Microchip document ref. 61155C, section 35.4.10
    ETHCON1CLR = _ETHCON1_ON_MASK | _ETHCON1_RXEN_MASK | _ETHCON1_TXRTS_MASK;
    while( ETHSTATbits.ETHBUSY );

    CLEAR_INTERRUPT_FLAG();

    // Clear ETHIRQIE
    ETHIEN = 0;
    ETHIRQ = 0;

    // Clear ETHTXST ETHRXST
    ETHTXSTCLR = _ETHTXST_TXSTADDR_MASK;
    ETHRXSTCLR = _ETHRXST_RXSTADDR_MASK;

    ETHCON1SET = _ETHCON1_ON_MASK;

    // MAC initialisation
    EMAC1CFG1SET = _EMAC1CFG1_SOFTRESET_MASK;
    EMAC1CFG1CLR = _EMAC1CFG1_SOFTRESET_MASK;

    EMAC1SUPPSET = _EMAC1SUPP_RESETRMII_MASK;
    EMAC1SUPPCLR = _EMAC1SUPP_RESETRMII_MASK;

    EMAC1MCFGSET = _EMAC1MCFG_RESETMGMT_MASK;
    EMAC1MCFGCLR = _EMAC1MCFG_RESETMGMT_MASK;

    uint32_t c;
    for(c = 0; c < ARRAY_SIZE(pMIIM_CLOCK_DIVIDERS); c++)
    {
        if((ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ / pMIIM_CLOCK_DIVIDERS[c]) <= ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ)
        {
            EMAC1MCFG = (c + 1) << _EMAC1MCFG_CLKSEL_POSITION;
            break;
        }
    }

    while(EMAC1MIND & _EMAC1MIND_MIIMBUSY_MASK);
}

void MACConfigure(phy_speed_t speed, bool fullDuplex)
{
    if(speed == PHY_SPEED_100MBPS)
        EMAC1SUPPSET = _EMAC1SUPP_SPEEDRMII_MASK;
    else
        EMAC1SUPPCLR = _EMAC1SUPP_SPEEDRMII_MASK;

    // EMAC1IPGT settings as per the reference manual
    if( fullDuplex )
    {
        EMAC1CFG2SET = _EMAC1CFG2_FULLDPLX_MASK;
        EMAC1IPGT = 0x15;
    }
    else
    {
        EMAC1CFG2CLR = _EMAC1CFG2_FULLDPLX_MASK;
        EMAC1IPGT = 0x12;
    }

    EMAC1CFG1 = _EMAC1CFG1_RXENABLE_MASK;
    EMAC1CFG2SET = _EMAC1CFG2_PADENABLE_MASK | _EMAC1CFG2_CRCENABLE_MASK;

    EMAC1MAXF = ipTOTAL_ETHERNET_FRAME_SIZE;
}

BaseType_t xNetworkInterfaceInitialise(void)
{
    if( !g_hEthernetTask )
    {
        // First time through
        g_hLinkUpSemaphore = xSemaphoreCreateBinary();
        s_hTxDMABufCountSemaphore = xSemaphoreCreateCounting(ipconfigPIC32_TX_DMA_DESCRIPTORS, ipconfigPIC32_TX_DMA_DESCRIPTORS);
        s_hTxDMABufMutex = xSemaphoreCreateMutex();
        configASSERT( g_hLinkUpSemaphore && s_hTxDMABufCountSemaphore && s_hTxDMABufMutex );

        memset(&s_tStats, 0, sizeof(s_tStats));
        memset((void *) s_tTxDMADescriptors, 0, sizeof(s_tTxDMADescriptors));
        memset((void *) s_tRxDMADescriptors, 0, sizeof(s_tRxDMADescriptors));

        xTaskCreate(&EthernetTask, "EthDrv", ipconfigPIC32_DRV_TASK_STACK_SIZE, NULL, ipconfigPIC32_DRV_TASK_PRIORITY, &g_hEthernetTask);
        configASSERT(g_hEthernetTask);

        const uint8_t *pMacAddr = FreeRTOS_GetMACAddress();

        EMAC1SA0 = pMacAddr[4] | (pMacAddr[5] << 8);
        EMAC1SA1 = pMacAddr[2] | (pMacAddr[3] << 8);
        EMAC1SA2 = pMacAddr[0] | (pMacAddr[1] << 8);

        if(g_interfaceState != ETH_NORMAL)
        {
            // Need to talk to PHY
            ControllerInitialise();
        }
    }
    else
    {
        DISABLE_INTERRUPT();
        ipconfigPIC32_PHY_DISABLE_INTERRUPT();

        // Link failed/stack reset
        s_tStats.linkFailures++;
        iptraceNETWORK_DOWN();
    }

    do
    {
        switch(g_interfaceState)
        {
        case ETH_POWER_DOWN:
            PowerdownEthernet();
            InterlockedCompareExchange(&g_interfaceState, ETH_NORMAL, ETH_POWER_DOWN);
            break;

        case ETH_WAKE_ON_LAN:
            InitiateWOLandWait();
            InterlockedCompareExchange(&g_interfaceState, ETH_NORMAL, ETH_WAKE_ON_LAN_WOKEN);
            break;

        case ETH_SELF_TEST:
            {
                TaskHandle_t hTask = s_hTaskWaitingForTestResult;
                s_hTaskWaitingForTestResult = NULL;

                if( hTask )
                    xTaskNotify(hTask, ExecuteSelfTests() ? 1 : 0, eSetValueWithOverwrite);

                InterlockedCompareExchange(&g_interfaceState, ETH_NORMAL, ETH_SELF_TEST);
            }

            break;

        default:
            ;
        }

        ControllerInitialise();
        PHYInitialise();

        xSemaphoreTake(g_hLinkUpSemaphore, portMAX_DELAY);
    }
    while(g_interfaceState != ETH_NORMAL);

    // Configure MAC based on PHY negotiated link parameters
    phy_status_t phyStatus;
    PHYGetStatus(&phyStatus);

    MACConfigure(phyStatus.speed, phyStatus.fullDuplex);
    InitialiseDMADescriptorLists();

    ETHHT0 = 0;
    ETHHT1 = 0;

    ETHRXFC = _ETHRXFC_BCEN_MASK | _ETHRXFC_MCEN_MASK | _ETHRXFC_UCEN_MASK | _ETHRXFC_RUNTEN_MASK | _ETHRXFC_CRCOKEN_MASK;

    // Enable interrupts and begin receiving
    IPCbits.ETHIP = ipconfigPIC32_ETH_INT_PRIORITY;
    CLEAR_INTERRUPT_FLAG();
    ENABLE_INTERRUPT();

    ETHIENSET = _ETHIEN_RXDONEIE_MASK | _ETHIEN_TXDONEIE_MASK;

    ETHCON1SET = _ETHCON1_RXEN_MASK;

    return pdPASS;
}

BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t * const pxNetworkBuffer, BaseType_t xReleaseAfterSend)
{
    configASSERT(xReleaseAfterSend != pdFALSE);

    if(pxNetworkBuffer->xDataLength > ipTOTAL_ETHERNET_FRAME_SIZE)
    {
        vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
        return pdFALSE;
    }

    if( !xSemaphoreTake(s_hTxDMABufCountSemaphore, ipconfigPIC32_DRV_TASK_BLOCK_TICKS) )
    {
        vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
        s_tStats.txNoDMADescriptors++;
        return pdFALSE;
    }

    xSemaphoreTake(s_hTxDMABufMutex, portMAX_DELAY);

    configASSERT(s_pCurrentTxDMADescKVA->pBufferPA == NULL);
    configASSERT((s_pCurrentTxDMADescKVA->hdr.control & ETH_DESC_EOWN) == 0);

    s_pCurrentTxDMADescKVA->hdr.control = ETH_DESC_NPV | ETH_DESC_SOP | ETH_DESC_EOP;

    s_pCurrentTxDMADescKVA->pBufferPA = (uint8_t *) KVA_TO_PA(pxNetworkBuffer->pucEthernetBuffer);
    s_pCurrentTxDMADescKVA->hdr.Count = pxNetworkBuffer->xDataLength;

    s_pCurrentTxDMADescKVA->hdr.EOWN = 1;

    s_pCurrentTxDMADescKVA = PA_TO_KVA1((uintptr_t) s_pCurrentTxDMADescKVA->pNextDescriptorPA);

    xSemaphoreGive(s_hTxDMABufMutex);

    ETHCON1SET = _ETHCON1_TXRTS_MASK;

    iptraceNETWORK_INTERFACE_TRANSMIT();

    return pdTRUE;
}

void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS] )
{
    memset(s_tNetworkBuffers, 0xBB, sizeof(s_tNetworkBuffers));

    size_t c;
    for(c = 0; c < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; c++)
    {
        volatile uint8_t *pBuffer = &s_tNetworkBuffers[c][0];

        pxNetworkBuffers[c].pucEthernetBuffer = (uint8_t *) (pBuffer + ipBUFFER_PADDING);

        // The need for the following will be removed in a future version of the stack
        *((NetworkBufferDescriptor_t **) pBuffer) = &pxNetworkBuffers[c];
    }
}

void InitialiseDMADescriptorLists(void)
{
    size_t c;
    volatile eth_dma_descriptor_t *p;

    // Rx descriptors list with the Ethernet Controller owning all blocks
    for(c = 0; c < ipconfigPIC32_RX_DMA_DESCRIPTORS; c++)
    {
        p = &s_tRxDMADescriptors[c];

        p->hdr.control = ETH_DESC_EOWN | ETH_DESC_NPV;
        p->statusVectorLow = p->statusVectorHigh = 0;

        // NetworkBufferDescriptor_t can be retrieved from ipBUFFER_PADDING bytes before the buffer start address
        if( !p->pBufferPA )
        {
            NetworkBufferDescriptor_t *pNBD = pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, portMAX_DELAY);
            configASSERT(pNBD != NULL);
            p->pBufferPA = (uint8_t *) KVA_TO_PA(pNBD->pucEthernetBuffer);
        }

        p->pNextDescriptorPA = (struct eth_dma_descriptor_t *) KVA_TO_PA(&s_tRxDMADescriptors[c + 1]);
    }

    p->pNextDescriptorPA = (struct eth_dma_descriptor_t *) KVA_TO_PA(&s_tRxDMADescriptors[0]);

    ETHRXST = KVA_TO_PA(&s_tRxDMADescriptors[0]);
    s_pCurrentRxDMADescKVA = &s_tRxDMADescriptors[0];

    // Tx descriptor list with the PIC32 owning all the blocks
    for(c = 0; c < ipconfigPIC32_TX_DMA_DESCRIPTORS; c++)
    {
        p = &s_tTxDMADescriptors[c];

        p->hdr.control = ETH_DESC_NPV;
        p->statusVectorLow = p->statusVectorHigh = 0;

        if( p->pBufferPA )
        {
            vReleaseNetworkBufferAndDescriptor( pxPacketBuffer_to_NetworkBuffer( PA_TO_KVA1((uintptr_t) p->pBufferPA) ) );
            p->pBufferPA = NULL;
        }

        p->pNextDescriptorPA = (struct eth_dma_descriptor_t *) KVA_TO_PA(&s_tTxDMADescriptors[c + 1]);
    }

    p->pNextDescriptorPA = (struct eth_dma_descriptor_t *) KVA_TO_PA(&s_tTxDMADescriptors[0]);

    ETHTXST = KVA_TO_PA(&s_tTxDMADescriptors[0]);
    s_pCurrentTxDMADescKVA = s_pPendingTxDMADescKVA = &s_tTxDMADescriptors[0];

    ETHCON2 = (ipTOTAL_ETHERNET_FRAME_SIZE + 15) & ~0x0FUL;

    // Make sure the counting semaphore count is reset to all available
    while( xSemaphoreGive(s_hTxDMABufCountSemaphore) == pdTRUE );

    configASSERT( uxSemaphoreGetCount(s_hTxDMABufCountSemaphore) == ipconfigPIC32_TX_DMA_DESCRIPTORS );
}

bool EthernetPrepareWakeOnLAN(void)
{
    if( !PHYSupportsWOL() )
    {
        return false;
    }

    if( InterlockedCompareExchange(&g_interfaceState, ETH_WAKE_ON_LAN, ETH_NORMAL) != ETH_NORMAL )
    {
        return false;
    }

    if( FreeRTOS_IsNetworkUp() )
        FreeRTOS_NetworkDown();
    else
        xSemaphoreGive(g_hLinkUpSemaphore);

    return true;
}

void InitiateWOLandWait(void)
{
    // Silence the Ethernet controller
    ETHCON1CLR = _ETHCON1_RXEN_MASK | _ETHCON1_TXRTS_MASK;
    while( ETHSTATbits.RXBUSY || ETHSTATbits.TXBUSY );

    // Arm the PHY and wait for an awakening
    PHYPrepareWakeOnLAN();

    xSemaphoreTake(g_hLinkUpSemaphore, portMAX_DELAY);
}

void PowerdownEthernet(void)
{
    ETHCON1CLR = _ETHCON1_RXEN_MASK | _ETHCON1_TXRTS_MASK;
    while( ETHSTATbits.RXBUSY || ETHSTATbits.TXBUSY );

    uint16_t bcr = PHY_READ(PHY_REG_BASIC_CONTROL);
    PHY_WRITE(PHY_REG_BASIC_CONTROL, bcr | PHY_CTRL_POWER_DOWN);

    ETHCON1CLR = _ETHCON1_ON_MASK;
    while( ETHSTATbits.ETHBUSY );

    xSemaphoreTake(g_hLinkUpSemaphore, portMAX_DELAY);
}

eth_interface_state_t EthernetGetInterfaceState(void)
{
    return g_interfaceState;
}

void EthernetInterfaceDown(void)
{
    if( InterlockedCompareExchange(&g_interfaceState, ETH_POWER_DOWN, ETH_NORMAL) != ETH_NORMAL )
    {
        return;
    }

    if( FreeRTOS_IsNetworkUp() )
        FreeRTOS_NetworkDown();
    else
        xSemaphoreGive(g_hLinkUpSemaphore);
}

void EthernetInterfaceUp(void)
{
    if(g_interfaceState == ETH_POWER_DOWN)
    {
        xSemaphoreGive(g_hLinkUpSemaphore);
    }
}

void EthernetSelfTest(TaskHandle_t hNotify)
{
    if( InterlockedCompareExchange(&g_interfaceState, ETH_SELF_TEST, ETH_NORMAL) != ETH_NORMAL )
    {
        return;
    }

    s_hTaskWaitingForTestResult = hNotify;

    if( FreeRTOS_IsNetworkUp() )
        FreeRTOS_NetworkDown();
    else
        xSemaphoreGive(g_hLinkUpSemaphore);
}

void EthernetGetStats(eth_stats_t *pStats)
{
    memcpy(pStats, &s_tStats, sizeof(s_tStats));
}

void EthernetResetStats(void)
{
    memset(&s_tStats, 0, sizeof(s_tStats));
}

bool ExecuteSelfTests(void)
{
    ControllerInitialise();

    // Fix PHY and MAC to operate at 100Mbps full duplex and put the loopback at the PHY
    PHY_WRITE(PHY_REG_BASIC_CONTROL, PHY_CTRL_RESET);

    if( !TEST_IF_TRUE_WITH_RETRY( (PHY_READ(PHY_REG_BASIC_CONTROL) & PHY_CTRL_RESET) == 0, SELF_TEST_RETRY_COUNT ) )
    {
        return false;
    }

    PHY_WRITE(PHY_REG_BASIC_CONTROL, PHY_CTRL_SPEED_100MBPS | PHY_CTRL_FULL_DUPLEX | PHY_CTRL_LOOPBACK);

    if( !TEST_IF_TRUE_WITH_RETRY( PHY_READ(PHY_REG_BASIC_STATUS) & PHY_STAT_LINK_IS_UP, SELF_TEST_RETRY_COUNT ) )
    {
        return false;
    }

    MACConfigure(PHY_SPEED_100MBPS, true);
//  EMAC1CFG1SET = _EMAC1CFG1_LOOPBACK_MASK;

    InitialiseDMADescriptorLists();

    ETHRXFC = _ETHRXFC_BCEN_MASK | _ETHRXFC_MCEN_MASK | _ETHRXFC_NOTMEEN_MASK | _ETHRXFC_UCEN_MASK | _ETHRXFC_CRCOKEN_MASK;

    ETHCON1SET = _ETHCON1_RXEN_MASK;

    uint16_t iterations = min(ipconfigPIC32_TX_DMA_DESCRIPTORS, ipconfigPIC32_RX_DMA_DESCRIPTORS);

    do
    {
        NetworkBufferDescriptor_t *pTxDescriptor = pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, 0);

        if( !pTxDescriptor )
        {
            return false;
        }

        pTxDescriptor->xDataLength = ipconfigNETWORK_MTU + sizeof(EthernetHeader_t);

        EthernetHeader_t *pTxHeader = (EthernetHeader_t *) pTxDescriptor->pucEthernetBuffer;
        memset(pTxHeader, 0xFF, sizeof(*pTxHeader));
        pTxHeader->usFrameType = FreeRTOS_htons(ipconfigNETWORK_MTU);

        uint8_t *pTxData = (uint8_t *) (pTxHeader + 1);

        uint16_t c;
        for(c = 0; c < ipconfigNETWORK_MTU; c++)
        {
            *pTxData++ = ipconfigRAND32();
        }

        // Start transmission
        if( !xNetworkInterfaceOutput(pTxDescriptor, pdTRUE) )
        {
            return false;
        }

        // Wait until Tx operation completes
        if( !TEST_IF_TRUE_WITH_RETRY((ETHCON1 & _ETHCON1_TXRTS_MASK) == 0, SELF_TEST_RETRY_COUNT) )
        {
            return false;
        }

        // Check transmit successful
        if( (s_pPendingTxDMADescKVA->statusVectorLow & ETH_TSVL_TRANSMIT_DONE) == 0 )
        {
            return false;
        }

        // Wait for packet reception
        if( !TEST_IF_TRUE_WITH_RETRY(ETHIRQ & _ETHIRQ_RXDONE_MASK, SELF_TEST_RETRY_COUNT) )
        {
            return false;
        }

        // Check packet received ok
        if( ((s_pCurrentRxDMADescKVA->statusVectorLow & ETH_RSVL_RX_OK) == 0)
            || (s_pPendingTxDMADescKVA->hdr.Count != (s_pCurrentRxDMADescKVA->hdr.Count - FCS_LENGTH)) )
        {
            return false;
        }

        // Compare data received with what was allegedly transmitted
        const NetworkBufferDescriptor_t *pRxDescriptor = pxPacketBuffer_to_NetworkBuffer( PA_TO_KVA1((uintptr_t) s_pCurrentRxDMADescKVA->pBufferPA) );

        if( memcmp(pTxDescriptor->pucEthernetBuffer, pRxDescriptor->pucEthernetBuffer, s_pPendingTxDMADescKVA->hdr.Count) != 0 )
        {
            return false;
        }

        s_pCurrentRxDMADescKVA = PA_TO_KVA1((uintptr_t) s_pCurrentRxDMADescKVA->pNextDescriptorPA);
        s_pPendingTxDMADescKVA = PA_TO_KVA1((uintptr_t) s_pPendingTxDMADescKVA->pNextDescriptorPA);
    }
    while( --iterations );

    return true;
}
