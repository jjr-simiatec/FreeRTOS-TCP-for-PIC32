/*
 * PIC32 Ethernet Driver for FreeRTOS+TCP
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

#if !defined(ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ)
#define ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ    2500000UL
#endif // ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ

#if !defined(ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ)

#if defined(__PIC32MX__)
#define ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ   configCPU_CLOCK_HZ
#elif defined(__PIC32MZ__)
#define ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ   100000000UL
#endif

#endif // ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ

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
static uint8_t s_tNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS][NET_BUFFER_SIZE_ROUNDED_UP] __attribute((aligned(4), coherent));

static eth_dma_descriptor_t s_tTxDMADescriptors[ipconfigPIC32_TX_DMA_DESCRIPTORS] __attribute__((coherent));
static eth_dma_descriptor_t s_tRxDMADescriptors[ipconfigPIC32_RX_DMA_DESCRIPTORS] __attribute__((coherent));

static eth_dma_descriptor_t *s_pCurrentRxDMADescKVA;
static eth_dma_descriptor_t *s_pCurrentTxDMADescKVA;
static eth_dma_descriptor_t *s_pPendingTxDMADescKVA;

static eth_stats_t s_tStats;

static TaskHandle_t s_hEthernetTask = NULL;
static SemaphoreHandle_t s_hTxDMABufCountSemaphore = NULL;
static SemaphoreHandle_t s_hTxDMABufMutex = NULL;

SemaphoreHandle_t g_hLinkUpSemaphore = NULL;
volatile BaseType_t g_interfaceState = ETH_NORMAL;

portTASK_FUNCTION_PROTO(EthernetTask, pParams);
static void ControllerInitialise(void);
static void InitialiseDMADescriptorLists(void);
static void InitiateWOLandWait(void);
static void PowerdownEthernet(void);
static void ExecuteSelfTests(void);

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
                    pStackRxDescriptor->xDataLength = s_pCurrentRxDMADescKVA->hdr.Count - 4;
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
                s_pCurrentRxDMADescKVA->status = 0;
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
                uint8_t *pBuffer = PA_TO_KVA1((uintptr_t) s_pPendingTxDMADescKVA->pBufferPA);

                s_pPendingTxDMADescKVA->pBufferPA = NULL;
                s_pPendingTxDMADescKVA->status = 0;
                s_pPendingTxDMADescKVA = PA_TO_KVA1((uintptr_t) s_pPendingTxDMADescKVA->pNextDescriptorPA);

                // Release the mutex while the buffer is released and reacquire afterwards
                xSemaphoreGive(s_hTxDMABufMutex);

                xSemaphoreGive(s_hTxDMABufCountSemaphore);
                vReleaseNetworkBufferAndDescriptor( pxPacketBuffer_to_NetworkBuffer(pBuffer) );

                xSemaphoreTake(s_hTxDMABufMutex, portMAX_DELAY);
            }

            xSemaphoreGive(s_hTxDMABufMutex);
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

    xTaskNotifyFromISR(s_hEthernetTask, irqs, eSetBits, &bHigherPriorityTaskWoken);

    CLEAR_INTERRUPT_FLAG();

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}

void ControllerInitialise(void)
{
    /** Ethernet Controller initialisation **/

    // Procedure taken from Microchip document ref. 61155C, section 35.4.10
    DISABLE_INTERRUPT();

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

    /** MAC initialisation **/
    EMAC1CFG1SET = _EMAC1CFG1_SOFTRESET_MASK;
    EMAC1CFG1CLR = _EMAC1CFG1_SOFTRESET_MASK;

    EMAC1SUPPSET = _EMAC1SUPP_RESETRMII_MASK;
    EMAC1SUPPCLR = _EMAC1SUPP_RESETRMII_MASK;

    EMAC1MCFGSET = _EMAC1MCFG_RESETMGMT_MASK;
    EMAC1MCFGCLR = _EMAC1MCFG_RESETMGMT_MASK;

    uint32_t c;
    for(c = 0; c < sizeof(pMIIM_CLOCK_DIVIDERS) / sizeof(pMIIM_CLOCK_DIVIDERS[0]); c++)
    {
        if((ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ / pMIIM_CLOCK_DIVIDERS[c]) <= ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ)
        {
            EMAC1MCFG = (c + 1) << _EMAC1MCFG_CLKSEL_POSITION;
            break;
        }
    }

    while(EMAC1MIND & _EMAC1MIND_MIIMBUSY_MASK);
}

BaseType_t xNetworkInterfaceInitialise(void)
{
    if( !s_hEthernetTask )
    {
        // First time through
        g_hLinkUpSemaphore = xSemaphoreCreateBinary();
        s_hTxDMABufCountSemaphore = xSemaphoreCreateCounting(ipconfigPIC32_TX_DMA_DESCRIPTORS, ipconfigPIC32_TX_DMA_DESCRIPTORS);
        s_hTxDMABufMutex = xSemaphoreCreateMutex();
        configASSERT( g_hLinkUpSemaphore && s_hTxDMABufCountSemaphore && s_hTxDMABufMutex );

        memset(&s_tStats, 0, sizeof(s_tStats));
        memset(s_tTxDMADescriptors, 0, sizeof(s_tTxDMADescriptors));
        memset(s_tRxDMADescriptors, 0, sizeof(s_tRxDMADescriptors));

        xTaskCreate(&EthernetTask, "EthDrv", ipconfigPIC32_DRV_TASK_STACK_SIZE, NULL, ipconfigPIC32_DRV_TASK_PRIORITY, &s_hEthernetTask);
        configASSERT(s_hEthernetTask);
    }
    else
    {
        // Link failed/stack reset
        s_tStats.linkFailures++;
        iptraceNETWORK_DOWN();
    }

    do
    {
        if(g_interfaceState != ETH_NORMAL)
        {
            switch(g_interfaceState)
            {
            case ETH_POWER_DOWN:
                PowerdownEthernet();
                break;

            case ETH_WAKE_ON_LAN:
                InitiateWOLandWait();
                break;

            case ETH_SELF_TEST:
                ExecuteSelfTests();
                break;
            }

            g_interfaceState = ETH_NORMAL;
        }

        ControllerInitialise();
        PHYInitialise();

        xSemaphoreTake(g_hLinkUpSemaphore, portMAX_DELAY);
    }
    while(g_interfaceState != ETH_NORMAL);

    // Configure MAC based on PHY negotiated link parameters
    phy_status_t phyStatus;
    PHYGetStatus(&phyStatus);

    if(phyStatus.speed == PHY_SPEED_100MBPS)
        EMAC1SUPPSET = _EMAC1SUPP_SPEEDRMII_MASK;
    else
        EMAC1SUPPCLR = _EMAC1SUPP_SPEEDRMII_MASK;

    // EMAC1IPGT settings as per the reference manual
    if( phyStatus.fullDuplex )
    {
        EMAC1CFG2SET = _EMAC1CFG2_FULLDPLX_MASK;
        EMAC1IPGT = 0x15;
    }
    else
    {
        EMAC1CFG2CLR = _EMAC1CFG2_FULLDPLX_MASK;
        EMAC1IPGT = 0x12;
    }

    InitialiseDMADescriptorLists();

    ETHHT0 = 0;
    ETHHT1 = 0;

    ETHRXFC = _ETHRXFC_BCEN_MASK | _ETHRXFC_MCEN_MASK | _ETHRXFC_UCEN_MASK | _ETHRXFC_RUNTEN_MASK | _ETHRXFC_CRCOKEN_MASK;

    EMAC1CFG1 = _EMAC1CFG1_RXENABLE_MASK;
    EMAC1CFG2SET = _EMAC1CFG2_PADENABLE_MASK | _EMAC1CFG2_CRCENABLE_MASK;
    EMAC1MAXF = ipTOTAL_ETHERNET_FRAME_SIZE;

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
    size_t c;
    for(c = 0; c < ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS; c++)
    {
        pxNetworkBuffers[c].pucEthernetBuffer = &s_tNetworkBuffers[c][ipBUFFER_PADDING];

        // The need for the following will be removed in a future version of the stack
        *((uintptr_t *) &s_tNetworkBuffers[c][0]) = (uintptr_t) &pxNetworkBuffers[c];
    }
}

void InitialiseDMADescriptorLists(void)
{
    size_t c;
    eth_dma_descriptor_t *p;

    // Rx descriptors list with the Ethernet Controller owning all blocks
    for(c = 0; c < ipconfigPIC32_RX_DMA_DESCRIPTORS; c++)
    {
        p = &s_tRxDMADescriptors[c];

        p->hdr.control = ETH_DESC_EOWN | ETH_DESC_NPV;
        p->status = 0;

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
        p->status = 0;

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
}

bool EthernetPrepareWakeOnLAN(void)
{
    if( (g_interfaceState != ETH_NORMAL) || !PHYSupportsWOL() )
    {
        return false;
    }

    g_interfaceState = ETH_WAKE_ON_LAN;

    if( FreeRTOS_IsNetworkUp() )
        FreeRTOS_NetworkDown();
    else
        xSemaphoreGive(g_hLinkUpSemaphore);

    return true;
}

void InitiateWOLandWait(void)
{
    // Silence the Ethernet controller
    DISABLE_INTERRUPT();

    ETHCON1CLR = _ETHCON1_RXEN_MASK | _ETHCON1_TXRTS_MASK;

    while( ETHSTATbits.RXBUSY || ETHSTATbits.TXBUSY );

    // Arm the PHY and wait for an awakening
    PHYPrepareWakeOnLAN();

    xSemaphoreTake(g_hLinkUpSemaphore, portMAX_DELAY);
}

void PowerdownEthernet(void)
{
    DISABLE_INTERRUPT();

    ETHCON1CLR = _ETHCON1_RXEN_MASK | _ETHCON1_TXRTS_MASK;
    while( ETHSTATbits.RXBUSY || ETHSTATbits.TXBUSY );

    uint16_t bcr = PHYRead(PHY_REG_BASIC_CONTROL);
    PHYWrite(PHY_REG_BASIC_CONTROL, bcr | PHY_CTRL_POWER_DOWN);

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
    if(g_interfaceState != ETH_NORMAL)
    {
        return;
    }

    g_interfaceState = ETH_POWER_DOWN;

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

void EthernetSelfTest(void)
{
    if(g_interfaceState != ETH_NORMAL)
    {
        return;
    }

    g_interfaceState = ETH_SELF_TEST;

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

void ExecuteSelfTests(void)
{
    ControllerInitialise();

    PHYWrite(PHY_REG_BASIC_CONTROL, PHY_CTRL_RESET);
    while( PHYRead(PHY_REG_BASIC_CONTROL) & PHY_CTRL_RESET );

    PHYWrite(PHY_REG_BASIC_CONTROL, PHY_CTRL_SPEED_100MBPS | PHY_CTRL_FULL_DUPLEX | PHY_CTRL_LOOPBACK);

    EMAC1SUPPSET = _EMAC1SUPP_SPEEDRMII_MASK;
    EMAC1CFG2SET = _EMAC1CFG2_FULLDPLX_MASK;
    EMAC1IPGT = 0x15;
}
