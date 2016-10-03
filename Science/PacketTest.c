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

// RTOS
#include <FreeRTOS.h>
#include <task.h>
// TCP/IP Stack
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>
#include <NetworkInterface.h>
#include <NetworkBufferManagement.h>
#include <FreeRTOS_IP_Private.h>
// C Runtime
#include <stdio.h>

#define PACKET_TIMER_HZ (5000U)
#define BYPASS_TCPIP_STACK
//#define BUILD_TRANSMITTER

#if defined(__PIC32MZ__)
#define LATxSET LATHSET
#define LATxCLR LATHCLR
#elif defined(__PIC32MX__)
#define LATxSET LATDSET
#define LATxCLR LATDCLR
#endif

void __attribute__((interrupt(IPL0AUTO), vector(_TIMER_3_VECTOR))) vT3InterruptWrapper(void);

static TaskHandle_t s_hPacketTask = NULL;

#if defined(BYPASS_TCPIP_STACK)

static const uint8_t pUDP_PACKET_DEFAULTS[] = {
    // Ethernet II Header
    0x01, 0x00, 0x5E, 0x01, 0x02, 0x03, // xDestinationAddress
    0xD8, 0x80, 0x39, 0x75, 0xAF, 0x8B, // xSourceAddress
    0x08, 0x00,                         // usFrameType
    // IP Header
    0x45,                   // ucVersionHeaderLength
    0x00,                   // ucDifferentiatedServicesCode
    0x00, 0x24,             // usLength
    0x00, 0x00,             // usIdentification
    0x00, 0x00,             // usFragmentOffset
    0x80,                   // ucTimeToLive
    0x11,                   // ucProtocol
    0x44, 0xb1,             // usHeaderChecksum
    0x0a, 0x0a, 0x0a, 0x0a, // ulSourceIPAddress
//  0xe0, 0x01, 0x02, 0x03, // ulDestinationIPAddress
    0x0a, 0x0a, 0x0a, 0x0b, // ulDestinationIPAddress
    // UDP Header
    0x1f, 0xaf, // usSourcePort
    0x30, 0x39, // usDestinationPort
    0x00, 0x10, // usLength
    0x00, 0x00  // usChecksum
};

#endif // BYPASS_TCPIP_STACK

void vT3InterruptHandler(void)
{
    IFS0CLR = _IFS0_T3IF_MASK;
    
    LATxSET = 0x01;
        
    BaseType_t bHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_hPacketTask, &bHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}

#if defined(BUILD_TRANSMITTER)

portTASK_FUNCTION(PacketTask, pParams)
{
    portTASK_USES_FLOATING_POINT();
    
    s_hPacketTask = xTaskGetCurrentTaskHandle();
    
    T3CON = 0;
    TMR3 = 0;

    PR3 = (configPERIPHERAL_CLOCK_HZ / PACKET_TIMER_HZ) - 1;
    
    // Setup timer 2 interrupt priority to be above the kernel priority so
    // the timer jitter is not effected by the kernel activity.
    IPC3bits.T3IP = configKERNEL_INTERRUPT_PRIORITY;

    // Clear the interrupt as a starting condition.
    IFS0bits.T3IF = 0;

    // Enable the interrupt.
    IEC0bits.T3IE = 1;

    // Start the timer.
    T3CONbits.TON = 1;
    
    Socket_t tTxSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
    
    configASSERT(tTxSocket != FREERTOS_INVALID_SOCKET);

    struct freertos_sockaddr tSockAddr;
    memset(&tSockAddr, 0, sizeof(tSockAddr));
    
    tSockAddr.sin_family = FREERTOS_AF_INET;
    tSockAddr.sin_port = FreeRTOS_htons(8111);
    
    BaseType_t result = FreeRTOS_bind(tTxSocket, &tSockAddr, sizeof(tSockAddr));
    
    configASSERT(result == 0);
    
    struct freertos_sockaddr tDestAddr;
    memset(&tDestAddr, 0, sizeof(tDestAddr));
    
    tDestAddr.sin_family = FREERTOS_AF_INET;
//    tDestAddr.sin_addr = FreeRTOS_inet_addr_quick(224, 1, 2, 3);
    tDestAddr.sin_addr = FreeRTOS_inet_addr_quick(10, 10, 10, 11);
    tDestAddr.sin_port = FreeRTOS_htons(12345);
    
    for( ; ; )
    {
#if defined(BYPASS_TCPIP_STACK)
        NetworkBufferDescriptor_t *pUdpBuffer = NULL;
        
        if( FreeRTOS_IsNetworkUp() )
            pUdpBuffer = pxGetNetworkBufferWithDescriptor( sizeof(UDPPacket_t) + 8, 0 );

        if( pUdpBuffer )
        {
            UDPPacket_t *pPacket = (UDPPacket_t *) pUdpBuffer->pucEthernetBuffer;
            memcpy(pPacket, pUDP_PACKET_DEFAULTS, sizeof(*pPacket));

            pPacket->xIPHeader.usHeaderChecksum = 0;
            pPacket->xIPHeader.usHeaderChecksum = usGenerateChecksum(0UL, (uint8_t *) &pPacket->xIPHeader, ipSIZE_OF_IPv4_HEADER);
            pPacket->xIPHeader.usHeaderChecksum = ~FreeRTOS_htons(pPacket->xIPHeader.usHeaderChecksum);
            
            pUdpBuffer->xDataLength = sizeof(UDPPacket_t) + 8;
        }
#else
        uint8_t *pUdpBuffer = FreeRTOS_GetUDPPayloadBuffer(128, 0);
#endif // BYPASS_TCPIP_STACK
        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        LATxCLR = 0x01;
        
        if( pUdpBuffer )
        {
#if defined(BYPASS_TCPIP_STACK)
            LATxSET = 0x02;
            xNetworkInterfaceOutput(pUdpBuffer, pdTRUE);
            LATxCLR = 0x02;
#else
            LATxSET = 0x02;

            if( FreeRTOS_sendto(tTxSocket, pUdpBuffer, sizeof(tSockAddr), FREERTOS_ZERO_COPY, &tDestAddr, sizeof(tDestAddr)) != 0 )
            {
                FreeRTOS_ReleaseUDPPayloadBuffer(pUdpBuffer);
            }

            LATxCLR = 0x02;
#endif   
        }
    }
}

#else

portTASK_FUNCTION(PacketTask, pParams)
{
    portTASK_USES_FLOATING_POINT();

    Socket_t tRxSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
    
    configASSERT(tRxSocket != FREERTOS_INVALID_SOCKET);

    struct freertos_sockaddr tSockAddr;
    memset(&tSockAddr, 0, sizeof(tSockAddr));
    
    tSockAddr.sin_family = FREERTOS_AF_INET;
    tSockAddr.sin_port = FreeRTOS_htons(12345);
    
    BaseType_t result = FreeRTOS_bind(tRxSocket, &tSockAddr, sizeof(tSockAddr));
    
    configASSERT(result == 0);
    
    struct freertos_sockaddr tRxAddr;
    memset(&tRxAddr, 0, sizeof(tRxAddr));
    
    for( ; ; )
    {
        uint8_t *pRxData = NULL;
        socklen_t tRxAddrSize = sizeof(tRxAddr);
        
        LATxCLR = 0x04;
        int32_t result = FreeRTOS_recvfrom(tRxSocket, &pRxData, 0, FREERTOS_ZERO_COPY, &tRxAddr, &tRxAddrSize);
        LATxSET = 0x04;
        
        if(result > 0)
        {
            FreeRTOS_ReleaseUDPPayloadBuffer(pRxData);
        }
    }
}

#endif // BUILD_TRANSMITTER
