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
#include <stdbool.h>

#define PACKET_TIMER_HZ (5000U)
#define SAMPLE_TIMER_HZ (10U)

#if defined(__PIC32MZ__)
#define LATxSET LATHSET
#define LATxCLR LATHCLR
#define LATxINV LATHINV
#elif defined(__PIC32MX__)
#define LATxSET LATDSET
#define LATxCLR LATDCLR
#define LATxINV LATDINV
#endif

#if !defined(__32MZ2048EFM144__)
#define BUILD_TRANSMITTER
//#define BYPASS_TCPIP_STACK
#endif

void __attribute__((interrupt(IPL0AUTO), vector(_TIMER_3_VECTOR))) vT3InterruptWrapper(void);

#if defined(BUILD_TRANSMITTER)
static TaskHandle_t s_hPacketTask = NULL;
//#define DESTINATION_ADDR FreeRTOS_inet_addr_quick(224, 1, 2, 3)
#define DESTINATION_ADDR FreeRTOS_inet_addr_quick(172, 16, 1, 201)
#define SOURCE_PORT FreeRTOS_htons(8111)
#else
static uint32_t s_packetCount = 0;
static uint32_t s_packetsSec = 0;
static uint8_t s_sampleCount = 0;
#endif

#define DESTINATION_PORT FreeRTOS_htons(12345)

#define PAYLOAD_SIZE    8U

static inline bool MulticastMACFromIPv4Addr(uint32_t ipv4addr, MACAddress_t *pMAC)
{
    // Is this a multicast address?
    if((ipv4addr & 0x000000F0) != 0x000000E0)
    {
        return false;
    }

    pMAC->ucBytes[0] = 0x01; pMAC->ucBytes[1] = 0x00; pMAC->ucBytes[2] = 0x5E;
    pMAC->ucBytes[3] = (ipv4addr >> 8) & 0x7F;
    pMAC->ucBytes[4] = (ipv4addr >> 16) & 0xFF;
    pMAC->ucBytes[5] = ipv4addr >> 24;

    return true;
}

void vT3InterruptHandler(void)
{
    IFS0CLR = _IFS0_T3IF_MASK;

#if defined(BUILD_TRANSMITTER)
    LATxSET = 0x01;

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_hPacketTask, &bHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
#else
    s_sampleCount++;

    if(s_sampleCount < SAMPLE_TIMER_HZ)
    {
        return;
    }

    s_packetsSec = s_packetCount;
    s_packetCount = 0;
    s_sampleCount = 0;
#endif
}

void Toggle5kHzTraffic(void)
{
#if defined(BUILD_TRANSMITTER)
    T3CONINV = _T3CON_ON_MASK;
    LATxCLR = 0x07;

    printf("\r\nTransmitter is %s.", T3CONbits.TON ? "running" : "inactive");
#endif
}

BaseType_t CLIToggle5kHzTraffic(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
#if defined(BUILD_TRANSMITTER)
    T3CONINV = _T3CON_ON_MASK;
    LATxCLR = 0x07;

    snprintf(pcWriteBuffer, xWriteBufferLen, "Transmitter is %s.", T3CONbits.TON ? "running" : "inactive");
#else
    snprintf(pcWriteBuffer, xWriteBufferLen, "Not implemented.");
#endif

    return pdFALSE;
}

BaseType_t CLIRxPacketsSec(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
#if defined(BUILD_TRANSMITTER)
    snprintf(pcWriteBuffer, xWriteBufferLen, "Not implemented.");
#else
    snprintf(pcWriteBuffer, xWriteBufferLen, "Rx packets/sec=%u.", s_packetsSec);
#endif

    return pdFALSE;
}

bool PacketTaskTransmitterRunning(void)
{
#if defined(BUILD_TRANSMITTER)
    return (T3CON & _T3CON_ON_MASK) != 0;
#else
    return false;
#endif
}

#if defined(BUILD_TRANSMITTER)

portTASK_FUNCTION(PacketTask, pParams)
{
    portTASK_USES_FLOATING_POINT();

    s_hPacketTask = xTaskGetCurrentTaskHandle();

    T3CON = 0;
    TMR3 = 0;

    PR3 = (configPERIPHERAL_CLOCK_HZ / PACKET_TIMER_HZ) - 1;

    // Set the timer interrupt priority to be above the kernel priority so
    // the timer is not effected by the kernel activity
    IPC3bits.T3IP = configMAX_SYSCALL_INTERRUPT_PRIORITY;

    // Prepare for interrupts
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;

    // Wait for the link to come up
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if !defined(BYPASS_TCPIP_STACK)
    Socket_t tTxSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    configASSERT(tTxSocket != FREERTOS_INVALID_SOCKET);

    struct freertos_sockaddr tSockAddr;
    memset(&tSockAddr, 0, sizeof(tSockAddr));

    tSockAddr.sin_family = FREERTOS_AF_INET;
    tSockAddr.sin_port = SOURCE_PORT;

    BaseType_t result = FreeRTOS_bind(tTxSocket, &tSockAddr, sizeof(tSockAddr));

    configASSERT(result == 0);

    struct freertos_sockaddr tDestAddr;
    memset(&tDestAddr, 0, sizeof(tDestAddr));

    tDestAddr.sin_family = FREERTOS_AF_INET;
    tDestAddr.sin_addr = DESTINATION_ADDR;
    tDestAddr.sin_port = DESTINATION_PORT;
#endif // BYPASS_TCPIP_STACK

    for( ; ; )
    {
#if defined(BYPASS_TCPIP_STACK)
        NetworkBufferDescriptor_t *pUdpBuffer = NULL;

        if( FreeRTOS_IsNetworkUp() )
            pUdpBuffer = pxGetNetworkBufferWithDescriptor( sizeof(UDPPacket_t) + 8, 0 );

        if( pUdpBuffer )
        {
            UDPPacket_t *pPacket = (UDPPacket_t *) pUdpBuffer->pucEthernetBuffer;

            MulticastMACFromIPv4Addr(DESTINATION_ADDR, &pPacket->xEthernetHeader.xDestinationAddress);
            memcpy( &pPacket->xEthernetHeader.xSourceAddress, &xDefaultPartUDPPacketHeader, sizeof(xDefaultPartUDPPacketHeader) );

            // IP header
            pPacket->xIPHeader.usLength = FreeRTOS_htons( sizeof(pPacket->xIPHeader) + sizeof(pPacket->xUDPHeader) + PAYLOAD_SIZE );
            pPacket->xIPHeader.ulDestinationIPAddress = DESTINATION_ADDR;
            pPacket->xIPHeader.usHeaderChecksum = 0;

            pPacket->xIPHeader.usHeaderChecksum = usGenerateChecksum(0UL, (uint8_t *) &pPacket->xIPHeader, sizeof(pPacket->xIPHeader));
            pPacket->xIPHeader.usHeaderChecksum = ~FreeRTOS_htons(pPacket->xIPHeader.usHeaderChecksum);

            // UDP header
            pPacket->xUDPHeader.usSourcePort = SOURCE_PORT;
            pPacket->xUDPHeader.usDestinationPort = DESTINATION_PORT;
            pPacket->xUDPHeader.usLength = FreeRTOS_htons( sizeof(pPacket->xUDPHeader) + PAYLOAD_SIZE );
            pPacket->xUDPHeader.usChecksum = 0;

            pUdpBuffer->xDataLength = sizeof(*pPacket) + PAYLOAD_SIZE;
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

    T3CON = (0b111 << _T3CON_TCKPS_POSITION);
    TMR3 = 0;

    PR3 = ((configPERIPHERAL_CLOCK_HZ / 256) / SAMPLE_TIMER_HZ) - 1;

    // Set the timer interrupt priority to be above the kernel priority so
    // the timer is not effected by the kernel activity
    IPC3bits.T3IP = configMAX_SYSCALL_INTERRUPT_PRIORITY;

    // Prepare for interrupts
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;

    // Wait for the link to come up
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    T3CONSET = _T3CON_ON_MASK;

    Socket_t tRxSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    configASSERT(tRxSocket != FREERTOS_INVALID_SOCKET);

    struct freertos_sockaddr tSockAddr;
    memset(&tSockAddr, 0, sizeof(tSockAddr));

    tSockAddr.sin_family = FREERTOS_AF_INET;
    tSockAddr.sin_port = DESTINATION_PORT;

    BaseType_t result = FreeRTOS_bind(tRxSocket, &tSockAddr, sizeof(tSockAddr));

    configASSERT(result == 0);

    struct freertos_sockaddr tRxAddr;
    memset(&tRxAddr, 0, sizeof(tRxAddr));

    for( ; ; )
    {
        uint8_t *pRxData = NULL;
        socklen_t tRxAddrSize = sizeof(tRxAddr);

        LATxCLR = 0x02;
        int32_t result = FreeRTOS_recvfrom(tRxSocket, &pRxData, 0, FREERTOS_ZERO_COPY, &tRxAddr, &tRxAddrSize);

        if(result > 0)
        {
            LATxSET = 0x02;
            s_packetCount++;
            FreeRTOS_ReleaseUDPPayloadBuffer(pRxData);
        }
    }
}

#endif // BUILD_TRANSMITTER
