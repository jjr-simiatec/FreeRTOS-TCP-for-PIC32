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

#ifndef ETHERNET_H
#define ETHERNET_H

typedef enum {
    ETH_NORMAL,
    ETH_WAKE_ON_LAN,
    ETH_WAKE_ON_LAN_WOKEN,
    ETH_POWER_DOWN,
    ETH_SELF_TEST
} eth_interface_state_t;

typedef enum {
    ETH_DESC_EOWN = 1UL << 7,
    ETH_DESC_NPV = 1UL << 8,
    ETH_DESC_EOP = 1UL << 30,
    ETH_DESC_SOP = 1UL << 31,
    ETH_DESC_COUNT_SHIFT = 16,
    ETH_DESC_COUNT_MASK = 0x07FF0000UL
} eth_dma_descriptor_control_bits_t;

typedef struct __attribute__((packed, aligned(4))) {
    // Volatile members can be updated by the Ethernet controller
    // Members ending in PA are physical addresses; assign using KVA_TO_PA
    volatile union {
        struct {
            unsigned : 7;
            unsigned EOWN : 1;
            unsigned NPV : 1;
            unsigned : 7;
            unsigned Count : 11;
            unsigned : 3;
            unsigned EOP : 1;
            unsigned SOP : 1;
        };

        uint32_t control;
    } hdr;

    uint8_t *pBufferPA;
    volatile uint64_t status;
    struct eth_dma_descriptor_t *pNextDescriptorPA;
} eth_dma_descriptor_t;

typedef struct {
    uint32_t framesTransmitted;
    uint32_t framesReceived;
    uint32_t singleCollisions;
    uint32_t multipleCollisions;
    uint32_t alignmentErrors;
    uint32_t fcsErrors;
    uint32_t receiveOverflows;
    uint32_t rxNoBuffers;
    uint32_t txNoDMADescriptors;
    uint32_t linkFailures;
} eth_stats_t;

extern void EthernetGetStats(eth_stats_t *pStats);
extern void EthernetResetStats(void);
extern eth_interface_state_t EthernetGetInterfaceState(void);
extern bool EthernetPrepareWakeOnLAN(void);
extern void EthernetInterfaceUp(void);
extern void EthernetInterfaceDown(void);

#endif // ETHERNET_H
