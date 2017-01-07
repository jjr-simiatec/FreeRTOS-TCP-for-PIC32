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

#ifndef PIC32_ETHERNET_PRIVATE_H
#define PIC32_ETHERNET_PRIVATE_H

#ifdef	__cplusplus
extern "C" {
#endif

#define FCS_LENGTH  (4U)

typedef enum {
    ETH_DESC_EOWN = 1UL << 7,
    ETH_DESC_NPV = 1UL << 8,
    ETH_DESC_EOP = 1UL << 30,
    ETH_DESC_SOP = 1UL << 31,
    ETH_DESC_COUNT_SHIFT = 16U,
    ETH_DESC_COUNT_MASK = 0x07FFUL << ETH_DESC_COUNT_SHIFT
} eth_dma_descriptor_control_bits_t;

typedef enum {
    ETH_TSVL_BYTE_COUNT_SHIFT = 0,
    ETH_TSVL_BYTE_COUNT_MASK = 0xFFFFUL << ETH_TSVL_BYTE_COUNT_SHIFT,
    ETH_TSVL_COLLISION_COUNT_SHIFT = 16U,
    ETH_TSVL_COLLISION_COUNT_MASK = 0x0FUL << ETH_TSVL_COLLISION_COUNT_SHIFT,
    ETH_TSVL_CRC_ERROR = 1UL << 20,
    ETH_TSVL_LENGTH_CHECK_ERROR = 1UL << 21,
    ETH_TSVL_LENGTH_OUT_OF_RANGE = 1UL << 22,
    ETH_TSVL_TRANSMIT_DONE = 1UL << 23,
    ETH_TSVL_MULTICAST = 1UL << 24,
    ETH_TSVL_BROADCAST = 1UL << 25,
    ETH_TSVL_DEFERRED = 1UL << 26,
    ETH_TSVL_EXCESSIVE_DEFER = 1UL << 27,
    ETH_TSVL_MAX_COLLISION = 1UL << 28,
    ETH_TSVL_LATE_COLLISION = 1UL << 29,
    ETH_TSVL_GIANT = 1UL << 30,
    ETH_TSVL_UNDERRUN = 1UL << 31
} eth_status_vector_tx_low_t;

typedef enum {
    ETH_TSVH_TOTAL_BYTES_TXED_SHIFT = 0,
    ETH_TSVH_TOTAL_BYTES_TXED_MASK = 0xFFFFUL << ETH_TSVH_TOTAL_BYTES_TXED_SHIFT,
    ETH_TSVH_CONTROL_FRAME = 1UL << 16,
    ETH_TSVH_PAUSE_FRAME = 1UL << 17,
    ETH_TSVH_BACK_PRESSURE = 1UL << 18,
    ETH_TSVH_VLAN_FRAME = 1UL << 19
} eth_status_vector_tx_high_t;

typedef enum {
    ETH_RSVL_BYTE_COUNT_SHIFT = 0,
    ETH_RSVL_BYTE_COUNT_MASK = 0xFFFFUL << ETH_RSVL_BYTE_COUNT_SHIFT,
    ETH_RSVL_LONG_DROP_EVENT = 1UL << 16,
    ETH_RSVL_PREVIOUSLY_SEEN = 1UL << 17,
    ETH_RSVL_CARRIER_EVENT = 1UL << 18,
    ETH_RSVL_RX_CODE_VIOLATION = 1UL << 19,
    ETH_RSVL_CRC_ERROR = 1UL << 20,
    ETH_RSVL_LENGTH_CHECK_ERROR = 1UL << 21,
    ETH_RSVL_LENGTH_OUT_OF_RANGE = 1UL << 22,
    ETH_RSVL_RX_OK = 1UL << 23,
    ETH_RSVL_IS_MULTICAST = 1UL << 24,
    ETH_RSVL_IS_BROADCAST = 1UL << 25,
    ETH_RSVL_DRIBBLE_NIBBLE = 1UL << 26,
    ETH_RSVL_CONTROL_FRAME = 1UL << 27,
    ETH_RSVL_PAUSE_FRAME = 1UL << 28,
    ETH_RSVL_UNKNOWN_OP_CODE = 1UL << 29,
    ETH_RSVL_VLAN_FRAME = 1UL << 30
} eth_status_vector_rx_low_t;

typedef enum {
    ETH_RSVH_CHECKSUM_SHIFT = 0,
    ETH_RSVH_CHECKSUM_MASK = 0xFFFFUL << ETH_RSVH_CHECKSUM_SHIFT,
    ETH_RSVH_RXF_RUNT_PACKET = 1UL << 24,
    ETH_RSVH_RXF_NOT_UNI_MULTI_CAST = 1UL << 25,
    ETH_RSVH_RXF_HASH_TABLE = 1UL << 26,
    ETH_RSVH_RXF_MAGIC_PACKET = 1UL << 27,
    ETH_RSVH_RXF_PATTERN_MATCH = 1UL << 28,
    ETH_RSVH_RXF_UNICAST = 1UL << 29,
    ETH_RSVH_RXF_BROADCAST = 1UL << 30,
    ETH_RSVH_RXF_MULTICAST = 1UL << 31
} eth_status_vector_rx_high_t;

typedef enum {
    ETH_TASK_PHY_INTERRUPT = 1UL << 24
} eth_task_notification_t;

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
    volatile uint32_t statusVectorHigh;
    volatile uint32_t statusVectorLow;
    struct eth_dma_descriptor_t *pNextDescriptorPA;
} eth_dma_descriptor_t;

extern TaskHandle_t g_hEthernetTask;
extern volatile BaseType_t g_interfaceState;
extern SemaphoreHandle_t g_hLinkUpSemaphore;

#ifdef	__cplusplus
}
#endif

#endif // PIC32_ETHERNET_PRIVATE_H
