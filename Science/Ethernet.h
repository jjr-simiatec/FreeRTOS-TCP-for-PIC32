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

#ifndef PIC32_ETHERNET_H
#define PIC32_ETHERNET_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
    ETH_NORMAL,
    ETH_WAKE_ON_LAN,
    ETH_WAKE_ON_LAN_WOKEN,
    ETH_POWER_DOWN,
    ETH_SELF_TEST
} eth_interface_state_t;

typedef struct {
    uint32_t framesTransmitted;
    uint32_t txFailures;
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
extern void EthernetSelfTest(TaskHandle_t hNotify);

#ifdef	__cplusplus
}
#endif

#endif // PIC32_ETHERNET_H
