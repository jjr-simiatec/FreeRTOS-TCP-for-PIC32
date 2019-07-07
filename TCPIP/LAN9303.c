/*
 * LAN9303 PHY Driver
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
#include <semphr.h>
// TCP/IP Stack
#include <FreeRTOS_IP.h>
#include <FreeRTOS_IP_Private.h>
// C Runtime
#include <stdint.h>
#include <stdbool.h>

#include "PHYGeneric.h"
#include "EthernetPrivate.h"
#include "LAN9303.h"

#if ipconfigPIC32_PHY_DRIVER == PIC32_PHY_DRIVER_LAN9303

void PHYInitialise(void)
{
    uint32_t test = 0;

    // Wait for byte order test magic value
    do
    {
        test = LAN9303ReadRegister(LAN9303_REG_BYTE_TEST);
    }
    while(test != LAN9303_BYTE_TEST_MAGIC);

    // Wait for device ready
    do
    {
        test = LAN9303ReadRegister(LAN9303_REG_HW_CFG);
    }
    while((test & LAN9303_HW_CFG_READY) == 0);

    uint32_t leds = LAN9303ReadRegister(LAN9303_REG_LED_CFG);

    leds &= ~LAN9303_LED_CFG_FN_MASK;
    leds |= (LAN9303_LED_CFG_LED0_ENABLE | LAN9303_LED_CFG_LED1_ENABLE
             | LAN9303_LED_CFG_LED3_ENABLE | LAN9303_LED_CFG_LED4_ENABLE
             | LAN9303_LED_CFG_FN_SPD_LNKACT);

    LAN9303WriteRegister(LAN9303_REG_LED_CFG, leds);

    // MAC is connected to the VPHY
    PHY_WRITE(PHY_REG_BASIC_CONTROL, PHY_CTRL_RESET);

    while( PHY_READ(PHY_REG_BASIC_CONTROL) & PHY_CTRL_RESET );

    // Force speed and duplex to max. for the VPHY
    PHY_WRITE(PHY_REG_BASIC_CONTROL, PHY_CTRL_FULL_DUPLEX | PHY_CTRL_SPEED_100MBPS);

    // Link is always up
    xSemaphoreGive(g_hLinkUpSemaphore);
}

void PHYGetStatus(phy_status_t *pStatus)
{
    pStatus->speed = PHY_SPEED_100MBPS;
    pStatus->fullDuplex = true;
}

void PHYDeferredInterruptHandler(void)
{
    // VPHY is always up
}

phy_tdr_state_t PHYCableDiagnostic(phy_tdr_cable_t type, float *pLenEstimate)
{
    return PHY_TDR_STATE_NOT_SUPPORTED;
}

bool PHYSupportsWOL(void)
{
    return false;
}

void PHYPrepareWakeOnLAN(void)
{
}

uint32_t LAN9303ReadRegister(lan9303_register_t reg)
{
    uint8_t phyaddr = (reg >> 6) | 0x10, regaddr = (reg & 0x3C) >> 1;

    uint16_t lsb = PHYRead(phyaddr, regaddr);
    uint16_t msb = PHYRead(phyaddr, regaddr | 1);

    return (msb << 16U) | lsb;
}

void LAN9303WriteRegister(lan9303_register_t reg, uint32_t val)
{
    uint8_t phyaddr = (reg >> 6) | 0x10, regaddr = (reg & 0x3C) >> 1;

    PHYWrite(phyaddr, regaddr, val & 0xFFFFU);
    PHYWrite(phyaddr, regaddr | 1, val >> 16U);
}

#endif
