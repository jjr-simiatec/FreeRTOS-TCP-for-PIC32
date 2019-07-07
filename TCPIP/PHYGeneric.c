/*
 * Generic PHY Interface and Definitions
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
#include <semphr.h>
// C Runtime
#include <stdbool.h>

#include "Ethernet.h"
#include "PHYGeneric.h"
#include "EthernetPrivate.h"
#include "PIC32Arch.h"
#include "FreeRTOSIPConfig.h"

#if !defined(ipconfigPIC32_PHY_INTERRUPT_VECTOR)
#error The interrupt vector for the PHY interrupt is not defined
#endif

#if !defined(ipconfigPIC32_PHY_ENABLE_INTERRUPT)
#error Method to enable PHY interrupt is not defined
#endif

#if !defined(ipconfigPIC32_PHY_DISABLE_INTERRUPT)
#error Method to disable PHY interrupt is not defined
#endif

#if !defined(ipconfigPIC32_PHY_CLEAR_INTERRUPT)
#error Method to clear PHY interrupt is not defined
#endif

void __attribute__(( interrupt(IPL0AUTO), vector(ipconfigPIC32_PHY_INTERRUPT_VECTOR) )) PHYInterruptWrapper(void);

uint16_t PHYRead(uint8_t phyaddr, uint8_t reg)
{
    EMAC1MADR = (reg << _EMAC1MADR_REGADDR_POSITION)
                 | (phyaddr << _EMAC1MADR_PHYADDR_POSITION);

    EMAC1MCMD = _EMAC1MCMD_READ_MASK;

    _sync();
    _nop(); _nop(); _nop();

#ifndef __MPLAB_DEBUGGER_SIMULATOR
    while(EMAC1MIND & _EMAC1MIND_MIIMBUSY_MASK);
#endif // __MPLAB_DEBUGGER_SIMULATOR

    EMAC1MCMD = 0;

    return EMAC1MRDD;
}

void PHYWrite(uint8_t phyaddr, uint8_t reg, uint16_t val)
{
    EMAC1MADR = (reg << _EMAC1MADR_REGADDR_POSITION)
                 | (phyaddr << _EMAC1MADR_PHYADDR_POSITION);

    EMAC1MWTD = val;

    _sync();
    _nop(); _nop(); _nop();

#ifndef __MPLAB_DEBUGGER_SIMULATOR
    while(EMAC1MIND & _EMAC1MIND_MIIMBUSY_MASK);
#endif
}

uint16_t PHY_MMDRead(uint8_t phyaddr, uint8_t devad, uint16_t reg)
{
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_ADDRESS | devad);
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_ADDRESS_DATA, reg);
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_DATA | devad);
    return PHYRead(phyaddr, PHY_REG_MMD_ACCESS_ADDRESS_DATA);
}

void PHY_MMDWrite(uint8_t phyaddr, uint8_t devad, uint16_t reg, uint16_t val)
{
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_ADDRESS | devad);
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_ADDRESS_DATA, reg);
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_DATA | devad);
    PHYWrite(phyaddr, PHY_REG_MMD_ACCESS_ADDRESS_DATA, val);
}

void PHYGenericPowerDown(uint8_t phyaddr)
{
    uint16_t bcr = PHYRead(phyaddr, PHY_REG_BASIC_CONTROL);
    PHYWrite(phyaddr, PHY_REG_BASIC_CONTROL, bcr | PHY_CTRL_POWER_DOWN);
}

void PHYInterruptHandler(void)
{
    ipconfigPIC32_PHY_DISABLE_INTERRUPT();

    InterlockedCompareExchange(&g_interfaceState, ETH_WAKE_ON_LAN_WOKEN, ETH_WAKE_ON_LAN);

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(g_hEthernetTask, ETH_TASK_PHY_INTERRUPT, eSetBits, &bHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}
