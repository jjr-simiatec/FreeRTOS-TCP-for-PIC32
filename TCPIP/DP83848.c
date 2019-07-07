/*
 * DP83848 PHY Driver
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
#include "DP83848.h"

#if ipconfigPIC32_PHY_DRIVER == PIC32_PHY_DRIVER_DP83848

void PHYInitialise(void)
{
    PHY_WRITE(PHY_REG_BASIC_CONTROL, PHY_CTRL_RESET);

    while( PHY_READ(PHY_REG_BASIC_CONTROL) & PHY_CTRL_RESET );

    ipconfigPIC32_PHY_CLEAR_INTERRUPT();
    ipconfigPIC32_PHY_ENABLE_INTERRUPT();

    PHY_WRITE(DP83848_REG_INTERRUPT_CONTROL, DP83848_MICR_INTERRUPT_ENABLE | DP83848_MICR_INTERRUPT_OUTPUT_ENABLE);
    PHY_WRITE(DP83848_REG_INTERRUPT_STATUS, DP83848_MISR_ENABLE_AUTO_NEG_COMPLETE_INT | DP83848_MISR_ENABLE_LINK_CHANGE_INT);
}

void PHYGetStatus(phy_status_t *pStatus)
{
    uint16_t linkStat = PHY_READ(DP83848_REG_PHY_STATUS);

    pStatus->speed = linkStat & DP83848_PHYSTS_SPEED_10MBPS ? PHY_SPEED_10MBPS : PHY_SPEED_100MBPS;
    pStatus->fullDuplex = (linkStat & DP83848_PHYSTS_FULL_DUPLEX) != 0;
}

void PHYDeferredInterruptHandler(void)
{
    uint16_t intSource = PHY_READ(DP83848_REG_INTERRUPT_STATUS);
    ipconfigPIC32_PHY_CLEAR_INTERRUPT();

    if(intSource & (DP83848_MISR_AUTO_NEG_COMPLETE | DP83848_MISR_LINK_STATUS_CHANGE))
    {
        uint16_t status = PHY_READ(DP83848_REG_PHY_STATUS);

        if(status & DP83848_PHYSTS_LINK_ESTABLISHED)
            xSemaphoreGive(g_hLinkUpSemaphore);
        else
            FreeRTOS_NetworkDown();
    }

    ipconfigPIC32_PHY_ENABLE_INTERRUPT();
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

#endif
