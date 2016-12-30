/*
 * LAN9303 PHY Driver
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
#include <semphr.h>
// TCP/IP Stack
#include <FreeRTOS_IP.h>
#include <FreeRTOS_IP_Private.h>
// C Runtime
#include <stdint.h>
#include <stdbool.h>

#include "PHYGeneric.h"

extern SemaphoreHandle_t g_hLinkUpSemaphore;

const uint16_t nPHY_ADDRESS = 0;

void __attribute__(( interrupt(IPL0AUTO), vector(_EXTERNAL_4_VECTOR) )) PHYInterruptWrapper(void);

void PHYInitialise(void)
{
    LATHSET = _LATH_LATH11_MASK;

    PHYWrite(PHY_REG_BASIC_CONTROL, PHY_CTRL_RESET);

    while( PHYRead(PHY_REG_BASIC_CONTROL) & PHY_CTRL_RESET );

    xSemaphoreGive(g_hLinkUpSemaphore);
}

void PHYGetStatus(phy_status_t *pStatus)
{
//    uint16_t linkStat = PHYRead(DP83848_REG_PHY_STATUS);

    pStatus->speed = PHY_SPEED_100MBPS;
    pStatus->fullDuplex = true;
}

void PHYInterruptHandler(void)
{
    IFS0CLR = _IFS0_INT4IF_MASK;

//    uint16_t intSource = PHYRead(DP83848_REG_INTERRUPT_STATUS);

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;

#if 0
    if(intSource & (DP83848_MISR_AUTO_NEG_COMPLETE | DP83848_MISR_LINK_STATUS_CHANGE))
    {
        uint16_t status = PHYRead(DP83848_REG_PHY_STATUS);

        if(status & DP83848_PHYSTS_LINK_ESTABLISHED)
        {
            xSemaphoreGiveFromISR(g_hLinkUpSemaphore, &bHigherPriorityTaskWoken);
        }
        else
        {
            if( FreeRTOS_NetworkDownFromISR() )
                bHigherPriorityTaskWoken = pdTRUE;
        }
    }
#endif

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
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
