/*
 * DP83848 PHY Driver
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

#include "EthernetPrivate.h"
#include "DP83848.h"

extern SemaphoreHandle_t g_hLinkUpSemaphore;

const uint16_t nPHY_ADDRESS = (1 << _EMAC1MADR_PHYADDR_POSITION);

void __attribute__(( interrupt(IPL0AUTO), vector(_EXTERNAL_3_VECTOR) )) PHYInterruptWrapper(void);

void PHYInitialise(void)
{
    PHYWrite(PHY_REG_BASIC_CONTROL, PHY_CTRL_RESET);

    while( PHYRead(PHY_REG_BASIC_CONTROL) & PHY_CTRL_RESET );

    IPC3bits.INT3IP = configKERNEL_INTERRUPT_PRIORITY;

    IFS0CLR = _IFS0_INT3IF_MASK;
    IEC0SET = _IEC0_INT3IE_MASK;

    PHYWrite(DP83848_REG_INTERRUPT_CONTROL, DP83848_MICR_INTERRUPT_ENABLE | DP83848_MICR_INTERRUPT_OUTPUT_ENABLE);
    PHYWrite(DP83848_REG_INTERRUPT_STATUS, DP83848_MISR_ENABLE_AUTO_NEG_COMPLETE_INT | DP83848_MISR_ENABLE_LINK_CHANGE_INT);
}

void PHYDisableInterrupt(void)
{
    IEC0CLR = _IEC0_INT3IE_MASK;
}

void PHYGetStatus(phy_status_t *pStatus)
{
    uint16_t linkStat = PHYRead(DP83848_REG_PHY_STATUS);

    pStatus->speed = linkStat & DP83848_PHYSTS_SPEED_10MBPS ? PHY_SPEED_10MBPS : PHY_SPEED_100MBPS;
    pStatus->fullDuplex = (linkStat & DP83848_PHYSTS_FULL_DUPLEX) != 0;
}

void PHYInterruptHandler(void)
{
    IEC0CLR = _IEC0_INT3IE_MASK;

    BaseType_t bHigherPriorityTaskWoken = pdFALSE;
    
    xTaskNotifyFromISR(g_hEthernetTask, ETH_TASK_PHY_INTERRUPT, eSetBits, &bHigherPriorityTaskWoken);

    portEND_SWITCHING_ISR(bHigherPriorityTaskWoken);
}

void PHYDeferredInterruptHandler(void)
{
    uint16_t intSource = PHYRead(DP83848_REG_INTERRUPT_STATUS);

    IFS0CLR = _IFS0_INT3IF_MASK;

    if(intSource & (DP83848_MISR_AUTO_NEG_COMPLETE | DP83848_MISR_LINK_STATUS_CHANGE))
    {
        uint16_t status = PHYRead(DP83848_REG_PHY_STATUS);

        if(status & DP83848_PHYSTS_LINK_ESTABLISHED)
            xSemaphoreGive(g_hLinkUpSemaphore);
        else
            FreeRTOS_NetworkDown();
    }

    IEC0SET = _IEC0_INT3IE_MASK;
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
