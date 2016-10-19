/*
 * Generic PHY Interface and Definitions
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
#include <semphr.h>
// C Runtime
#include <stdbool.h>

#include "PHYGeneric.h"

uint16_t PHYRead(uint8_t reg)
{
    EMAC1MADR = reg | nPHY_ADDRESS;
    EMAC1MCMD = _EMAC1MCMD_READ_MASK;

    _sync();
    _nop(); _nop(); _nop();

#ifndef __MPLAB_DEBUGGER_SIMULATOR
    while(EMAC1MIND & _EMAC1MIND_MIIMBUSY_MASK);
#endif // __MPLAB_DEBUGGER_SIMULATOR

    EMAC1MCMD = 0;

    return EMAC1MRDD;
}

void PHYWrite(uint8_t reg, uint16_t val)
{
    EMAC1MADR = reg | nPHY_ADDRESS;
    EMAC1MWTD = val;

    _sync();
    _nop(); _nop(); _nop();

#ifndef __MPLAB_DEBUGGER_SIMULATOR
    while(EMAC1MIND & _EMAC1MIND_MIIMBUSY_MASK);
#endif
}

uint16_t PHY_MMDRead(uint8_t devad, uint16_t reg)
{
    PHYWrite(PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_ADDRESS | devad);
    PHYWrite(PHY_REG_MMD_ACCESS_ADDRESS_DATA, reg);
    PHYWrite(PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_DATA | devad);
    return PHYRead(PHY_REG_MMD_ACCESS_ADDRESS_DATA);
}

void PHY_MMDWrite(uint8_t devad, uint16_t reg, uint16_t val)
{
    PHYWrite(PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_ADDRESS | devad);
    PHYWrite(PHY_REG_MMD_ACCESS_ADDRESS_DATA, reg);
    PHYWrite(PHY_REG_MMD_ACCESS_CONTROL, PHY_MMDCTRL_ACTYPE_DATA | devad);
    PHYWrite(PHY_REG_MMD_ACCESS_ADDRESS_DATA, val);
}
