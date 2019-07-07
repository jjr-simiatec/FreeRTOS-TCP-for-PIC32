/*
 * PHY PKOB Specific Definitions
 *
 * Copyright (c) 2017-2019 John Robertson
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

#ifndef PHY_HARDWARE_PROFILE_H
#define PHY_HARDWARE_PROFILE_H

#if defined(__PIC32MX__)

#define ipconfigPIC32_PHY_DRIVER                PIC32_PHY_DRIVER_DP83848

#define ipconfigPIC32_PHY_ADDRESS               1

#define ipconfigPIC32_PHY_ASSERT_HW_RESET()
#define ipconfigPIC32_PHY_CLEAR_HW_RESET()
#define ipconfigPIC32_PHY_ENABLE_INTERRUPT()    IEC0SET = _IEC0_INT3IE_MASK
#define ipconfigPIC32_PHY_DISABLE_INTERRUPT()   IEC0CLR = _IEC0_INT3IE_MASK
#define ipconfigPIC32_PHY_CLEAR_INTERRUPT()     IFS0CLR = _IFS0_INT3IF_MASK

#define ipconfigPIC32_PHY_INTERRUPT_VECTOR      _EXTERNAL_3_VECTOR

#elif defined(__PIC32MZ__)

#if (__PIC32_FEATURE_SET0 == 'E')

#define ipconfigPIC32_PHY_DRIVER                PIC32_PHY_DRIVER_LAN8740A
//#define ipconfigPIC32_PHY_DRIVER                PIC32_PHY_DRIVER_LAN9303

#define ipconfigPIC32_PHY_ADDRESS               0

#define ipconfigPIC32_PHY_ASSERT_HW_RESET()     LATHCLR = _LATH_LATH11_MASK
#define ipconfigPIC32_PHY_CLEAR_HW_RESET()      LATHSET = _LATH_LATH11_MASK
#define ipconfigPIC32_PHY_ENABLE_INTERRUPT()    IEC0SET = _IEC0_INT4IE_MASK
#define ipconfigPIC32_PHY_DISABLE_INTERRUPT()   IEC0CLR = _IEC0_INT4IE_MASK
#define ipconfigPIC32_PHY_CLEAR_INTERRUPT()     IFS0CLR = _IFS0_INT4IF_MASK

#define ipconfigPIC32_PHY_INTERRUPT_VECTOR      _EXTERNAL_4_VECTOR

#elif (__PIC32_FEATURE_SET0 == 'D')

#define ipconfigPIC32_PHY_DRIVER                PIC32_PHY_DRIVER_LAN8740A

#define ipconfigPIC32_PHY_ADDRESS               0

#define ipconfigPIC32_PHY_ASSERT_HW_RESET()     LATJCLR = _LATJ_LATJ15_MASK
#define ipconfigPIC32_PHY_CLEAR_HW_RESET()      LATJSET = _LATJ_LATJ15_MASK
#define ipconfigPIC32_PHY_ENABLE_INTERRUPT()    IEC3SET = _IEC3_CNBIE_MASK
#define ipconfigPIC32_PHY_DISABLE_INTERRUPT()   IEC3CLR = _IEC3_CNBIE_MASK
#define ipconfigPIC32_PHY_CLEAR_INTERRUPT()     {CNFBCLR = _CNFB_CNFB11_MASK; IFS3CLR = _IFS3_CNBIF_MASK;}

#define ipconfigPIC32_PHY_INTERRUPT_VECTOR      _CHANGE_NOTICE_B_VECTOR

#endif

#endif

#endif // PHY_HARDWARE_PROFILE_H
