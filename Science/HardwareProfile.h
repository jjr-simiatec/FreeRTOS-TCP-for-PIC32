/*
 * Shared Board Specific Definitions
 *
 * Copyright (c) 2017 John Robertson
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

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#if defined(__PIC32MX__)

#define PHY_ASSERT_HW_RESET()
#define PHY_CLEAR_HW_RESET()
#define PHY_ENABLE_INTERRUPT()  IEC0SET = _IEC0_INT3IE_MASK
#define PHY_DISABLE_INTERRUPT() IEC0CLR = _IEC0_INT3IE_MASK
#define PHY_CLEAR_INTERRUPT()   IFS0CLR = _IFS0_INT3IF_MASK

#elif defined(__PIC32MZ__)

#if (__PIC32_FEATURE_SET0 == 'E')

#define PHY_ASSERT_HW_RESET()   LATHCLR = _LATH_LATH11_MASK
#define PHY_CLEAR_HW_RESET()    LATHSET = _LATH_LATH11_MASK
#define PHY_ENABLE_INTERRUPT()  IEC0SET = _IEC0_INT4IE_MASK
#define PHY_DISABLE_INTERRUPT() IEC0CLR = _IEC0_INT4IE_MASK
#define PHY_CLEAR_INTERRUPT()   IFS0CLR = _IFS0_INT4IF_MASK

#elif (__PIC32_FEATURE_SET0 == 'D')

#define PHY_ASSERT_HW_RESET()   LATJCLR = _LATJ_LATJ15_MASK
#define PHY_CLEAR_HW_RESET()    LATJSET = _LATJ_LATJ15_MASK
#define PHY_ENABLE_INTERRUPT()  CNNEBSET = _CNNEB_CNNEB11_MASK
#define PHY_DISABLE_INTERRUPT() CNNEBCLR = _CNNEB_CNNEB11_MASK
#define PHY_CLEAR_INTERRUPT()   {CNFBCLR = _CNFB_CNFB11_MASK; IFS3CLR = _IFS3_CNBIF_MASK;}

#endif

#endif

#if defined(PHY_LAN8740A)

#define PHY_ADDRESS     0

#elif defined(PHY_LAN9303)

#define PHY_ADDRESS     0

#elif defined(PHY_DP83848)

#define PHY_ADDRESS     1

#else

#error No PHY type defined!

#endif

#endif // HARDWARE_PROFILE_H
