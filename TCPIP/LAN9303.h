/*
 * LAN9303 PHY Driver
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

#ifndef LAN_9303_H
#define LAN_9303_H

typedef enum {
    LAN9303_PHYADDR_VPHY,
    LAN9303_PHYADDR_PORT1,
    LAN9303_PHYADDR_PORT2
} lan9303_phy_addr_indx_t;

typedef enum {
    LAN9303_REG_ID_REV = 0x050,
    LAN9303_REG_IRQ_CFG = 0x054,
    LAN9303_REG_INT_STS = 0x058,
    LAN9303_REG_INT_EN = 0x05C,
    LAN9303_REG_BYTE_TEST = 0x064,
    LAN9303_REG_HW_CFG = 0x074,
    LAN9303_REG_GPT_CFG = 0x08C,
    LAN9303_REG_GPT_CNT = 0x090,
    LAN9303_REG_FREE_RUN = 0x09C,
    LAN9303_REG_PMI_DATA = 0x0A4,
    LAN9303_REG_PMI_ACCESS = 0x0A8,
    LAN9303_REG_MANUAL_FC_1 = 0x1A0,
    LAN9303_REG_MANUAL_FC_2 = 0x1A4,
    LAN9303_REG_MANUAL_FC_0 = 0x1A8,
    LAN9303_REG_SWITCH_CSR_DATA = 0x1AC,
    LAN9303_REG_SWITCH_CSR_CMD = 0x1B0,
    LAN9303_REG_E2P_CMD = 0x1B4,
    LAN9303_REG_E2P_DATA = 0x1B8,
    LAN9303_REG_LED_CFG = 0x1BC
} lan9303_register_t;

// LAN9303_REG_BYTE_TEST
#define LAN9303_BYTE_TEST_MAGIC     0x87654321UL

// LAN9303_REG_HW_CFG
typedef enum {
    LAN9303_HW_CFG_READY = 1 << 27,
    LAN9303_HW_CFG_AMDIX_EN_STRAP_PORT2 = 1 << 26,
    LAN9303_HW_CFG_AMDIX_EN_STRAP_PORT1 = 1 << 25
} lan9303_hw_cfg_t;

// LAN9303_REG_LED_CFG
typedef enum {
    LAN9303_LED_CFG_FN_SPD_FDCOL   = 0x00 << 8,
    LAN9303_LED_CFG_FN_10ACT_FDCOL = 0x01 << 8,
    LAN9303_LED_CFG_FN_SPD_LNKACT  = 0x02 << 8,
    LAN9303_LED_CFG_FN_RXDV_TXEN   = 0x03 << 8,
    LAN9303_LED_CFG_FN_MASK = 0x03 << 8,
    LAN9303_LED_CFG_LED5_ENABLE = 1 << 5,
    LAN9303_LED_CFG_LED4_ENABLE = 1 << 4,
    LAN9303_LED_CFG_LED3_ENABLE = 1 << 3,
    LAN9303_LED_CFG_LED2_ENABLE = 1 << 2,
    LAN9303_LED_CFG_LED1_ENABLE = 1 << 1,
    LAN9303_LED_CFG_LED0_ENABLE = 1 << 0
} lan9303_led_cfg_t;

extern uint32_t LAN9303ReadRegister(lan9303_register_t reg);
extern void LAN9303WriteRegister(lan9303_register_t reg, uint32_t val);

#endif // LAN_9303_H
