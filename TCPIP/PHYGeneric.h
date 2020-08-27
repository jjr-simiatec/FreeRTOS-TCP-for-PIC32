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

#ifndef PHYGENERIC_H
#define	PHYGENERIC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define PIC32_PHY_DRIVER_DP83848    1
#define PIC32_PHY_DRIVER_LAN8740A   2
#define PIC32_PHY_DRIVER_LAN9303    3

typedef enum {
    // Basic
    PHY_REG_BASIC_CONTROL = 0,
    PHY_REG_BASIC_STATUS,
    // Extended
    PHY_REG_IDENTIFIER_1,
    PHY_REG_IDENTIFIER_2,
    PHY_REG_AUTO_NEG_ADVERTISEMENT,
    PHY_REG_AUTO_NEG_PARTNER_ABILITY,
    PHY_REG_AUTO_NEG_EXPANSION,
    PHY_REG_AUTO_NEG_NEXT_PAGE_TX,
    PHY_REG_AUTO_NEG_NEXT_PAGE_RX,
    PHY_REG_MMD_ACCESS_CONTROL = 13,
    PHY_REG_MMD_ACCESS_ADDRESS_DATA,
} phy_register_t;

// PHY_REG_BASIC_CONTROL
typedef enum {
    PHY_CTRL_RESET = 1 << 15,
    PHY_CTRL_LOOPBACK = 1 << 14,
    PHY_CTRL_SPEED_100MBPS = 1 << 13,
    PHY_CTRL_AUTO_NEGOTIATE = 1 << 12,
    PHY_CTRL_POWER_DOWN = 1 << 11,
    PHY_CTRL_ISOLATE = 1 << 10,
    PHY_CTRL_RESTART_NEGOTIATION = 1 << 9,
    PHY_CTRL_FULL_DUPLEX = 1 << 8,
    PHY_CTRL_COLLISION_TEST = 1 << 7
} phy_basic_control_t;

// PHY_REG_BASIC_STATUS
typedef enum {
    PHY_STAT_100BASE_T4 = 1 << 15,
    PHY_STAT_100BASE_TX_FULL_DUPLEX = 1 << 14,
    PHY_STAT_100BASE_TX_HALF_DUPLEX = 1 << 13,
    PHY_STAT_10BASE_T_FULL_DUPLEX = 1 << 12,
    PHY_STAT_10BASE_T_HALF_DUPLEX = 1 << 11,
    PHY_STAT_100BASE_T2_FULL_DUPLEX = 1 << 10,
    PHY_STAT_100BASE_T2_HALF_DUPLEX = 1 << 9,
    PHY_STAT_EXTENDED_STATUS = 1 << 8,
    PHY_STAT_MF_PREAMBLE_SUPPRESSION = 1 << 6,
    PHY_STAT_AUTO_NEGOTIATION_COMPLETE = 1 << 5,
    PHY_STAT_REMOTE_FAULT = 1 << 4,
    PHY_STAT_CAN_AUTO_NEGOTIATE = 1 << 3,
    PHY_STAT_LINK_IS_UP = 1 << 2,
    PHY_STAT_JABBER_DETECTED = 1 << 1,
    PHY_STAT_EXTENDED_CAPABILITIES = 1 << 0
} phy_basic_status_t;

// PHY_REG_AUTO_NEG_ADVERTISEMENT
// PHY_REG_AUTO_NEG_PARTNER_ABILITY
typedef enum {
    PHY_AN_ADV_LPA_NEXT_PAGE = 1 << 15,
    PHY_AN_ADV_LPA_REMOTE_FAULT = 1 << 13,
    PHY_AN_ADV_LPA_ASYMMETRIC_PAUSE = 1 << 11,
    PHY_AN_ADV_LPA_SYMMETRIC_PAUSE = 1 << 10,
    PHY_AN_ADV_LPA_100BASE_T4 = 1 << 9,
    PHY_AN_ADV_LPA_100BASE_TX_FULL_DUPLEX = 1 << 8,
    PHY_AN_ADV_LPA_100BASE_TX_HALF_DUPLEX = 1 << 7,
    PHY_AN_ADV_LPA_10BASE_T_FULL_DUPLEX = 1 << 6,
    PHY_AN_ADV_LPA_10BASE_T_HALF_DUPLEX = 1 << 5,
    PHY_AN_ADV_LPA_SELECTOR_FIELD_MASK = 0x1F,
    PHY_AN_ADV_LPA_SELECTOR_IEEE_802_3 = 0x01
} phy_auto_neg_common_t;

// PHY_REG_AUTO_NEG_PARTNER_ABILITY only
typedef enum {
    PHY_AN_LPA_ACKNOWLEDGE = 1 << 14,
} phy_auto_neg_partner_ability_t;

// PHY_REG_AUTO_NEG_EXPANSION
typedef enum {
    PHY_AN_EXP_NP_STORE_BIT_VALID = 1 << 6,
    PHY_AN_EXP_NP_STORE_AN_NPRX = 1 << 5,
    PHY_AN_EXP_PARALLEL_DETECT_FAULT = 1 << 4,
    PHY_AN_EXP_LINK_PARTNER_NP_CAPABLE = 1 << 3,
    PHY_AN_EXP_LOCAL_NP_CAPABLE = 1 << 2,
    PHY_AN_EXP_PAGE_RECEIVED = 1 << 1,
    PHY_AN_EXP_LINK_PARTNER_CAN_AUTO_NEG = 1 << 0
} phy_auto_neg_expansion_t;

// PHY_REG_AUTO_NEG_NEXT_PAGE_TX
// PHY_REG_AUTO_NEG_NEXT_PAGE_RX
typedef enum {
    PHY_AN_NP_NEXT_PAGE = 1 << 15,
    PHY_AN_NP_MESSAGE_PAGE = 1 << 13,
    PHY_AN_NP_ACKNOWLEDGE_2 = 1 << 12,
    PHY_AN_NP_TOGGLE = 1 << 11,
    PHY_AN_NP_MCF_MASK = 0x07FF
} phy_auto_neg_next_page_t;

typedef enum {
    PHY_AN_NPRX_ACNOWLEDGE = 1 << 14
} phy_auto_neg_next_page_rx_t;

// PHY_REG_MMD_ACCESS_CONTROL
typedef enum {
    PHY_MMDCTRL_ACTYPE_ADDRESS = 0 << 14,
    PHY_MMDCTRL_ACTYPE_DATA = 1 << 14,
    PHY_MMDCTRL_ACTYPE_DATA_PI = 2 << 14,
    PHY_MMDCTRL_ACTYPE_DATA_PIWR = 3 << 14
} phy_mmd_access_ctrl_t;

// Generic interface
typedef enum {
    PHY_SPEED_INVALID,
    PHY_SPEED_10MBPS,
    PHY_SPEED_100MBPS
} phy_speed_t;

typedef struct {
    phy_speed_t speed;
    bool fullDuplex;
} phy_status_t;

typedef enum {
    PHY_WOL_INACTIVE,
    PHY_WOL_INITIALISING,
    PHY_WOL_ARMED
} phy_wol_state_t;

typedef enum {
    PHY_TDR_CABLE_UNKNOWN,
    PHY_TDR_CABLE_CAT5,
    PHY_TDR_CABLE_CAT5e,
    PHY_TDR_CABLE_CAT6,
    _PHY_TDR_CABLE_TYPES
} phy_tdr_cable_t;

typedef enum {
    PHY_TDR_STATE_NOT_SUPPORTED = -100,
    PHY_TDR_STATE_ERROR,
    PHY_TDR_STATE_GOOD = 0,
    PHY_TDR_STATE_FAILED,
    PHY_TDR_STATE_SHORTED,
    PHY_TDR_STATE_OPEN
} phy_tdr_state_t;

#define PHY_READ(reg)           PHYRead(ipconfigPIC32_PHY_ADDRESS, reg)
#define PHY_WRITE(reg, val)     PHYWrite(ipconfigPIC32_PHY_ADDRESS, reg, val)

#define PHY_MMD_READ(devad, reg)        PHY_MMDRead(ipconfigPIC32_PHY_ADDRESS, devad, reg)
#define PHY_MMD_WRITE(devad, reg, val)  PHY_MMDWrite(ipconfigPIC32_PHY_ADDRESS, devad, reg, val)

extern void PHYInitialise(void);
extern bool PHYSupportsWOL(void);
extern void PHYPrepareWakeOnLAN(void);
extern void PHYDeferredInterruptHandler(void);

extern uint16_t PHYRead(uint8_t phyaddr, uint8_t reg);
extern void PHYWrite(uint8_t phyaddr, uint8_t reg, uint16_t val);
extern uint16_t PHY_MMDRead(uint8_t phyaddr, uint8_t devad, uint16_t reg);
extern void PHY_MMDWrite(uint8_t phyaddr, uint8_t devad, uint16_t reg, uint16_t val);

extern void PHYGetStatus(phy_status_t *pStatus);
extern phy_tdr_state_t PHYCableDiagnostic(phy_tdr_cable_t type, float *pLenEstimate);

#ifdef	__cplusplus
}
#endif

#endif	// PHYGENERIC_H
