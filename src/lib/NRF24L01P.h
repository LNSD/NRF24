/**
 * @file NRF24L01P.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24L01P_H
#define NRF24L01P_H

namespace NRF24
{
    /**
     * @name nRF24L01+ Register Map
     */

    const static uint8_t CONFIG      = 0x00;
    const static uint8_t EN_AA       = 0x01;
    const static uint8_t EN_RXADDR   = 0x02;
    const static uint8_t SETUP_AW    = 0x03;
    const static uint8_t SETUP_RETR  = 0x04;
    const static uint8_t RF_CH       = 0x05;
    const static uint8_t RF_SETUP    = 0x06;
    const static uint8_t STATUS      = 0x07;
    const static uint8_t OBSERVE_TX  = 0x08;
    const static uint8_t RPD         = 0x09;
    const static uint8_t RX_ADDR_P0  = 0x0A;
    const static uint8_t RX_ADDR_P1  = 0x0B;
    const static uint8_t RX_ADDR_P2  = 0x0C;
    const static uint8_t RX_ADDR_P3  = 0x0D;
    const static uint8_t RX_ADDR_P4  = 0x0E;
    const static uint8_t RX_ADDR_P5  = 0x0F;
    const static uint8_t TX_ADDR     = 0x10;
    const static uint8_t RX_PW_P0    = 0x11;
    const static uint8_t RX_PW_P1    = 0x12;
    const static uint8_t RX_PW_P2    = 0x13;
    const static uint8_t RX_PW_P3    = 0x14;
    const static uint8_t RX_PW_P4    = 0x15;
    const static uint8_t RX_PW_P5    = 0x16;
    const static uint8_t FIFO_STATUS = 0x17;
    const static uint8_t DYNPD       = 0x1C;
    const static uint8_t FEATURE     = 0x1D;

    /// nRF24L01 retro-compatibility

    const static uint8_t CD = RPD;

    /**
     * @name Registers Bit Mnemonics
     */

    // CONFIG register
    const static uint8_t MASK_RX_DR  = 6;
    const static uint8_t MASK_TX_DS  = 5;
    const static uint8_t MASK_MAX_RT = 4;
    const static uint8_t EN_CRC      = 3;
    const static uint8_t CRCO        = 2;
    const static uint8_t PWR_UP      = 1;
    const static uint8_t PRIM_RX     = 0;

    // EN_AA register
    const static uint8_t ENAA_P5 = 5;
    const static uint8_t ENAA_P4 = 4;
    const static uint8_t ENAA_P3 = 3;
    const static uint8_t ENAA_P2 = 2;
    const static uint8_t ENAA_P1 = 1;
    const static uint8_t ENAA_P0 = 0;

    // EN_RXADDR register
    const static uint8_t ERX_P5 = 5;
    const static uint8_t ERX_P4 = 4;
    const static uint8_t ERX_P3 = 3;
    const static uint8_t ERX_P2 = 2;
    const static uint8_t ERX_P1 = 1;
    const static uint8_t ERX_P0 = 0;

    // SETUP_AW register

    // SETUP_RETR register
    const static uint8_t ARD = 4;
    const static uint8_t ARC = 0;

    // RF_CH register

    // RF_SETUP register
    const static uint8_t CONT_WAVE  = 7;
    const static uint8_t RF_DR_LOW  = 5;
    const static uint8_t PLL_LOCK   = 4;
    const static uint8_t RF_DR_HIGH = 3;
    const static uint8_t RF_PWR     = 1;

    /// nRF24L01 reto-compatibility
    const static uint8_t LNA_HCURR = 0;

    // STATUS register
    const static uint8_t RX_DR   = 6;
    const static uint8_t TX_DS   = 5;
    const static uint8_t MAX_RT  = 4;
    const static uint8_t RX_P_NO = 1;
    const static uint8_t TX_FULL = 0;

    // OBSERVE_TX register
    const static uint8_t PLOS_CNT = 4;
    const static uint8_t ARC_CNT  = 0;

    // RPD register

    // RX_ADDR_P# registers

    // TX_ADDR register

    // RX_PW_P# registers

    // FIFO_STATUS register
    const static uint8_t TX_REUSE     = 6;
    const static uint8_t TX_FIFO_FULL = 5;
    const static uint8_t TX_EMPTY     = 4;
    const static uint8_t RX_FULL      = 1;
    const static uint8_t RX_EMPTY     = 0;

    // DYNPD register
    const static uint8_t DPL_P5 = 5;
    const static uint8_t DPL_P4 = 4;
    const static uint8_t DPL_P3 = 3;
    const static uint8_t DPL_P2 = 2;
    const static uint8_t DPL_P1 = 1;
    const static uint8_t DPL_P0 = 0;

    // FEATURE register
    const static uint8_t EN_DPL     = 2;
    const static uint8_t EN_ACK_PAY = 1;
    const static uint8_t EN_DYN_ACK = 0;

    /**
     * @name Command Mnemonics
     */

    const static uint8_t R_REGISTER    		= 0x00;
    const static uint8_t W_REGISTER    		= 0x20;
    const static uint8_t R_RX_PAYLOAD  		= 0x61;
    const static uint8_t W_TX_PAYLOAD  		= 0xA0;
    const static uint8_t FLUSH_TX      		= 0xE1;
    const static uint8_t FLUSH_RX      		= 0xE2;
    const static uint8_t REUSE_TX_PL   		= 0xE3;
    const static uint8_t R_RX_PL_WID   		= 0x60;
    const static uint8_t W_ACK_PAYLOAD 		= 0xA8;
    const static uint8_t W_TX_PAYLOAD_NOACK = 0xB0;
    const static uint8_t NOP           		= 0xFF;

    /// nRF24L01 reto-compatibility
    const static uint8_t ACTIVATE = 0x50;

    /**
     * @Name Command Masks
     */

    const static uint8_t REGISTER_MASK 		= 0x1F;
    const static uint8_t W_ACK_PAYLOAD_MASK = 0x07;

}

#endif //NRF24L01P_H
