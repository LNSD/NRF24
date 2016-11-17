/**
 * @file Debug.cpp
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "Debug.h"

void NRF24::Debug::debugConfigRegister(uint8_t content)
{
    Register::CONFIG config = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: CONFIG register content (0x");
    Serial.print(config.raw, HEX);
    Serial.println(")");

    // MASK_RX_DR bit
    Serial.print("\t - MASK_RX_DR: ");
    Serial.println(config.MASK_RX_DR);

    // MASK_TX_DS bit
    Serial.print("\t - MASK_TX_DS: ");
    Serial.println(config.MASK_TX_DS);

    // MASK_MAX_RT bit
    Serial.print("\t - MASK_MAX_RT: ");
    Serial.println(config.MASK_MAX_RT);

    // EN_CRC bit
    Serial.print("\t - EN_CRC: ");
    Serial.println(config.EN_CRC);

    // CRCO bit
    Serial.print("\t - CRCO: ");
    Serial.println(config.CRCO);

    // PWR_UP bit
    Serial.print("\t - PWR_UP: ");
    Serial.println(config.PWR_UP);

    // PRIM_RX bit
    Serial.print("\t - PRIM_RX: ");
    Serial.println(config.PRIM_RX);
}

void NRF24::Debug::debugEnAARegister(uint8_t content)
{
    Register::EN_AA enAA = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: EN_AA register content (0x");
    Serial.print(enAA.raw, HEX);
    Serial.println(")");

    // ENAA_P5 bit
    Serial.print("\t - ENAA_P5: ");
    Serial.println(enAA.ENAA_P5);

    // ENAA_P4 bit
    Serial.print("\t - ENAA_P4: ");
    Serial.println(enAA.ENAA_P4);

    // ENAA_P3 bit
    Serial.print("\t - ENAA_P3: ");
    Serial.println(enAA.ENAA_P3);

    // ENAA_P2 bit
    Serial.print("\t - ENAA_P2: ");
    Serial.println(enAA.ENAA_P2);

    // ENAA_P1 bit
    Serial.print("\t - ENAA_P1: ");
    Serial.println(enAA.ENAA_P1);

    // ENAA_P0 bit
    Serial.print("\t - ENAA_P0: ");
    Serial.println(enAA.ENAA_P0);
}

void NRF24::Debug::debugEnRxAddrRegister(uint8_t content)
{
    Register::EN_RXADDR enRxAddr = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: EN_RXADDR register content (0x");
    Serial.print(enRxAddr.raw, HEX);
    Serial.println(")");

    // ERX_P5 bit
    Serial.print("\t - ERX_P5: ");
    Serial.println(enRxAddr.ERX_P5);

    // ERX_P4 bit
    Serial.print("\t - ERX_P4: ");
    Serial.println(enRxAddr.ERX_P4);

    // ERX_P3 bit
    Serial.print("\t - ERX_P3: ");
    Serial.println(enRxAddr.ERX_P3);

    // ERX_P2 bit
    Serial.print("\t - ERX_P2: ");
    Serial.println(enRxAddr.ERX_P2);

    // ERX_P1 bit
    Serial.print("\t - ERX_P1: ");
    Serial.println(enRxAddr.ERX_P1);

    // ERX_P0 bit
    Serial.print("\t - ERX_P0: ");
    Serial.println(enRxAddr.ERX_P0);
}

void NRF24::Debug::debugSetupAWRegister(uint8_t content)
{
    Register::SETUP_AW setupAw = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: SETUP_AW register content (0x");
    Serial.print(setupAw.raw, HEX);
    Serial.println(")");

    // AW bits
    Serial.print("\t - AW (0x");
    Serial.print(setupAw.AW, HEX);
    Serial.print("): ");
    Serial.println(setupAw.AW, BIN);
}

void NRF24::Debug::debugSetupRetrRegister(uint8_t content)
{
    Register::SETUP_RETR setupRetr = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: SETUP_RETR register content (0x");
    Serial.print(setupRetr.raw, HEX);
    Serial.println(")");

    // ARD bits
    Serial.print("\t - ARD (0x");
    Serial.print(setupRetr.ARD, HEX);
    Serial.print("): ");
    Serial.println(setupRetr.ARD, BIN);

    // ARC bits
    Serial.print("\t - ARC (0x");
    Serial.print(setupRetr.ARC, HEX);
    Serial.print("): ");
    Serial.println(setupRetr.ARC, BIN);
}

void NRF24::Debug::debugRFChRegister(uint8_t content)
{
    Register::RF_CH rfCh = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: RF_CH register content (0x");
    Serial.print(rfCh.raw, HEX);
    Serial.println(")");

    // AW bits
    Serial.print("\t - RF_CH (0x");
    Serial.print(rfCh.RF_CH, HEX);
    Serial.print("): ");
    Serial.println(rfCh.RF_CH, BIN);
}

void NRF24::Debug::debugRFSetupRegister(uint8_t content)
{
    Register::RF_SETUP rfSetup = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: RF_SETUP register content (0x");
    Serial.print(rfSetup.raw, HEX);
    Serial.println(")");

    // CONT_WAVE bit
    Serial.print("\t - CONT_WAVE: ");
    Serial.println(rfSetup.CONT_WAVE);

    // PLL_LOCK bit
    Serial.print("\t - PLL_LOCK: ");
    Serial.println(rfSetup.PLL_LOCK);

    // RF_DR bits
    uint8_t rfdr = (uint8_t) ((rfSetup.RF_DR_HIGH << 1) | rfSetup.RF_DR_LOW);

    Serial.print("\t - RF_DR (0x");
    Serial.print(rfdr, HEX);
    Serial.print("): ");
    Serial.println(rfdr, BIN);

    // RF_PWR bits
    Serial.print("\t - RF_PWR (0x");
    Serial.print(rfSetup.RF_PWR, HEX);
    Serial.print("): ");
    Serial.println(rfSetup.RF_PWR, BIN);
}

void NRF24::Debug::debugStatusRegister(uint8_t content)
{
    Register::STATUS status = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: STATUS register content (0x");
    Serial.print(status.raw, HEX);
    Serial.println(")");

    // RX_DR bit
    Serial.print("\t - RX_DR: ");
    Serial.println(status.RX_DR);

    // TX_DS bit
    Serial.print("\t - TX_DS: ");
    Serial.println(status.TX_DS);

    // MAX_RT bit
    Serial.print("\t - MAX_RT: ");
    Serial.println(status.MAX_RT);

    // RX_P_NO bits
    Serial.print("\t - RX_P_NO (0x");
    Serial.print(status.RX_P_NO, HEX);
    Serial.print("): ");
    Serial.println(status.RX_P_NO, BIN);

    // TX_FULL bit
    Serial.print("\t - TX_FULL: ");
    Serial.println(status.TX_FULL);
}

void NRF24::Debug::debugObserveTxRegister(uint8_t content)
{
    Register::OBSERVE_TX observeTx = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: OBSERVE_TX register content (0x");
    Serial.print(observeTx.raw, HEX);
    Serial.println(")");

    // PLOS_CNT bits
    Serial.print("\t - PLOS_CNT (0x");
    Serial.print(observeTx.PLOS_CNT, HEX);
    Serial.print("): ");
    Serial.println(observeTx.PLOS_CNT, DEC);

    // ARC_CNT bits
    Serial.print("\t - ARC_CNT (0x");
    Serial.print(observeTx.ARC_CNT, HEX);
    Serial.print("): ");
    Serial.println(observeTx.ARC_CNT, DEC);
}

void NRF24::Debug::debugRpdRegister(uint8_t content)
{
    Register::RPD rpd = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: RPD register content (0x");
    Serial.print(rpd.raw, HEX);
    Serial.println(")");

    // RPD bit
    Serial.print("\t - RPD: ");
    Serial.println(rpd.RPD);
}

void NRF24::Debug::debugRxPipeAddressRegister(uint8_t *content, RxPipe pipe, uint8_t len)
{
    uint8_t length = (pipe<2)? len:1;

    // Debug info header
    Serial.print(" - DEBUG: RX_ADDR_P");
    Serial.print(pipe, DEC);
    Serial.print(" (");
    Serial.print(length, DEC);
    Serial.print((length>1) ? " bytes): ":" byte): ");

    for (int i = 0; i < length; ++i) {
        if (content[i]<=0xF) Serial.print(0, HEX);
        Serial.print(content[i], HEX);
        Serial.print((i<length-1) ? ":":"\n");
    }
}

void NRF24::Debug::debugTxAddressRegister(uint8_t *content, uint8_t len)
{
    // Debug info header
    Serial.print(" - DEBUG: TX_ADDR (");
    Serial.print(len, DEC);
    Serial.print(" bytes): ");

    for (int i = 0; i < len; ++i) {
        if (content[i] <= 0xF) Serial.print(0, HEX);
        Serial.print(content[i], HEX);
        Serial.print((i<len-1) ? ":":"\n");
    }
}

void NRF24::Debug::debugRxPipePayloadWidthRegister(uint8_t content, RxPipe pipe)
{
    Register::RX_PW_PN rxPwPN = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: RX_PW_P");
    Serial.print(pipe, DEC);
    Serial.print(" register content: ");
    Serial.println(rxPwPN.RX_PW_PN, DEC);
}

void NRF24::Debug::debugFifoStatusRegister(uint8_t content)
{
    Register::FIFO_STATUS fifoStatus = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: FIFO_STATUS register content (0x");
    Serial.print(fifoStatus.raw, HEX);
    Serial.println(")");

    // TX_REUSE bit
    Serial.print("\t - TX_REUSE: ");
    Serial.println(fifoStatus.TX_REUSE);

    // TX_FULL bit
    Serial.print("\t - TX_FULL: ");
    Serial.println(fifoStatus.TX_FULL);

    // TX_EMPTY bit
    Serial.print("\t - TX_EMPTY: ");
    Serial.println(fifoStatus.TX_EMPTY);

    // RX_FULL bit
    Serial.print("\t - RX_FULL: ");
    Serial.println(fifoStatus.RX_FULL);

    // RX_EMPTY bit
    Serial.print("\t - RX_EMPTY: ");
    Serial.println(fifoStatus.RX_EMPTY);
}

void NRF24::Debug::debugDynpdRegister(uint8_t content)
{
    Register::DYNPD dynpd = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: DYNPD register content (0x");
    Serial.print(dynpd.raw, HEX);
    Serial.println(")");

    // DPL_P0 bit
    Serial.print("\t - DPL_P0:");
    Serial.println(dynpd.DPL_P0);

    // DPL_P1 bit
    Serial.print("\t - DPL_P1:");
    Serial.println(dynpd.DPL_P1);

    // DPL_P2 bit
    Serial.print("\t - DPL_P2:");
    Serial.println(dynpd.DPL_P2);

    // DPL_P3 bit
    Serial.print("\t - DPL_P3:");
    Serial.println(dynpd.DPL_P3);

    // DPL_P4 bit
    Serial.print("\t - DPL_P4:");
    Serial.println(dynpd.DPL_P4);

    // DPL_P5 bit
    Serial.print("\t - DPL_P5:");
    Serial.println(dynpd.DPL_P5);
}

void NRF24::Debug::debugFeatureRegister(uint8_t content)
{
    Register::FEATURE feature = { .raw = content };

    // Debug info header
    Serial.print(" - DEBUG: FEATURE register content (0x");
    Serial.print(feature.raw, HEX);
    Serial.println(")");

    // EN_DPL bit
    Serial.print("\t - EN_DPL: ");
    Serial.println(feature.EN_DPL);

    // EN_ACK_PAY bit
    Serial.print("\t - EN_ACK_PAY: ");
    Serial.println(feature.EN_ACK_PAY);

    // EN_DYN_ACK bit
    Serial.print("\t - EN_DYN_ACK: ");
    Serial.println(feature.EN_DYN_ACK);
}