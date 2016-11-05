/**
 * NRF24lib.cpp -- Arduino test sketch
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "NRF24.h"
#include "NRF24Debug.h"

/**
 * Hardware configuration
 */

#define SCK 13
#define MISO 12
#define MOSI 11
#define CE 8
#define CSN 7
#define IRQ 2

#define LED 4
#define PB 3

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
NRF24 nRF24(CSN, CE);

/**
 * Setup
 */
void setup() {
    Serial.begin(115200);
    Serial.println("nRF24lib test program");

    // NRF24::Config config(NRF24::Mode_PTX);
    // config.setTransceiverMode(NRF24::Mode_PRX);
    // config.setPower(NRF24::OutputPower_0dBm);
    // config.setDataRate(NRF24::DataRate_2Mbps);

    // config.setRfChannel(2);
    // config.disableConstCarrier();
    // config.setCRC(NRF24::CRC_16);

    // config.setAddrWidth(5);

    // config.enableAutoAck();
    // config.setAutoRtDelay(250);
    // config.setAutoRtCount(3);

    // config.disableAckPayload();
    // config.disableDynamicAck();

    Serial.print(" - Configuring: ");
    nRF24.configure(); // nRF24.configure(config);
    Serial.println("DONE");

    // NRF24Debug::debugConfigRegister(nRF24.readRegister(CONFIG));
    // NRF24Debug::debugEnAARegister(nRF24.readRegister(EN_AA));
    // NRF24Debug::debugEnRxAddrRegister(nRF24.readRegister(EN_RXADDR));
    // NRF24Debug::debugSetupAWRegister(nRF24.readRegister(SETUP_AW));
    // NRF24Debug::debugSetupRetrRegister(nRF24.readRegister(SETUP_RETR));
    // NRF24Debug::debugRfChRegister(nRF24.readRegister(RF_CH));
    // NRF24Debug::debugRfSetupRegister(nRF24.readRegister(RF_SETUP));
    // NRF24Debug::debugStatusRegister(nRF24.readRegister(STATUS));
    // NRF24Debug::debugObserveTxRegister(nRF24.readRegister(OBSERVE_TX));
    // NRF24Debug::debugRPDRegister(nRF24.readRegister(RPD));

    /*for (int i = 0; i < 6; ++i) {
        if (i > 1)
        {
            NRF24Debug::debugDataPipeRxAddrRegister(nRF24.readRegister(RX_ADDR_P2+i-2), i);
        }
        else
        {
            uint8_t buffer[5];
            nRF24.readRegister(RX_ADDR_P0+i, buffer, sizeof(buffer));
            NRF24Debug::debugDataPipeRxAddrRegister(buffer, i);
        }
    }*/

    /*uint8_t buffer[5];
    nRF24.readRegister(TX_ADDR, buffer, sizeof(buffer));
    NRF24Debug::debugTxAddrRegister(buffer);*/

    /*for (int i = 0; i < 6; ++i) {
        NRF24Debug::debugRxBytesPipeRegister(nRF24.readRegister(RX_PW_P0+i), i);
    }*/
    // NRF24Debug::debugFIFOStatusRegister(nRF24.readRegister(FIFO_STATUS));

    // NRF24Debug::debugDYNPDRegister(nRF24.readRegister(DYNPD));
    // NRF24Debug::debugFeatureRegister(nRF24.readRegister(FEATURE));

    /*Serial.print(" - Plus variant: ");
    Serial.println(nRF24.isPVariant()? "TRUE":"FALSE");*/
}

/**
 * Loop
 */
void loop() { }