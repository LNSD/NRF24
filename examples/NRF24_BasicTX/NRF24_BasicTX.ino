/**
 * @file NRF24_BasicTX.ino -- Basic trasnmitter example sketch
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "Arduino.h"
#include "NRF24.h"

/**
 * Hardware configuration
 */

#define SCK 13
#define MISO 12
#define MOSI 11
#define CE 8
#define CSN 7
#define IRQ 2

#define LD 4
#define PB 3

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
NRF24::Driver nRF24(CSN, CE);

/**
 * Transmitter configuraton
 */

uint8_t addr[5] = { 0x3F, 0x54, 0xC2, 0x7A, 0x11 };

const uint8_t PAYLOAD_SIZE = 4; // 4 bytes
uint8_t buffer[PAYLOAD_SIZE] = {'S', 'U', 'P', '!'};

/**
 * Send buffer content
 * @return If transmission successful
 */
bool send(uint8_t *buffer, uint8_t len);

/**
 * Setup
 */
void setup()
{
    // Print preamble
    Serial.begin(115200);
    Serial.println("nRF24lib: Basic transmitter example sketch");

    // RF radio configuration
    NRF24::Configuration config(NRF24::Mode_PTX);

    config.setRFChannel(0x70);

    config.setTxAddress(addr);

    // Wait NO ACK
    config.disableAutoAck();
    config.setAutoRtCount(NRF24::AUTO_RT_DISABLED);

    Serial.print(" - Configuring: ");
    nRF24.configure(config);
    Serial.println("DONE");

    // RF radio setup
    nRF24.begin(); // "Power Down" -> "Standby-I"

    // Reset current status
    // Notice reset and flush is the last thing we do
    nRF24.resetCurrentStatus();

    // Flush buffers
    nRF24.flushTxFifo();
    nRF24.flushRxFifo();

    // State: "Standby-I"
}

/**
 * Loop
 */
void loop() {

    // Send buffer
    Serial.print(" - Sending buffer: ");

    if (send(buffer, PAYLOAD_SIZE)) {
        Serial.println("DONE");
    } else {
        Serial.println("FAILED");
    }

    delay(2500); // Wait 2.5 seconds
}

bool send(uint8_t *buffer, uint8_t len)
{
    // Write data to the TX FIFO
    nRF24.writeTxPayload(buffer, len);

    // Start transmission
    nRF24.start();

    // State: "TX Settling"

    delayMicroseconds(15); // More than 10us

    // State: "TX Mode"

    nRF24.stop();

    // State: "Standby-I"

    /**
     * Monitor the transmission (Blocking)
     */

    /*
     * STATUS register fields meaning
     *  - The transmission was successful.  (TX_DS)
     *  - The transmission failed, max retries reached (MAX_RT)
     *  - There is an ack packet waiting (RX_DR)
     */

    NRF24::Register::STATUS status;

    /* Monitor the transmission. Block here until we get TX_DS (transmission completed and ACK'd)
     * or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
     * is flaky and we get neither.
     */

    const uint32_t timeout = 500; // 500 ms timeout
    unsigned long sentAt = millis();

    do {
        status = nRF24.getCommStatus();
    } while(!status.TX_DS && !status.MAX_RT && (millis() - sentAt < timeout));

    // Clear status
    nRF24.resetCurrentStatus();

    // Return if TX data was sent
    return status.TX_DS;
}