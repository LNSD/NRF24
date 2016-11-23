/**
 * @file NRF24_BasicRX.ino
 * @brief Basic receiver example sketch
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "Arduino.h"
#include "BSP.h"
#include "Core.h"

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
NRF24::Radio nRF24(CSN, CE);

// Setup board LED on pin 4
BSP::LED led(LD);

/**
 * Receiver configuraton
 */

uint8_t addr[5] = { 0x3F, 0x54, 0xC2, 0x7A, 0x11 };

const size_t PAYLOAD_SIZE = 4; // 4 bytes
unsigned char buffer[PAYLOAD_SIZE];

/**
 * Check if there are packets available in the RX FIFO
 * @return If packets are available
 */
bool available();

/**
 * Setup
 */
void setup() {
    // Print preamble
    Serial.begin(115200);
    Serial.println("nRF24lib: Basic receiver example sketch");

    Serial.print(" - Configuring: ");
    nRF24.configure();

    // RF radio configuration
    nRF24.setTransceiverMode(NRF24::RX_Mode);

    nRF24.setRFChannel(0x70);

    nRF24.setRxPipeAddress(0, addr, sizeof(addr));
    nRF24.setRxPipePayloadLength(0, PAYLOAD_SIZE);
    nRF24.enableRxPipeAddress(0);

    // Send NO ACK
    nRF24.disableAutoAck();


    Serial.println("DONE");

    // RF radio setup
    nRF24.begin(); // "Power Down" -> "Standby-I"

    // Clear current status
    // Notice clear and flush is the last thing we do
    nRF24.clearStatus();

    // Flush buffers
    nRF24.flushTxFifo();
    nRF24.flushRxFifo();

    // State: "Standby-I"

    nRF24.start();

    // State: "RX Settling"

    delayMicroseconds(130);

    // State: "RX Mode"

    Serial.println(" - Started listening...");
}

/**
 * Loop
 */
void loop()
{
    if (available())
    {
        led.turnOn();

        Serial.print(" - Received packet: ");

        nRF24.readPayload((uint8_t *) buffer, PAYLOAD_SIZE);

        nRF24.clearStatus();
        nRF24.flushRxFifo();

        for (int b = 0; b < PAYLOAD_SIZE; ++b) {
            Serial.print((char) buffer[b]);
        }
        Serial.println();

        delay(100); // Wait 100 ms
        led.turnOff();
    }
}

bool available()
{
    return (nRF24.getRxFifoStatus() != NRF24::FIFO_STATUS_EMPTY);
}