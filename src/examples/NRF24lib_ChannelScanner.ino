/**
 * NRF24lib_ChannelScanner.ino -- Channel scanner example sketch
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

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

#define LED 4
#define PB 3

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
NRF24::Driver nRF24(CSN, CE);

/**
 * Scan configuration
 */

const uint8_t NUM_CHANNELS = NRF24::MAX_RF_CHANNEL+1;
uint8_t values[NUM_CHANNELS];

const int NUM_REPS = 25;

/**
 * Setup
 */
void setup() {

    // Print preamble
    Serial.begin(115200);
    Serial.println("nRF24lib RF Channel scanner test sketch");

    // Rf radio configuration
    NRF24::Configuration config(NRF24::Mode_PRX);

    config.setRFChannel(76);
    config.disableAutoAck();
    config.disableDynamicPayloads();

    Serial.print(" - Configuring: ");
    nRF24.configure(config);
    Serial.println("DONE");

    // Reset current status
    // Notice reset and flush is the last thing we do
    nRF24.resetCurrentStatus();

    // Flush buffers
    nRF24.flushTxFifo();
    nRF24.flushRxFifo();

    // Rf radio setup
    nRF24.begin(); // "Power Down" -> "Standby-I"


    // Print out header, high then low digit
    Serial.println(" - Scanning: \n");

    Serial.print("\t");

    for (int i = 0; i < NUM_CHANNELS; ++i)
    {
        Serial.print(i >> 4, HEX);
    }
    Serial.print("\n\t");

    for (int i = 0; i < NUM_CHANNELS; ++i)
    {
        Serial.print(i & 0xF, HEX);
    }
    Serial.print("\n\t");

    for (int i = 0; i < NUM_CHANNELS; ++i)
    {
        Serial.print("-");
    }
    Serial.print("\n\t");
}

/**
 * Loop
 */
void loop()
{
    // Clear measurement values
    memset(values, 0, sizeof(values));

    unsigned long start = millis();

    // Scan all channels NUM_REPS times
    for (int rep = 0; rep < NUM_REPS; ++rep)
    {
        for (int ch = 0; ch < NUM_CHANNELS; ++ch)
        {
            // State: Standby-I

            // Select this channel
            nRF24.setRFChannel(ch);

            // Start listening
            nRF24.resetCurrentStatus();
            nRF24.flushRxFifo();
            nRF24.flushTxFifo();

            nRF24.start();

            // State: RX Settling

            delayMicroseconds(130 + 40); // Tstby2a + Tdelay_AGC

            // State: RX Mode

            nRF24.stop();

            // State: Standby-I

            nRF24.flushRxFifo();
            nRF24.flushTxFifo();

            if (nRF24.isCarrierDetected())
            {
                ++values[ch];
            }
        }
    }

    unsigned long end = millis();

    // Print out channel measurements, clamped to a single hex digit
    for (int i = 0; i < NUM_CHANNELS; ++i) 
    {
        Serial.print((values[i] > 0xF)? 0xF : values[i], HEX);
    }

    // Print scan elapsed time
    Serial.print(" - ");
    Serial.print(end - start);
    Serial.print(" ms");
    Serial.print("\n\t");
}
