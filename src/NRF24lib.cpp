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

    // Print preamble
    Serial.begin(115200);
    Serial.println("nRF24lib test sketch");

    // Rf radio configuration
    Serial.print(" - Configuring: ");
    Serial.println("DONE");
}

/**
 * Loop
 */
void loop() { }