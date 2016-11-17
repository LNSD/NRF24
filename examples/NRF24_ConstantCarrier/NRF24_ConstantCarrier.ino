/**
 * @file NRF24lib_ConstantCarrier.ino -- Constant carrierr example sketch
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

#define LD 4
#define PB 3

// Set up nRF24L01 radio on SPI bus plus pins 7 & 8
NRF24::Driver nRF24(CSN, CE);

/**
 * RF Channel configuration
 */

uint8_t rfChannel = 0x70; // RF Channel number
String inString = "";     // String to hold input

/**
 * Setup
 */
void setup() {

    // Print preamble
    Serial.begin(115200);
    Serial.println("nRF24lib: Constant carrier test sketch");

    // RF radio configuration
    NRF24::Configuration config(NRF24::Mode_PTX);
    config.setOutputPower(NRF24::OutputPower_0dBm);

    config.enableConstantCarrier();
    config.forcePllLock();

    config.setRFChannel(rfChannel);

    Serial.print(" - Configuring: ");
    nRF24.configure(config);
    Serial.println("DONE");

    // Reset current status
    // Notice reset and flush is the last thing we do
    nRF24.resetCurrentStatus();

    // Flush buffers
    nRF24.flushTxFifo();
    nRF24.flushRxFifo();

    /* 
     * RF radio setup 
     */
    
    // State: "Power down"
    
    nRF24.begin(); // "Power Down" -> "Standby-I"

    // State: "Santandy-I"
    
    nRF24.start(); // "Standby-I" -> "TX Mode"

    // State: "TX Settling"
    
    delayMicroseconds(130);

    // State: "TX Mode"

    Serial.print(" - Default carrier set to RF Channel ");
    Serial.print(rfChannel);
    Serial.print(" (0x");
    Serial.print(rfChannel, HEX);
    Serial.println(")");

}

/**
 * Loop
 */
void loop()
{
  // Read serial input
  while (Serial.available() > 0)
  {
    int inChar = Serial.read();
    // Check if input is a digit
    if (isDigit(inChar)) 
    {  
      // convert the incoming byte to a char
      // and add it to the string
      inString += (char) inChar;
    }
  }
  
  // Parse input number
  int number = inString.toInt();
  
  // Clear the string for new input:
  inString = "";
  
  if (number != 0)
  {
    if (0 > number && number >= NRF24::MAX_RF_CHANNEL)
    {
        Serial.print(" - Channel ");
        Serial.print(number);
        Serial.println(" is not a valid RF Channel: ERROR");
    } else {
        rfChannel = number;
        
        Serial.print(" - Setting carrier to RF Channel ");
        Serial.print(rfChannel);
        Serial.print(" (0x");
        Serial.print(rfChannel, HEX);
        Serial.print("): ");

        // State: "TX Mode"
        
        nRF24.stop();

        // State: "Standby-I"
        
        // Select this channel
        nRF24.setRFChannel(rfChannel);

        // Start listening
        nRF24.resetCurrentStatus();
        nRF24.flushRxFifo();
        nRF24.flushTxFifo();

        nRF24.start();

        // State: "TX Settling"
    
        delayMicroseconds(130);

        // State: "TX Mode"

        Serial.println("DONE");
    }
  }

  delay(100);
}
