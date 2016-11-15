/**
 * BSP_example.ino -- Board Support Package example
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "BSP.h"

/**
 * Hardware configuration
 */

#define PB 3
#define LD 4

BSP::LED led(LD); // Set up a LED on pin 4
BSP::PushButton button(PB); // Set up a button on pin 3

/**
 * Setup
 */
void setup()
{
    // Start serial connection
    Serial.begin(115200);
}

/**
 * Loop
 */
void loop()
{
    // Read the pushbutton value into a variable
    bool pushed = button.isPushed();

    // Print out the value of the pushbutton
    Serial.print(" - Button pushed: ");
    Serial.print((pushed)? "TRUE":"FALSE");
    Serial.print(" (");
    Serial.print(digitalRead(PB));
    Serial.println(")");

    // Turn on the led when the button's pressed,
    // and off when it's not:
    if (pushed) {
        led.turnOn();
    } else {
        led.turnOff();
    }

    delay(100); // Wait 100ms
}
