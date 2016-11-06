/**
 * BSP.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef BSP_H
#define BSP_H

#include <stdint.h>

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

/**
 * LED driver class
 */
class LED {
public:

    /**
     * @name Arduino Constructor
     */

    /**
     * Arduino constructor
     *
     * Creates a new instance of this driver. Before using, create an instance and pass
     * the unique pin where is the LED connected.
     *
     * @param pin Pin where the LED is attached
     */
    LED(uint8_t pin) : _pin(pin) {
        pinMode(_pin, OUTPUT);
    }

    /**
     * @name State setters and getters
     */

    /**
     * Set LED state
     * @param state LED state
     */
    void setState(boolean state) {
        _state = state;
        digitalWrite(_pin, (_state) ? HIGH : LOW);
    }

    /**
     * Get current LED state
     * @return LED status
     */
    boolean getState() {
        return _state;
    }

    /**
     * @name Driver functions
     */

    /**
     * Turn LED on
     */
    void turnOn() {
        setState(true);
    }

    /**
     * Turn LED off
     */
    void turnOff() {
        setState(false);
    }

    /**
     * Toggle LED state
     */
    void toggleState() {
        setState(!getState());
    }

private:
    uint8_t _pin = 4;
    bool _state = false;
};

/**
 * Push button driver class
 */
class PushButton {
public:

    /**
     * @name Arduino Constructor
     */

    /**
     * Arduino constructor
     *
     * Creates a new instance of this driver. Before using, create an instance and pass
     * the unique pin where is the Push button connected.
     *
     * @param pin Pin where the Push Button is attached
     */
    PushButton(uint8_t pin) : _pin(pin) {
        pinMode(_pin, INPUT_PULLUP);
    }

    /**
     * @name Driver functions
     */

    /**
     * Check if the push button is pushed or not.
     */
    bool isPushed() {
        return !digitalRead(_pin);
    }

private:
    uint8_t _pin;
};

#endif //BSP_H
