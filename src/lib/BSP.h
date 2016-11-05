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

class LED {
public:
    LED(uint8_t pin) : _pin(pin) {
        pinMode(_pin, OUTPUT);
    }

    void setState(boolean state) {
        _state = state;
        digitalWrite(_pin, (_state) ? HIGH : LOW);
    }

    boolean getState() {
        return _state;
    }

    void turnOn() {
        setState(true);
    }

    void turnOff() {
        setState(false);
    }

    void toggleState() {
        setState(!getState());
    }

private:
    uint8_t _pin = 4;
    bool _state = false;
};

class PushButton {
public:
    PushButton(uint8_t pin) : _pin(pin) {
        pinMode(_pin, INPUT_PULLUP);
    }

    bool isPushed() {
        return !digitalRead(_pin);
    }

private:
    uint8_t _pin;
};

#endif //BSP_H
