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
      return ~(digitalRead(_pin) > 0);
    }

  private:
    uint8_t _pin;
};

#endif //BSP_H
