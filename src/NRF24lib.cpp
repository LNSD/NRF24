#include <Arduino.h>

#include "NRF24.h"

#define SCK 13
#define MISO 12
#define MOSI 11
#define CE 8
#define CSN 7
#define IRQ 2

NRF24 nRF24(CSN, CE, IRQ);

void setup() {
    Serial.begin(9600);

    Serial.println("Hello nRF24lib!");
}

void loop() {

}
