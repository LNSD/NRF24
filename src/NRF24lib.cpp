#include <Arduino.h>

#include "NRF24.h"

#define SCK 13
#define MISO 12
#define MOSI 11
#define CE 8
#define CSN 7
#define IRQ 2

NRF24 nRF24(CSN, CE);

void setup() {
    Serial.begin(9600);
    Serial.println("Hello nRF24lib!");

    nRF24.configure();

    uint8_t config = nRF24.read_register(CONFIG);
    Serial.print("Config: 0x");
    Serial.print(config, HEX);
    Serial.print(" - ");
    Serial.print(config, BIN);
}

void loop() { }