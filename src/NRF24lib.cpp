#include "NRF24.h"

#define SCK 13
#define MISO 12
#define MOSI 11
#define CE 8
#define CSN 7
#define IRQ 2

NRF24 nRF24(CSN, CE);

void setup() {
    Serial.begin(115200);
    Serial.println("nRF24lib test program");

    Serial.print(" - Configuring: ");
    nRF24.configure();
    Serial.println("DONE");

    // NRF24::debugConfigRegister(nRF24.read_register(CONFIG));
    // NRF24::debugEnAARegister(nRF24.read_register(EN_AA));
    // NRF24::debugEnRxAddrRegister(nRF24.read_register(EN_RXADDR));
    // NRF24::debugSetupAWRegister(nRF24.read_register(SETUP_AW));
    // NRF24::debugSetupRetrRegister(nRF24.read_register(SETUP_RETR));
    // NRF24::debugRfChRegister(nRF24.read_register(RF_CH));
    // NRF24::debugRfSetupRegister(nRF24.read_register(RF_SETUP));
    // NRF24::debugStatusRegister(nRF24.read_register(STATUS));
    // NRF24::debugObserveTxRegister(nRF24.read_register(OBSERVE_TX));
    // NRF24::debugRPDRegister(nRF24.read_register(RPD));

    // for (int i = 0; i < 5; ++i) {
    //     NRF24::debugRxBytesPipeRegister(nRF24.read_register(RX_PW_P0+i), i);
    // }
    // NRF24::debugFIFOStatusRegister(nRF24.read_register(FIFO_STATUS));

    // NRF24::debugDYNPDRegister(nRF24.read_register(DYNPD));
    // NRF24::debugFeatureRegister(nRF24.read_register(FEATURE));
}

void loop() { }