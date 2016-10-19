#include "NRF24.h"
#include "NRF24Debug.h"

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

    NRF24Debug::debugConfigRegister(nRF24.read_register(CONFIG));
    // NRF24Debug::debugEnAARegister(nRF24.read_register(EN_AA));
    // NRF24Debug::debugEnRxAddrRegister(nRF24.read_register(EN_RXADDR));
    // NRF24Debug::debugSetupAWRegister(nRF24.read_register(SETUP_AW));
    // NRF24Debug::debugSetupRetrRegister(nRF24.read_register(SETUP_RETR));
    // NRF24Debug::debugRfChRegister(nRF24.read_register(RF_CH));
    // NRF24Debug::debugRfSetupRegister(nRF24.read_register(RF_SETUP));
    // NRF24Debug::debugStatusRegister(nRF24.read_register(STATUS));
    // NRF24Debug::debugObserveTxRegister(nRF24.read_register(OBSERVE_TX));
    // NRF24Debug::debugRPDRegister(nRF24.read_register(RPD));

    // for (int i = 0; i < 5; ++i) {
    //     NRF24Debug::debugRxBytesPipeRegister(nRF24.read_register(RX_PW_P0+i), i);
    // }
    // NRF24Debug::debugFIFOStatusRegister(nRF24.read_register(FIFO_STATUS));

    // NRF24Debug::debugDYNPDRegister(nRF24.read_register(DYNPD));
    // NRF24Debug::debugFeatureRegister(nRF24.read_register(FEATURE));
}

void loop() { }