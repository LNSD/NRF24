#include <NRF24Debug.h>
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

     NRF24Debug::debugConfigRegister(nRF24.readRegister(CONFIG));
     NRF24Debug::debugEnAARegister(nRF24.readRegister(EN_AA));
     NRF24Debug::debugEnRxAddrRegister(nRF24.readRegister(EN_RXADDR));
     NRF24Debug::debugSetupAWRegister(nRF24.readRegister(SETUP_AW));
     NRF24Debug::debugSetupRetrRegister(nRF24.readRegister(SETUP_RETR));
     NRF24Debug::debugRfChRegister(nRF24.readRegister(RF_CH));
     NRF24Debug::debugRfSetupRegister(nRF24.readRegister(RF_SETUP));
     NRF24Debug::debugStatusRegister(nRF24.readRegister(STATUS));
     NRF24Debug::debugObserveTxRegister(nRF24.readRegister(OBSERVE_TX));
     NRF24Debug::debugRPDRegister(nRF24.readRegister(RPD));

    for (int i = 0; i < 6; ++i) {
        if (i > 1)
        {
            NRF24Debug::debugDataPipeRxAddrRegister(nRF24.readRegister(RX_ADDR_P2+i-2), i);
        }
        else
        {
            uint8_t buffer[5];
            nRF24.readRegister(RX_ADDR_P0+i, buffer, sizeof(buffer));
            NRF24Debug::debugDataPipeRxAddrRegister(buffer, i);
        }
    }

    /*uint8_t buffer[5];
    nRF24.readRegister(TX_ADDR, buffer, sizeof(buffer));
    NRF24Debug::debugTxAddrRegister(buffer);*/

    /*for (int i = 0; i < 6; ++i) {
        NRF24Debug::debugRxBytesPipeRegister(nRF24.readRegister(RX_PW_P0+i), i);
    }*/
    // NRF24Debug::debugFIFOStatusRegister(nRF24.readRegister(FIFO_STATUS));

    // NRF24Debug::debugDYNPDRegister(nRF24.readRegister(DYNPD));
    // NRF24Debug::debugFeatureRegister(nRF24.readRegister(FEATURE));

}

void loop() { }