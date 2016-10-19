#ifndef NRF24_H
#define NRF24_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "NRF24L01.h"

/**
 * Pipes definition.
 */
typedef enum {
    NRF24_RX_P0 = 0,
    NRF24_RX_P1 = 1,
    NRF24_RX_P2 = 2,
    NRF24_RX_P3 = 3,
    NRF24_RX_P4 = 4,
    NRF24_RX_P5 = 5
} NRF24_RxPipe_t;

class NRF24
{
public:
    // Constructor and configure subroutines
    NRF24(uint8_t csn, uint8_t ce);              // Hardware SPI
	NRF24(uint8_t csn, uint8_t c, uint8_t irq);  // Hardware SPI + IRQ
	void configure();

    // Register read and write functions
    uint8_t readRegister(uint8_t reg);
    void readRegister(uint8_t reg, uint8_t *buf, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t value);
    void writeRegister(uint8_t reg, uint8_t *buf, uint8_t len);

    // Command functions
    uint8_t getRxPayloadLength();
    void readRxPayload(uint8_t* data, uint8_t len);
    void writeTxPayload(uint8_t* data, uint8_t len);
    void writePipeACKPayload(NRF24_RxPipe_t pipe, uint8_t* data, uint8_t len);
    void disableAAforPayload();
    void reuseTxPayload();
    void flushTXFIFO();
    void flushRXFIFO();

private:
	uint8_t _sck, _mosi, _miso, _csn;
	uint8_t _ce, _irq;

	// Low-level signal & SPI-specific functions
	void csn(uint8_t val);
	void ce(uint8_t val);
    void spiCmdTransfer(uint8_t cmd);
    void spiCmdTransfer(uint8_t cmd, void *buf, size_t len);
};

#endif
