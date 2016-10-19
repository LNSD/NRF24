#ifndef NRF24_H
#define NRF24_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "NRF24L01.h"

class NRF24
{
public:
    // Constructor and configure subroutines
    NRF24(uint8_t csn, uint8_t ce);              // Hardware SPI
	NRF24(uint8_t csn, uint8_t c, uint8_t irq);  // Hardware SPI + IRQ
	void configure();

    // Register read and write functions
    uint8_t read_register(uint8_t reg);
    void read_register_multi(uint8_t reg, uint8_t* buf, uint8_t len);
    void write_register(uint8_t reg, uint8_t value);
    void write_register_multi(uint8_t reg, uint8_t* buf, uint8_t len);

    // Util functions

private:
	uint8_t _sck, _mosi, _miso, _csn;
	uint8_t _ce, _irq;

	// Low-level signal & SPI-specific functions
	void csn(uint8_t val);
	void ce(uint8_t val);
	uint8_t spi_transfer(uint8_t c);
	void spi_transfer(void *buf, size_t count);
};

#endif
