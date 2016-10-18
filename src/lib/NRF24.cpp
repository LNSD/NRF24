#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>

#include "NRF24.h"

/**
 * MACROS
 */

#ifndef _BV
#define _BV(bit) (1<<(bit))
#endif

/**
 * Constructor and configure subroutines
 */

//region Constructor and configure subroutines

/**
 * Instantiates a new NRF24 class using hardware SPI.
 * @param csn SPI chip select pin (CS/SSEL)
 * @param ce SPI chip enable pin (CE)
 */
NRF24::NRF24(uint8_t csn, uint8_t ce):
    _sck(13),
    _miso(12),
    _mosi(11),
    _csn(csn),
    _ce(ce),
    _irq(0)
{
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);
}

/**
 * Instantiates a new NRF24 class using hardware SPI and IRQ.
 * @param csn SPI chip select pin (CS/SSEL)
 * @param ce SPI chip enable pin (CE)
 * @param irq SPI chip interrupt pin (IRQ)
*/
NRF24::NRF24(uint8_t csn, uint8_t ce, uint8_t irq):
    _sck(13),
    _miso(12),
    _mosi(11),
    _csn(csn),
    _ce(ce),
    _irq(irq)
{
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);

    //TODO Configure interrupt pin to IRQ
}

/**
 * Setups the hardware
 */
void NRF24::configure()
{
    // SPI configure
    SPI.begin();
    SPI.beginTransaction(SPISettings((uint32_t) 1000000, MSBFIRST, SPI_MODE0));

    // Interface configure
    ce(LOW);
    csn(HIGH);
}

/**
 * Setups the hardware
 * @param config nRF24 configuration
 */
void NRF24::configure(NRF24Config config)
{
    // SPI configure
    SPI.begin();
    SPI.beginTransaction(SPISettings((uint32_t) 1000000, MSBFIRST, SPI_MODE0));

    // Interface configure
    ce(LOW);
    csn(HIGH);
}

//endregion

/**
 *  Low-level signal & SPI-specific functions
 */

//region Low-level signal & SPI-specific functions

/**
 * Low-level CSN signal enable/disable
 * @param level Output signal level
 */
void NRF24::csn(uint8_t val)
{
    digitalWrite(_csn, val);
}

/**
 * Low-level CE signal enable/disable
 * @param level Output signal level
 */
void NRF24::ce(uint8_t val)
{
    digitalWrite(_ce, val);
}

/**
 * Low-level SPI write wrapper
 * @param c Byte to write to the SPI bus
 * @return Data received while transaction
 */
uint8_t NRF24::spi_transfer(uint8_t c)
{
    uint8_t rxValue;

    csn(LOW);
    rxValue = SPI.transfer(c);
    csn(HIGH);

    return rxValue;
}

/**
 * Low-level SPI write wrapper
 * @param  buf Array of data to be transferred
 * @param  count Array data length
 */
void NRF24::spi_transfer(void *buf, size_t count)
{
    csn(LOW);
    SPI.transfer(buf, count);
    csn(HIGH);
}

//endregion

/**
 * Register read and write functions
 */

//region Register read and write functions

/**
 * Read nRF24 register
 * @param reg Register to read
 * @return Register content
 */
uint8_t NRF24::read_register(uint8_t reg)
{
    uint8_t cmd[2] = {R_REGISTER | (REGISTER_MASK & reg), 0xff};

    spi_transfer(cmd, 2);

    return cmd[1];
}

/**
 * Read nRF24 multi-byte register
 * @param reg Register to read
 * @param buf Array of data received
 * @param len Array length
*/
void NRF24::read_register_multi(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t cmd = R_REGISTER | (REGISTER_MASK & reg);

    for (int i = 0; i < len; ++i)
    {
        buf[i] = 0xff;
    }

    spi_transfer(cmd);
    spi_transfer(buf, len);
}

/**
 * Write nRF24 register
 * @param reg Register to write
 * @param value Data to write
 */
void NRF24::write_register(uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {W_REGISTER | (REGISTER_MASK & reg), value};

    spi_transfer(cmd, 2);
}

/**
 * Write nRF24 multi-byte register
 * @param reg Register to write
 * @param buf Array of data to write
 * @param len Array length
 */
void NRF24::write_register_multi(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t cmd = W_REGISTER | (REGISTER_MASK & reg);

    spi_transfer(cmd);
    spi_transfer(buf, len);
}

//endregion

/**
 * Configuration functions. Getters and setters
 */

//region Configuration functions. Getters and setters

//endregion

/**
 * Command functions
 */

//region Command functions

// TODO Command functions

//endregion

/**
 * Util & debug functions
 */

//region Util & debug functions

//endregion