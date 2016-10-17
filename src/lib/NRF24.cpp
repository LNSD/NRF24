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
 * Constructors
 */

/**************************************************************************/
/*!
    @brief  Instantiates a new NRF24 class using hardware SPI.
    @param  csn        SPI chip select pin (CS/SSEL)
    @param  ce         SPI chip enable pin (CE)
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief  Instantiates a new NRF24 class using hardware SPI and IRQ.
    @param  csn        SPI chip select pin (CS/SSEL)
    @param  ce         SPI chip enable pin (CE)
    @param  irq        SPI chip interrupt pin (IRQ)
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void NRF24::begin()
{
    // SPI init
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // Interface init
    ce(LOW);
    csn(HIGH);
}

/**
 * Low-level signals & SPI
 */

/**************************************************************************/
/*!
    @brief  Check if nRF24 chip is P variant
    @returns  If nRF24 chip is P variant
*/
/**************************************************************************/
bool NRF24::isPVariant()
{
    uint8_t setup = read_register(RF_SETUP);
    uint8_t aux = setup;

    aux &= ~_BV(RF_DR_HIGH);
    aux |= _BV(RF_DR_LOW);

    write_register(RF_SETUP, aux);
    aux = read_register(RF_SETUP);

    write_register(RF_SETUP, setup);

    return (bool)(aux & _BV(RF_DR_LOW));
}


/**
 * Low-level signals & SPI
 */

 /**************************************************************************/
 /*!
     @brief  Low-level CSN signal enable/disable
     @param  level      output signal level
 */
 /**************************************************************************/
void NRF24::csn(uint8_t val)
{
  digitalWrite(_csn, val);
}

/**************************************************************************/
/*!
    @brief  Low-level CE signal enable/disable
    @param  level      output signal level
*/
/**************************************************************************/
void NRF24::ce(uint8_t val)
{
  digitalWrite(_ce, val);
}

/**************************************************************************/
/*!
    @brief  Low-level SPI write wrapper
    @param  c       8-bit command to write to the SPI bus
    @returns  Data received while transaction
*/
/**************************************************************************/
uint8_t NRF24::spi_transfer(uint8_t c)
{
    uint8_t rxValue;

    csn(LOW);
    rxValue = SPI.transfer(c);
    csn(HIGH);

    return rxValue;
}

/**************************************************************************/
/*!
    @brief  Low-level SPI write wrapper
    @param  buf     Array of data to be transferred
    @param  count   Array data length
*/
/**************************************************************************/
void NRF24::spi_transfer(void *buf, size_t count)
{
    csn(LOW);
    SPI.transfer(buf, count);
    csn(HIGH);
}

/**
 * Command functions
 */

/**************************************************************************/
/*!
    @brief  Read nRF24 register
    @param  reg     Register to read
    @returns  Register content
*/
/**************************************************************************/
uint8_t NRF24::read_register(uint8_t reg)
{
    uint8_t cmd[2] = {R_REGISTER | (REGISTER_MASK & reg), 0xff};

    csn(LOW);
    spi_transfer(cmd, 2);
    csn(HIGH);

    return cmd[1];
}

/**************************************************************************/
/*!
    @brief  Read nRF24 multi-byte register
    @param  reg     Register to read
    @param  buf     Array of data received
    @param  len     Array length
*/
/**************************************************************************/
void NRF24::read_register_multi(uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t cmd = R_REGISTER | (REGISTER_MASK & reg);

    for (int i = 0; i < len; ++i)
    {
        buf[i] = 0xff;
    }

    csn(LOW);
    spi_transfer(&cmd, 1);
    spi_transfer(buf, len);
    csn(HIGH);
}

/**************************************************************************/
/*!
    @brief  Write nRF24 register
    @param  reg       Register to write
    @param  value     Data to write
*/
/**************************************************************************/
void NRF24::write_register(uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {W_REGISTER | (REGISTER_MASK & reg), value};

    csn(LOW);
    spi_transfer((uint8_t*)cmd, 2);
    csn(HIGH);
}

/**************************************************************************/
/*!
    @brief  Write nRF24 multi-byte register
    @param  reg     Register to write
    @param  buf     Array of data to write
    @param  len     Array length
*/
/**************************************************************************/
void NRF24::write_register_multi(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    uint8_t cmd = W_REGISTER | (REGISTER_MASK & reg);

    csn(LOW);
    spi_transfer(&cmd, 1);
    spi_transfer((uint8_t*)buf, len);
    csn(HIGH);
}


