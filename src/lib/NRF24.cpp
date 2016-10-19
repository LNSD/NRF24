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

//endregion

/**
 *  Low-level signal & SPI-specific functions
 */

//region Low-level signal & SPI-specific functions

/**
 * Low-level CSN signal enable/disable
 * @param level Output signal level
 */
inline void NRF24::csn(uint8_t val)
{
    digitalWrite(_csn, val);
}

/**
 * Low-level CE signal enable/disable
 * @param level Output signal level
 */
inline void NRF24::ce(uint8_t val)
{
    digitalWrite(_ce, val);
}

/**
 * Low-level SPI transfer wrapper
 * @param byte Byte to be written to the SPI bus
 * @return Data received while transaction
 */
inline uint8_t NRF24::spiTransfer(uint8_t byte)
{
    uint8_t rxValue;

    csn(LOW);
    rxValue = SPI.transfer(byte);
    csn(HIGH);

    return rxValue;
}

/**
 * Low-level SPI transfer wrapper
 * @param buf Array of data to be transferred
 * @param count Array data length
 */
inline void NRF24::spiTransfer(void *buf, size_t count)
{
    csn(LOW);
    SPI.transfer(buf, count);
    csn(HIGH);
}

/**
 * Low-level SPI command wrapper
 * @param cmd Preceding SPI command
 * @param buf Array of data to be transferred
 * @param len Array data length
 */
inline void NRF24::spiCmdTransfer(uint8_t cmd, void *buf, size_t len)
{
    csn(LOW);
    SPI.transfer(cmd);
    SPI.transfer(buf, len);
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
uint8_t NRF24::readRegister(uint8_t reg)
{
    uint8_t cmd[2] = {R_REGISTER | (REGISTER_MASK & reg), 0xff};  // TODO Check if necessary

    spiTransfer(cmd, 2);

    return cmd[1];
}

/**
 * Read nRF24 multi-byte register
 * @param reg Register to read
 * @param buf Array of data received
 * @param len Array length
*/
void NRF24::readRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t cmd = R_REGISTER | (REGISTER_MASK & reg);

    // TODO Check if necessary
    for (int i = 0; i < len; ++i)
    {
        buf[i] = 0xff;
    }

    spiCmdTransfer(cmd, buf, len);
}

/**
 * Write nRF24 register
 * @param reg Register to write
 * @param value Data to write
 */
void NRF24::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {W_REGISTER | (REGISTER_MASK & reg), value};

    spiTransfer(cmd, 2);
}

/**
 * Write nRF24 multi-byte register
 * @param reg Register to write
 * @param buf Array of data to write
 * @param len Array length
 */
void NRF24::writeRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t cmd = W_REGISTER | (REGISTER_MASK & reg);

    spiCmdTransfer(cmd, buf, len);
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

/**
 * Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
 * @Note Flush RX FIFO if the read value is larger than 32 bytes
 * @return RX payload width
 */
uint8_t NRF24::getRxPayloadLength()
{
    uint8_t width = 0xff; // TODO Check if necessary


    spiCmdTransfer(R_RX_PL_WID, &width, 1);

    return width;
}

/**
 * Read RX-payload: 1 - 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
 * @param data Payload buffer
 * @param len Payload length
 */
void NRF24::readRxPayload(uint8_t* data, uint8_t len)
{
    // TODO Check if necessary
    for (int i = 0; i < len; ++i)
    {
        data[i] = 0xff;
    }

    spiCmdTransfer(R_RX_PAYLOAD, data, len);
}

/**
 * Write TX-payload: 1 - 32 bytes. A read operation always starts at byte 0 used in TX payload.
 * @param data Payload buffer
 * @param len Payload length
 */
void NRF24::writeTxPayload(uint8_t* data, uint8_t len)
{
    spiCmdTransfer(W_TX_PAYLOAD, data, len);
}

/**
 * Used in RX mode. Write Payload to be transmitted together with ACK packet on chosen pipe.
 * @note Maximum three ACK packet payloads can be pending. Payloads with same # are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0.
 * @param pipe Pipe where payload is written
 * @param data Payload buffer
 * @param len Payload length
 */
void NRF24::writePipeACKPayload(NRF24_RxPipe_t pipe, uint8_t* data, uint8_t len)
{
    spiCmdTransfer((uint8_t) (W_ACK_PAYLOAD | (pipe & 0x07)), data, len);
}

/**
 * Used in TX mode. Disables AUTOACK on this specific packet.
 */
void NRF24::disableAAforPayload()
{
    spiTransfer(W_TX_PAYLOAD_NOACK);
}

/**
 * Used for a PTX device. Reuse last transmitted payload.
 * @note TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deacti- vated during package transmission.
 */
void NRF24::reuseTxPayload()
{
    spiTransfer(REUSE_TX_PL);
}

/**
 * Flush TX FIFO, used in TX mode.
 */
void NRF24::flushTXFIFO()
{
    spiTransfer(FLUSH_TX);
}

/**
 * Flush RX FIFO, used in RX mode.
 * @note Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
 */
void NRF24::flushRXFIFO()
{
    spiTransfer(FLUSH_RX);
}

//endregion

/**
 * Util functions
 */

//region Util functions

//endregion