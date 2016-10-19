#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <stdio.h>

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

/**
 * Parse byte bits into boolean array
 * @param bit Boolean bit array
 * @param byte Byte to parse
 */
void inline NRF24::parseToBoolean(boolean *bit, uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        bit[i] = (byte & _BV(i)) > 0;
    }
}

/**
 * Parse CONFIG register content and show debug info
 * @param content Register content
 */
void NRF24::debugConfigRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: CONFIG register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // MASK_RX_DR bit
    Serial.print("\t - MASK_RX_DR: ");
    Serial.println(bit[MASK_RX_DR], BIN);

    // MASK_TX_DS bit
    Serial.print("\t - MASK_TX_DS: ");
    Serial.println(bit[MASK_TX_DS], BIN);

    // MASK_MAX_RT bit
    Serial.print("\t - MASK_MAX_RT: ");
    Serial.println(bit[MASK_MAX_RT], BIN);

    // EN_CRC bit
    Serial.print("\t - EN_CRC: ");
    Serial.println(bit[EN_CRC], BIN);

    // CRCO bit
    Serial.print("\t - CRCO: ");
    Serial.println(bit[CRCO], BIN);

    // PWR_UP bit
    Serial.print("\t - PWR_UP: ");
    Serial.println(bit[PWR_UP], BIN);

    // PRIM_RX bit
    Serial.print("\t - PRIM_RX: ");
    Serial.println(bit[PRIM_RX], BIN);
}

/**
 * Parse EN_AA register content and show debug info
 * @param content Register content
 */
void NRF24::debugEnAARegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: EN_AA register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // ENAA_P5 bit
    Serial.print("\t - ENAA_P5: ");
    Serial.println(bit[ENAA_P5], BIN);

    // ENAA_P4 bit
    Serial.print("\t - ENAA_P4: ");
    Serial.println(bit[ENAA_P4], BIN);

    // ENAA_P3 bit
    Serial.print("\t - ENAA_P3: ");
    Serial.println(bit[ENAA_P3], BIN);

    // ENAA_P2 bit
    Serial.print("\t - ENAA_P2: ");
    Serial.println(bit[ENAA_P2], BIN);

    // ENAA_P1 bit
    Serial.print("\t - ENAA_P1: ");
    Serial.println(bit[ENAA_P1], BIN);

    // ENAA_P0 bit
    Serial.print("\t - ENAA_P0: ");
    Serial.println(bit[ENAA_P0], BIN);
}

/**
 * Parse EN_RXADDR register content and show debug info
 * @param content Register content
 */
void NRF24::debugEnRxAddrRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: EN_RXADDR register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // ERX_P5 bit
    Serial.print("\t - ERX_P5: ");
    Serial.println(bit[ERX_P5], BIN);

    // ERX_P4 bit
    Serial.print("\t - ERX_P4: ");
    Serial.println(bit[ERX_P4], BIN);

    // ERX_P3 bit
    Serial.print("\t - ERX_P3: ");
    Serial.println(bit[ERX_P3], BIN);

    // ERX_P2 bit
    Serial.print("\t - ERX_P2: ");
    Serial.println(bit[ERX_P2], BIN);

    // ERX_P1 bit
    Serial.print("\t - ERX_P1: ");
    Serial.println(bit[ERX_P1], BIN);

    // ERX_P0 bit
    Serial.print("\t - ERX_P0: ");
    Serial.println(bit[ERX_P0], BIN);
}

/**
 * Parse SETUP_AW register content and show debug info
 * @param content Register content
 */
void NRF24::debugSetupAWRegister(uint8_t content)
{
    // Debug info header
    Serial.print(" - DEBUG: SETUP_AW register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // AW bits
    uint8_t aw = (_BV(AW) | _BV(AW+1)) & content;

    Serial.print("\t - AW (0x");
    Serial.print(aw, HEX);
    Serial.print("): ");
    Serial.println(aw, BIN);
}

/**
 * Parse SETUP_RETR register content and show debug info
 * @param content Register content
 */
void NRF24::debugSetupRetrRegister(uint8_t content)
{
    // Debug info header
    Serial.print(" - DEBUG: SETUP_RETR register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // ARD bits
    uint8_t ard = ((_BV(ARD) | _BV(ARD+1) | _BV(ARD+2) | _BV(ARD+3)) & content) >> ARD;

    Serial.print("\t - ARD (0x");
    Serial.print(ard, HEX);
    Serial.print("): ");
    Serial.println(ard, BIN);

    // ARC bits
    uint8_t arc = (_BV(ARC) | _BV(ARC+1) | _BV(ARC+2) | _BV(ARC+3)) & content;

    Serial.print("\t - ARC (0x");
    Serial.print(arc, HEX);
    Serial.print("): ");
    Serial.println(arc, BIN);
}

/**
 * Parse RF_CH register content and show debug info
 * @param content Register content
 */
void NRF24::debugRfChRegister(uint8_t content)
{
    // Debug info header
    Serial.print(" - DEBUG: RF_CH register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // AW bits
    Serial.print("\t - RF_CH (0x");
    Serial.print(content, HEX);
    Serial.print("): ");
    Serial.println(content, BIN);
}

/**
 * Parse RF_SETUP register content and show debug info
 * @param content Register content
 */
void NRF24::debugRfSetupRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: RF_SETUP register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // CONT_WAVE bit
    Serial.print("\t - CONT_WAVE: ");
    Serial.println(bit[CONT_WAVE], BIN);

    // PLL_LOCK bit
    Serial.print("\t - PLL_LOCK: ");
    Serial.println(bit[PLL_LOCK], BIN);

    // RF_DR bits
    uint8_t rfdr = ((_BV(RF_DR_HIGH) & content) >> RF_DR_HIGH) | ((_BV(RF_DR_LOW) & content) >> RF_DR_LOW-1);

    Serial.print("\t - RF_DR (0x");
    Serial.print(rfdr, HEX);
    Serial.print("): ");
    Serial.println(rfdr, BIN);

    // RF_PWR bits
    uint8_t rfpwr = ((_BV(RF_PWR) | _BV(RF_PWR+1)) & content) >> RF_PWR;

    Serial.print("\t - RF_PWR (0x");
    Serial.print(rfpwr, HEX);
    Serial.print("): ");
    Serial.println(rfpwr, BIN);
}

/**
 * Parse STATUS register content and show debug info
 * @param content Register content
 */
void NRF24::debugStatusRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: STATUS register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // RX_DR bit
    Serial.print("\t - RX_DR: ");
    Serial.println(bit[RX_DR], BIN);

    // TX_DS bit
    Serial.print("\t - TX_DS: ");
    Serial.println(bit[TX_DS], BIN);

    // MAX_RT bit
    Serial.print("\t - MAX_RT: ");
    Serial.println(bit[MAX_RT], BIN);

    // RX_P_NO bits
    uint8_t rxpno = ((_BV(RX_P_NO) | _BV(RX_P_NO+1) | _BV(RX_P_NO+2)) & content) >> RX_P_NO;

    Serial.print("\t - RX_P_NO (0x");
    Serial.print(rxpno, HEX);
    Serial.print("): ");
    Serial.println(rxpno, BIN);

    // TX_FULL bit
    Serial.print("\t - TX_FULL: ");
    Serial.println(bit[TX_FULL], BIN);
}

/**
 * Parse OBSERVE_TX register content and show debug info
 * @param content Register content
 */
void NRF24::debugObserveTxRegister(uint8_t content)
{
    // Debug info header
    Serial.print(" - DEBUG: OBSERVE_TX register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // PLOS_CNT bits
    uint8_t ploscnt = ((_BV(PLOS_CNT) | _BV(PLOS_CNT+1) | _BV(PLOS_CNT+2) | _BV(PLOS_CNT+3)) & content) >> PLOS_CNT;

    Serial.print("\t - PLOS_CNT (0x");
    Serial.print(ploscnt, HEX);
    Serial.print("): ");
    Serial.println(ploscnt, BIN);

    // ARC_CNT bits
    uint8_t arccnt = (_BV(ARC_CNT) | _BV(ARC_CNT+1) | _BV(ARC_CNT+2) | _BV(ARC_CNT+3)) & content;

    Serial.print("\t - ARC_CNT (0x");
    Serial.print(arccnt, HEX);
    Serial.print("): ");
    Serial.println(arccnt, BIN);
}

/**
 * Parse RPD register content and show debug info
 * @param content Register content
 */
void NRF24::debugRPDRegister(uint8_t content)
{
    // Debug info header
    Serial.print(" - DEBUG: RPD register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // RPD bit
    Serial.print("\t - RPD: ");
    Serial.println(_BV(RPD_BIT) & content, BIN);
}

/**
 * Parse RX_PW_P# registers content and show debug info
 * @param content Register content
 * @param pipe Pipe number
 */
void NRF24::debugRxBytesPipeRegister(uint8_t content, uint8_t pipe)
{
    // Debug info header
    Serial.print(" - DEBUG: RX_PW_P");
    Serial.print(pipe, DEC);
    Serial.print(" register content (0x");
    Serial.print(content, HEX);
    Serial.print("): ");
    Serial.println(content, BIN);
}

/**
 * Parse FIFO_STATUS register content and show debug info
 * @param content Register content
 */
void NRF24::debugFIFOStatusRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: FIFO_STATUS register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // TX_REUSE bit
    Serial.print("\t - TX_REUSE: ");
    Serial.println(bit[TX_REUSE], BIN);

    // TX_FULL bit
    Serial.print("\t - TX_FULL: ");
    Serial.println(bit[FIFO_FULL], BIN);

    // TX_EMPTY bit
    Serial.print("\t - TX_EMPTY: ");
    Serial.println(bit[TX_EMPTY], BIN);

    // RX_FULL bit
    Serial.print("\t - RX_FULL: ");
    Serial.println(bit[RX_FULL], BIN);

    // RX_EMPTY bit
    Serial.print("\t - RX_EMPTY: ");
    Serial.println(bit[RX_EMPTY], BIN);
}

/**
 * Parse DYNPD register content and show debug info
 * @param content Register content
 */
void NRF24::debugDYNPDRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: DYNPD register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // DPL_P# bits
    for (int i = 0; i < 5; ++i) {
        Serial.print("\t - DPL_P");
        Serial.print(i, DEC);
        Serial.print(": ");
        Serial.println(bit[i], BIN);
    }
}

/**
 * Parse FEATURE register content and show debug info
 * @param content Register content
 */
void NRF24::debugFeatureRegister(uint8_t content)
{
    boolean bit[8] = {false, false, false, false, false, false, false, false};
    parseToBoolean(bit, content);

    // Debug info header
    Serial.print(" - DEBUG: FEATURE register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // EN_DPL bit
    Serial.print("\t - EN_DPL: ");
    Serial.println(bit[EN_DPL], BIN);

    // EN_ACK_PAY bit
    Serial.print("\t - EN_ACK_PAY: ");
    Serial.println(bit[EN_ACK_PAY], BIN);

    // EN_DYN_ACK bit
    Serial.print("\t - EN_DYN_ACK: ");
    Serial.println(bit[EN_DYN_ACK], BIN);
}

//endregion