/**
 * NRF24.cpp
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

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
    // SPI configuration
    SPI.begin();

    // Minimum ideal SPI bus speed is 2x data rate
    // If we assume 2Mbps data rate and 16Mhz clock,
    // a divider of 4 is the minimum we want.
    // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
    // SPI.setBitOrder(MSBFIRST);
    // SPI.setDataMode(SPI_MODE0);
    // SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.beginTransaction(SPISettings((uint32_t) 4000000, MSBFIRST, SPI_MODE0));

    // Interface configure
    ce(LOW);
    csn(HIGH);

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay( 5 ) ;
}

/**
 * Setups hardware
 * @param configuration NRF24 configuration holder
 */
void NRF24::configure(Config configuration)
{
    // SPI configuration
    SPI.begin();

    // Minimum ideal SPI bus speed is 2x data rate
    // If we assume 2Mbps data rate and 16Mhz clock,
    // a divider of 4 is the minimum we want.
    // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
    // SPI.setBitOrder(MSBFIRST);
    // SPI.setDataMode(SPI_MODE0);
    // SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.beginTransaction(SPISettings((uint32_t) 4000000, MSBFIRST, SPI_MODE0));

    // Interface configure
    ce(LOW);
    csn(HIGH);

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay( 5 ) ;

    // Set configuration to NRF24
    setTransceiverMode(configuration._mode);
    setOutputRfPower(configuration._power);
    setDataRate(configuration._dataRate);
    setRfChannel(configuration._rfCh);

    if (configuration._constCarrier) {
        enableConstantCarrier();
    } else {
        disableConstantCarrier();
    }

    if (configuration._pllLock) {
        forcePllLock();
    } else {
        disablePllLock();
    }

    setAddrLength(configuration._addrWidth);

    for (int p = 0; p < 6; p++) {
        if (configuration._rxPipeAddrStatus[p]) {
            enablePipeRxAddr((NRF24::RxPipe) p);
        } else {
            disablePipeRxAddr((NRF24::RxPipe) p);
        }
    }

    for (int p = 0; p < 6; p++) {
        setRxPipePayloadSize((NRF24::RxPipe) p, configuration._rxPipePayloadSize[p]);
    }

    for (int p = 0; p < 6; p++) {
        if (p < 2) {
            setRxPipeAddr((NRF24::RxPipe) p, configuration._rxPipeAddrLong[p], configuration._addrWidth);
        } else {
            setRxPipeAddr((NRF24::RxPipe) p, &configuration._rxPipeAddrShort[p - 2], 1);
        }
    }

    setTxAddr(configuration._txAddr, configuration._addrWidth);

    for(int p = 0; p < 6; p++)
    {
        if (configuration._autoAck) {
            enableRxPipeAutoAck((NRF24::RxPipe) p);
        } else {
            disableRxPipeAutoAck((NRF24::RxPipe) p);
        }
    }

    if (configuration._crc != CRC_DISABLED) {
        enableCRC(configuration._crc);
    } else {
        disableCRC();
    }

    setAutoRtDelay(configuration._autoRtDelay);
    setAutoRtCount(configuration._autoRtCount);

    for (int p = 0; p < 6; p++) {
        if (configuration._dynamicPayload[p]) {
            enableRxPipeDynamicPayloads((NRF24::RxPipe) p);
        } else {
            disableRxPipeDynamicPayloads((NRF24::RxPipe) p);
        }
    }

    if (configuration._ackPayload) {
        enableAckPayload();
    } else {
        disableAckPayload();
    }

    if (configuration._dynamicAck) {
        enableDynamicAck();
    } else {
        disableDynamicAck();
    }

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
 * Low-level SPI command wrapper
 * @param cmd SPI command
 */
inline void NRF24::spiCmdTransfer(uint8_t cmd)
{
    csn(LOW);
    SPI.transfer(cmd);
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
    uint8_t data;

    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), &data, 1);

    return data;
}

/**
 * Read nRF24 multi-byte register
 * @param reg Register to read
 * @param buf Array of data received
 * @param len Array length
*/
void NRF24::readRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), buf, len);
}

/**
 * Write nRF24 register
 * @param reg Register to write
 * @param value Data to write
 */
void NRF24::writeRegister(uint8_t reg, uint8_t value)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), &value, 1);
}

/**
 * Write nRF24 multi-byte register
 * @param reg Register to write
 * @param buf Array of data to write
 * @param len Array length
 */
void NRF24::writeRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), buf, len);
}

//endregion

/**
 * Configuration functions. Getters and setters
 */

//region Configuration functions. Getters and setters

/**
 * Set transceiver mode
 * @param mode Transceiver mode
 */
void NRF24::setTransceiverMode(TransceiverMode mode)
{
    uint8_t config = readRegister(CONFIG);

    if(mode == Mode_PTX)
    {
        config &= ~_BV(PRIM_RX);
    }
    else
    {
        config |= _BV(PRIM_RX);
    }

    writeRegister(CONFIG, config);
}

/**
 * Get current transceiver mode
 * @return Transceiver mode
 */
NRF24::TransceiverMode NRF24::getTransceiverMode()
{
    uint8_t result = readRegister(CONFIG) & _BV(PRIM_RX);

    if(result == Mode_PTX)
    {
        return Mode_PTX;
    }
    else
    {
        return Mode_PRX;
    }
}

/**
 * Enable constant carrier
 */
void NRF24::enableConstantCarrier()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup |= _BV(CONT_WAVE);

    writeRegister(RF_SETUP, rfsetup);
}

/**
 * Disable constant carrier
 */
void NRF24::disableConstantCarrier()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup &= ~_BV(CONT_WAVE);

    writeRegister(RF_SETUP, rfsetup);
}

/**
 * Check if constant carrier is enabled
 */
bool NRF24::isConstantCarrierEnabled()
{
    return (bool)(readRegister(RF_SETUP) & _BV(CONT_WAVE));
}

/**
 * Force PLL Lock signal
 * @note Only used in test
 */
void NRF24::forcePllLock()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup |= _BV(PLL_LOCK);

    writeRegister(RF_SETUP, rfsetup);
}

/**
 * Disable PLL Lock signal
 */
void NRF24::disablePllLock()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup &= ~_BV(PLL_LOCK);

    writeRegister(RF_SETUP, rfsetup);
}

/**
 * Check if PLL Lock is forced
 */
bool NRF24::isPllLockForced()
{
    return (readRegister(RF_SETUP) & _BV(PLL_LOCK)) > 0;
}

/**
 * Set transceiver's output power level
 * @param level Output power level
 */
void NRF24::setOutputRfPower(OutputPower level)
{
    uint8_t setup = readRegister(RF_SETUP) ;
    setup &= ~(_BV(RF_PWR) | _BV(RF_PWR+1));

    switch(level)
    {
        case OutputPower_0dBm:
            setup |= (_BV(RF_PWR) | _BV(RF_PWR+1));
            break;
        case OutputPower_M6dBm:
            setup |= _BV(RF_PWR+1);
            break;
        case OutputPower_M12dBm:
            setup |= _BV(RF_PWR);
            break;
        case OutputPower_M18dBm:
            break;
    }

    writeRegister(RF_SETUP, setup);
}

/**
 * Get current transceiver's output power level
 * @return Current output power
 */
NRF24::OutputPower NRF24::getOutputRfPower()
{
    OutputPower result = OutputPower_0dBm;
    uint8_t power = readRegister(RF_SETUP) & (_BV(RF_PWR) | _BV(RF_PWR+1));

    if (power == (_BV(RF_PWR) | _BV(RF_PWR+1)))
    {
        result = OutputPower_0dBm ;
    }
    else if(power == _BV(RF_PWR+1))
    {
        result = OutputPower_M6dBm ;
    }
    else if(power == _BV(RF_PWR))
    {
        result = OutputPower_M12dBm ;
    }
    else
    {
        result = OutputPower_M18dBm ;
    }

    return result;
}

/**
 * Set transceiver's datarate
 * @param rate Datarate
 */
void NRF24::setDataRate(DataRate rate)
{
    uint8_t setup = readRegister(RF_SETUP);
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    switch(rate)
    {
        case DataRate_250kbps:
            setup |= _BV(RF_DR_LOW);
            break;
        case DataRate_2Mbps:
            setup |= _BV(RF_DR_HIGH);
            break;
        case DataRate_1Mbps:
            break;
    }

    writeRegister(RF_SETUP, setup);
}

/**
 * Get current transceiver's datarate
 * @return Current datarate
 */
NRF24::DataRate NRF24::getDataRate()
{
    uint8_t dr = readRegister(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    if(dr == _BV(RF_DR_LOW))
    {
        // '10' = 250Kbps
        return DataRate_250kbps;
    }
    else if(dr == _BV(RF_DR_HIGH))
    {
        // '01' = 2Mbps
        return DataRate_2Mbps;
    }
    else if (dr == 0)
    {
        // '00' = 1Mbps
        return DataRate_1Mbps;
    }

    return DataRate_2Mbps;
}

/**
 * Set transceiver's RF channel
 * @param channel RF channel
 */
void NRF24::setRfChannel(uint8_t channel)
{
    const uint8_t max_channel = 127;
    writeRegister(RF_CH, min(channel, max_channel));
}

/**
 * Get transceiver's current RF channel
 * @return Current RF channel
 */
uint8_t NRF24::getRfChannel()
{
    return readRegister(RF_CH);
}
/**
 * Set address length. Address length: 3 - 5 bytes.
 * @param length Address length
 */
void NRF24::setAddrLength(uint8_t length)
{
    if(length >= 3  && length <= 5)
    {
        writeRegister(SETUP_AW, length-2);
    }
}

/**
 * Get current address length
 * @return Current address length
 */
uint8_t NRF24::getAddrLength()
{
    return readRegister(SETUP_AW) + 2;
}

/**
 * Enable RX pipe
 * @param pipe RX pipe
 */
void NRF24::enablePipeRxAddr(RxPipe pipe)
{
    writeRegister(EN_RXADDR, readRegister(EN_RXADDR) | _BV(pipe));
}

/**
 * Disable RX pipe
 * @param pipe RX pipe
 */
void NRF24::disablePipeRxAddr(RxPipe pipe)
{
    writeRegister(EN_RXADDR, readRegister(EN_RXADDR) & ~_BV(pipe));
}

/**
 * Get list of RX pipes status
 * @param pipes Boolean array holding status of all the RX pipes
 */
void NRF24::whichRxAddrAreEnabled(bool *pipes)
{
    uint8_t enRxAddr = readRegister(EN_RXADDR);
    for (int p = 0; p < 6; ++p)
    {
        pipes[p] = (bool) (enRxAddr & _BV(p));
    }
}

/**
 * Set destination address (TX address)
 * @param addr Address
 * @param len Address length
 */
void NRF24::setTxAddr(uint8_t *addr, uint8_t len)
{
    writeRegister(TX_ADDR, addr, len);
}

/**
 * Get current destination address (TX address)
 * @param addr Current address
 * @param len Adress length
 */
void NRF24::getTxAddr(uint8_t *addr, uint8_t len)
{
    readRegister(TX_ADDR, addr, len);
}

const uint8_t pipeRx[6] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

/**
 * Set input pipe address (RX pipe address)
 * @param pipe RX pipe
 * @param addr Pipe address
 * @param len Address length
 */
void NRF24::setRxPipeAddr(RxPipe pipe, uint8_t *addr, uint8_t len)
{
    if(pipe < 2)
    {
        writeRegister(pipeRx[pipe], addr, len);
    }
    else
    {
        writeRegister(pipeRx[pipe], addr[0]);
    }
}

/**
 * Get current input pipe address (RX pipe address)
 * @param pipe RX pipe
 * @param addr Pipe address
 * @param len Address length
 */
void NRF24::getRxPipeAddr(RxPipe pipe, uint8_t *addr, uint8_t len)
{
    if(pipe < 2)
    {
        readRegister(pipeRx[pipe], addr, len);
    }
    else
    {
        addr[0] = readRegister(pipeRx[pipe]);
    }
}

const uint8_t pipe_payload[6] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};

/**
 * Set input pipe payload size. Max payload size: 32 bytes
 * @param pipe RX pipe
 * @param size Payload size
 */
void NRF24::setRxPipePayloadSize(RxPipe pipe, uint8_t size)
{
    const uint8_t max_size = 32;
    writeRegister(pipe_payload[pipe], min(size, max_size));
}

/**
 * Get current input pipe payload size
 * @param pipe pipe
 * @return Payload size
 */
uint8_t NRF24::getRxPipePayloadSize(RxPipe pipe)
{
    return readRegister(pipe_payload[pipe]);
}

/**
 * Enable input pipe dynamic payloads
 * @param pipe RX pipe
 */
void NRF24::enableRxPipeDynamicPayloads(RxPipe pipe)
{
    uint8_t dynpd = readRegister(DYNPD);
    dynpd |= _BV(pipe);

    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_DPL));
    writeRegister(DYNPD, dynpd);
}

/**
 * Disable input pipe dynamic payloads
 * @param pipe RX pipe
 */
void NRF24::disableRxPipeDynamicPayloads(RxPipe pipe)
{
    uint8_t dynpd = readRegister(DYNPD);

    if(!(dynpd & ~_BV(pipe))) {
        writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_DPL));
    }

    writeRegister(DYNPD, dynpd & ~_BV(pipe));
}

/**
 * Disable all pipes dynamic payloads
 */
void NRF24::disableDynamicPayloads()
{
    writeRegister(DYNPD, 0x00);
}

/**
 * Check which inout pipe dynamic payloads are enabled
 * @param dynamicPayloads RX pipe dynamic payloads status
 */
void NRF24::whichRxPipeDynamicPayloadsAreEnabled(bool *dynamicPayloads)
{
    if(readRegister(FEATURE) & _BV(EN_DPL))
    {
        uint8_t dPayloads = readRegister(DYNPD);
        for (int p = 0; p < 6; ++p)
        {
            bool dynp = (bool) (dPayloads & _BV(p));
            dynamicPayloads[p] = dynp;
        }
    }
    else
    {
        const bool defaultDynPayloads[6] = {false,false,false,false,false,false};
        memcpy(dynamicPayloads, defaultDynPayloads, sizeof(defaultDynPayloads));
    }
}

/**
 * Enable CRC and set length
 * @param length CRC length
 */
void NRF24::enableCRC(CRCLength length)
{
    uint8_t config = readRegister(CONFIG);
    config |= _BV(EN_CRC);

    if(length == CRC_16)
    {
        config |= _BV(CRCO);
    }
    else
    {
        config &= ~_BV(CRCO);
    }

    writeRegister(CONFIG, config);
}

/**
 * Disable CRC
 */
void NRF24::disableCRC()
{
    uint8_t config = readRegister(CONFIG);
    config &= ~_BV(EN_CRC);
    writeRegister(CONFIG, config);
}

/**
 * Get current CRC configuration
 * @return Current CRC configuration
 */
NRF24::CRCLength NRF24::getCRCConfig()
{
    uint8_t config = readRegister(CONFIG);
    if(config & _BV(EN_CRC))
    {
        if(config & _BV(CRCO))
        {
            return CRC_16;
        }
        else
        {
            return CRC_8;
        }
    }
    else
    {
        return CRC_DISABLED;
    }
}

/**
 * Enable input pipe auto ACK
 * @param pipe RX pipe
 */
void NRF24::enableRxPipeAutoAck(RxPipe pipe)
{
    writeRegister(EN_AA, readRegister(EN_AA) | _BV(pipe));
}

/**
 * Disable input pipe auto ACK
 * @param pipe RX pipe
 */
void NRF24::disableRxPipeAutoAck(RxPipe pipe)
{
    writeRegister(EN_AA, readRegister(EN_AA) & ~_BV(pipe));
}

/**
 * Get which input pipes have auto ACK enabled
 * @param autoAck Boolean array holding status of RX pipes auto ACK
 */
void NRF24::whichRxPipeAutoAckAreEnabled(bool *autoAck)
{
    uint8_t autoack = readRegister(EN_AA);
    for (int p = 0; p < 6; ++p)
    {
        autoAck[p] = (bool) (autoack & _BV(p));
    }
}

/**
 * Set autoretransmission delay
 * @param delay Autoretransmission delay
 */
void NRF24::setAutoRtDelay(uint16_t delay)
{
    const uint16_t min_delay = 250;
    const uint16_t max_delay = 4000;

    uint8_t setupRetr = readRegister(SETUP_RETR);
    setupRetr &= 0x0F;

    if (delay < min_delay) {
        setupRetr |= 0x00;
    } else if (delay > max_delay) {
        setupRetr |= 0xF0;
    } else {
        setupRetr |= (delay/250 - 1) << 4;
    }

    writeRegister(SETUP_RETR, setupRetr);
}

/**
 * Get current autoretransmission delay
 * @return Current autoretranmission delay
 */
uint8_t NRF24::getAutoRtDelay()
{
    return (readRegister(SETUP_RETR) >> 4);
}

/**
 * Set max autoretransmission retries
 * @param count Max autoretransmission retries
 */
void NRF24::setAutoRtCount(uint8_t count)
{
    const uint8_t max_count = 0xF;
    uint8_t setupRetr = readRegister(SETUP_RETR);
    setupRetr &= 0xF0;
    setupRetr |= min(count, max_count);

    writeRegister(SETUP_RETR, setupRetr);
}

/**
 * Get current max autoretransmission retries
 * @return Current max autoretransmission retries
 */
uint8_t NRF24::getAutoRtCount()
{
    return readRegister(SETUP_RETR) & 0x0F;
}

/**
 * Enable ACK payload
 */
void NRF24::enableAckPayload()
{
    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_ACK_PAY));
}

/**
 * Disable ACK payload
 */
void NRF24::disableAckPayload()
{
    writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_ACK_PAY));
}

/**
 * Get current ACK payload configuration
 */
bool NRF24::isAckPayloadEnabled()
{
    return (bool)(readRegister(FEATURE) & _BV(EN_ACK_PAY));
}

/**
 * Enable dynamic ACK
 */
void NRF24::enableDynamicAck()
{
    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_DYN_ACK));
}

/**
 * Disable dynamic ACK
 */
void NRF24::disableDynamicAck()
{
    writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_DYN_ACK));
}

/**
 * Get current dynamic ACK configuration
 */
bool NRF24::isDynamicAckEnabled()
{
    return (bool)(readRegister(FEATURE) & _BV(EN_DYN_ACK));
}

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
    uint8_t width;

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
void NRF24::writePipeACKPayload(RxPipe pipe, uint8_t* data, uint8_t len)
{
    spiCmdTransfer((uint8_t) (W_ACK_PAYLOAD | (pipe & 0x07)), data, len);
}

/**
 * Used in TX mode. Disables AUTOACK on this specific packet.
 */
void NRF24::disableAAforPayload()
{
    spiCmdTransfer(W_TX_PAYLOAD_NOACK);
}

/**
 * Used for a PTX device. Reuse last transmitted payload.
 * @note TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deacti- vated during package transmission.
 */
void NRF24::reuseTxPayload()
{
    spiCmdTransfer(REUSE_TX_PL);
}

/**
 * Flush TX FIFO, used in TX mode.
 */
void NRF24::flushTXFIFO()
{
    spiCmdTransfer(FLUSH_TX);
}

/**
 * Flush RX FIFO, used in RX mode.
 * @note Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
 */
void NRF24::flushRXFIFO()
{
    spiCmdTransfer(FLUSH_RX);
}

//endregion

/**
 * Get status functions
 */

//region Status functions

/**
 * Get count of lost packets. The counter is overflow pro- tected to 15, and discontinues at max until reset.
 * @note The counter is reset by writing to RF_CH.
 * @return Number of packets
 */
uint8_t NRF24::getLostPacketsCount()
{
    return (readRegister(OBSERVE_TX) & 0xF0) >> 4;
}

/**
 * Get count of retransmitted packets. The counter is reset when transmission of a new packet starts.
 * @return Number of packets
 */
uint8_t NRF24::getRtCount()
{
    return readRegister(OBSERVE_TX) & 0x0F;
}

/**
 * Check if carrier is detected
 * @note nRF24L01+ must be in receive mode.
 */
bool NRF24::isCarrierDetected()
{
    return (bool) (readRegister(RPD) & 0x01);
}

/**
 * Check if TX payload reuse is active (It should be active until W_TX_PAYLOAD or FLUSH TX is executed).
 */
bool NRF24::isReuseTxPayloadActive()
{
    return (bool) (readRegister(FIFO_STATUS) & _BV(TX_REUSE));
}

/**
 * Get current TX FIFO status
 * @return FIFO status
 */
NRF24::FIFOStatus NRF24::getTxFifoStatus()
{
    uint8_t fifo_status = readRegister(FIFO_STATUS);
    if(fifo_status & _BV(TX_FULL))
        return FIFO_STATUS_FULL;
    else if(fifo_status & _BV(TX_EMPTY))
        return  FIFO_STATUS_EMPTY;
    else
        return FIFO_STATUS_OK;
}

/**
 * Get current RX FIFO status
 * @return FIFO status
 */
NRF24::FIFOStatus NRF24::getRxFifoStatus()
{
    uint8_t fifo_status = readRegister(FIFO_STATUS);
    if(fifo_status & _BV(RX_FULL))
        return FIFO_STATUS_FULL;
    else if(fifo_status & _BV(RX_EMPTY))
        return  FIFO_STATUS_EMPTY;
    else
        return FIFO_STATUS_OK;
}

/**
 * Reset RX_DR, TX_DS and MAX_RT bits.
 * @note Write 1 to clear bits.
 */
void NRF24::resetCurrentStatus()
{
    writeRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}

//endregion

/**
 * Driver functions
 */

//region Driver functions

/**
 * Set the PWR_UP bit in the CONFIG register high to enter standby-I mode.
 */
void NRF24::powerUp()
{
    writeRegister(CONFIG, readRegister(CONFIG) | _BV(PWR_UP));
}

/**
 * Set the PWR_UP bit in the CONFIG register low to entert Power down mode.
 */
void NRF24::powerDown() {
    writeRegister(CONFIG, readRegister(CONFIG) & ~_BV(PWR_UP));
}

/**
 * Begin transmission. Move to "Standby-I" state
 */
void NRF24::begin()
{
    // State: "Power Down"

    powerUp();

    // State: "Crystal oscillator start up"

    delayMicroseconds(4500); // Tpd2stby = max(4.5ms)

    // State: "Standby-I"
}

void NRF24::start()
{
    // State: "Standby-I"

    ce(HIGH);

    /* State:
     *  if (Mode_PRX) -> "RX Mode"
     *  if (Mode_PTX):
     *      if (TX_FIFO_EMPTY) -> "Standby-II"
     *      else -(130us)-> "TX Mode"
     */
}

/**
 * Go back to "Standby-I" state
 */
void NRF24::stop()
{
    // State: "RX Mode/TX Mode"

    ce(LOW);

    // State: "Standby-I"
}

/**
 * Go to "Power down" state
 */
void NRF24::end()
{
    // State: Any

    stop(); // Reset CE
    powerDown();

    // State: "Power down"
}
//endregion

/**
 * Interrupt related functions
 */

//region Interrupt related functions

/**
 * Clear communication status
 */
void NRF24::clearCommStatus()
{
    writeRegister(STATUS, readRegister(STATUS) | (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)));
}

/**
 * Get current communication status
 * @param status
 */
void NRF24::getCommStatus(bool *status)
{
    // No Operation. Might be used to read the STATUS register.
    csn(LOW);
    uint8_t reg = SPI.transfer(NOP);
    csn(HIGH);

    status[0] = (bool) (reg & _BV(RX_DR));
    status[1] = (bool) (reg & _BV(TX_DS));
    status[2] = (bool) (reg & _BV(MAX_RT));
}

//endregion

/**
 * Util functions
 */

//region Util functions

/**
 * Check if NRF24 is P variant or not
 * @return Whether is P variant ot not
 */
bool NRF24::isPVariant()
{
    uint8_t setup = readRegister(RF_SETUP);
    uint8_t aux = setup;

    aux &= ~_BV(RF_DR_HIGH);
    aux |= _BV(RF_DR_LOW);

    writeRegister(RF_SETUP, aux);
    aux = readRegister(RF_SETUP);

    // Restore RF_SETUP original content
    writeRegister(RF_SETUP, setup);

    return (bool)(aux & _BV(RF_DR_LOW));
}

//endregion