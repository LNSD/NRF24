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

NRF24::Driver::Driver(uint8_t csn, uint8_t ce):
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

NRF24::Driver::Driver(uint8_t csn, uint8_t ce, uint8_t irq):
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

void NRF24::Driver::configure()
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

void NRF24::Driver::configure(Configuration configuration)
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
    setOutputRFPower(configuration._outputPower);
    setDataRate(configuration._dataRate);
    setRFChannel(configuration._rfCh);

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

    setAddressWidth(configuration._addressWidth);

    for (int p = 0; p < 6; p++) {
        if (configuration._rxPipeAddressStatus[p]) {
            enableRxPipeAddress((NRF24::RxPipe) p);
        } else {
            disableRxPipeAddress((NRF24::RxPipe) p);
        }
    }

    for (int p = 0; p < 6; p++) {
        setRxPipePayloadSize((NRF24::RxPipe) p, configuration._rxPipePayloadSize[p]);
    }

    for (int p = 0; p < 6; p++) {
        if (p < 2) {
            setRxPipeAddress((NRF24::RxPipe) p, configuration._rxPipeAddrLong[p], configuration._addressWidth);
        } else {
            setRxPipeAddress((NRF24::RxPipe) p, &configuration._rxPipeAddrShort[p - 2], 1);
        }
    }

    setTxAddress(configuration._txAddr, configuration._addressWidth);

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

inline void NRF24::Driver::csn(uint8_t val)
{
    digitalWrite(_csn, val);
}

inline void NRF24::Driver::ce(uint8_t val)
{
    digitalWrite(_ce, val);
}

inline void NRF24::Driver::spiCmdTransfer(uint8_t cmd)
{
    csn(LOW);
    SPI.transfer(cmd);
    csn(HIGH);
}

inline void NRF24::Driver::spiCmdTransfer(uint8_t cmd, void *buf, size_t len)
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

uint8_t NRF24::Driver::readRegister(uint8_t reg)
{
    uint8_t data;

    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), &data, 1);

    return data;
}

void NRF24::Driver::readRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), buf, len);
}

void NRF24::Driver::writeRegister(uint8_t reg, uint8_t value)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), &value, 1);
}

void NRF24::Driver::writeRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), buf, len);
}

//endregion

/**
 * Configuration functions. Getters and setters
 */

//region Configuration functions. Getters and setters

void NRF24::Driver::setTransceiverMode(TransceiverMode mode)
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

NRF24::TransceiverMode NRF24::Driver::getTransceiverMode()
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

void NRF24::Driver::enableConstantCarrier()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup |= _BV(CONT_WAVE);

    writeRegister(RF_SETUP, rfsetup);
}

void NRF24::Driver::disableConstantCarrier()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup &= ~_BV(CONT_WAVE);

    writeRegister(RF_SETUP, rfsetup);
}

bool NRF24::Driver::isConstantCarrierEnabled()
{
    return (bool)(readRegister(RF_SETUP) & _BV(CONT_WAVE));
}

void NRF24::Driver::forcePllLock()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup |= _BV(PLL_LOCK);

    writeRegister(RF_SETUP, rfsetup);
}

void NRF24::Driver::disablePllLock()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup &= ~_BV(PLL_LOCK);

    writeRegister(RF_SETUP, rfsetup);
}

bool NRF24::Driver::isPllLockForced()
{
    return (readRegister(RF_SETUP) & _BV(PLL_LOCK)) > 0;
}

void NRF24::Driver::setOutputRFPower(OutputPower level)
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

NRF24::OutputPower NRF24::Driver::getOutputRFPower()
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

void NRF24::Driver::setDataRate(DataRate rate)
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

NRF24::DataRate NRF24::Driver::getDataRate()
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

void NRF24::Driver::setRFChannel(uint8_t channel)
{
    writeRegister(RF_CH, min(channel, MAX_RF_CHANNEL));
}

uint8_t NRF24::Driver::getRFChannel()
{
    return readRegister(RF_CH);
}

void NRF24::Driver::setAddressWidth(uint8_t width)
{
    if(width >= 3  && width <= 5)
    {
        writeRegister(SETUP_AW, width-2);
    }
}

uint8_t NRF24::Driver::getAddressWidth()
{
    return readRegister(SETUP_AW) + 2;
}

void NRF24::Driver::enableRxPipeAddress(RxPipe pipe)
{
    writeRegister(EN_RXADDR, readRegister(EN_RXADDR) | _BV(pipe));
}

void NRF24::Driver::disableRxPipeAddress(RxPipe pipe)
{
    writeRegister(EN_RXADDR, readRegister(EN_RXADDR) & ~_BV(pipe));
}

void NRF24::Driver::whichRxPipeAddrAreEnabled(bool *enabledAddr)
{
    uint8_t enRxAddr = readRegister(EN_RXADDR);
    for (int p = 0; p < 6; ++p)
    {
        enabledAddr[p] = (bool) (enRxAddr & _BV(p));
    }
}

void NRF24::Driver::setTxAddress(uint8_t *addr, uint8_t len)
{
    writeRegister(TX_ADDR, addr, len);
}

void NRF24::Driver::getTxAddress(uint8_t *addr, uint8_t len)
{
    readRegister(TX_ADDR, addr, len);
}

const uint8_t pipeRx[6] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

void NRF24::Driver::setRxPipeAddress(RxPipe pipe, uint8_t *addr, uint8_t len)
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

void NRF24::Driver::getRxPipeAddress(RxPipe pipe, uint8_t *addr, uint8_t len)
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

void NRF24::Driver::setRxPipePayloadSize(RxPipe pipe, uint8_t size)
{
    const uint8_t max_size = 32;
    writeRegister(pipe_payload[pipe], min(size, max_size));
}

uint8_t NRF24::Driver::getRxPipePayloadSize(RxPipe pipe)
{
    return readRegister(pipe_payload[pipe]);
}

void NRF24::Driver::enableRxPipeDynamicPayloads(RxPipe pipe)
{
    uint8_t dynpd = readRegister(DYNPD);
    dynpd |= _BV(pipe);

    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_DPL));
    writeRegister(DYNPD, dynpd);
}

void NRF24::Driver::disableRxPipeDynamicPayloads(RxPipe pipe)
{
    uint8_t dynpd = readRegister(DYNPD);

    if(!(dynpd & ~_BV(pipe))) {
        writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_DPL));
    }

    writeRegister(DYNPD, dynpd & ~_BV(pipe));
}

void NRF24::Driver::disableDynamicPayloads()
{
    writeRegister(DYNPD, 0x00);
}

void NRF24::Driver::whichRxPipeDynamicPayloadsAreEnabled(bool *dynamicPayloads)
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

void NRF24::Driver::enableCRC(CRCLength length)
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

void NRF24::Driver::disableCRC()
{
    uint8_t config = readRegister(CONFIG);
    config &= ~_BV(EN_CRC);
    writeRegister(CONFIG, config);
}

NRF24::CRCLength NRF24::Driver::getCRCConfig()
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

void NRF24::Driver::enableRxPipeAutoAck(RxPipe pipe)
{
    writeRegister(EN_AA, readRegister(EN_AA) | _BV(pipe));
}

void NRF24::Driver::disableRxPipeAutoAck(RxPipe pipe)
{
    writeRegister(EN_AA, readRegister(EN_AA) & ~_BV(pipe));
}

void NRF24::Driver::whichRxPipeAutoAckAreEnabled(bool *autoAck)
{
    uint8_t autoack = readRegister(EN_AA);
    for (int p = 0; p < 6; ++p)
    {
        autoAck[p] = (bool) (autoack & _BV(p));
    }
}

void NRF24::Driver::setAutoRtDelay(uint16_t delay)
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

uint8_t NRF24::Driver::getAutoRtDelay()
{
    return (readRegister(SETUP_RETR) >> 4);
}

void NRF24::Driver::setAutoRtCount(uint8_t count)
{
    const uint8_t max_count = 0xF;
    uint8_t setupRetr = readRegister(SETUP_RETR);
    setupRetr &= 0xF0;
    setupRetr |= min(count, max_count);

    writeRegister(SETUP_RETR, setupRetr);
}

uint8_t NRF24::Driver::getAutoRtCount()
{
    return readRegister(SETUP_RETR) & 0x0F;
}

void NRF24::Driver::enableAckPayload()
{
    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_ACK_PAY));
}

void NRF24::Driver::disableAckPayload()
{
    writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_ACK_PAY));
}

bool NRF24::Driver::isAckPayloadEnabled()
{
    return (bool)(readRegister(FEATURE) & _BV(EN_ACK_PAY));
}

void NRF24::Driver::enableDynamicAck()
{
    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_DYN_ACK));
}

void NRF24::Driver::disableDynamicAck()
{
    writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_DYN_ACK));
}

bool NRF24::Driver::isDynamicAckEnabled()
{
    return (bool)(readRegister(FEATURE) & _BV(EN_DYN_ACK));
}

//endregion

/**
 * Command functions
 */

//region Command functions

uint8_t NRF24::Driver::getRxPayloadLength()
{
    uint8_t width;

    spiCmdTransfer(R_RX_PL_WID, &width, 1);

    return width;
}

void NRF24::Driver::readRxPayload(uint8_t* data, uint8_t len)
{
    spiCmdTransfer(R_RX_PAYLOAD, data, len);
}

void NRF24::Driver::writeTxPayload(uint8_t* data, uint8_t len)
{
    spiCmdTransfer(W_TX_PAYLOAD, data, len);
}

void NRF24::Driver::writePipeACKPayload(RxPipe pipe, uint8_t* data, uint8_t len)
{
    spiCmdTransfer((uint8_t) (W_ACK_PAYLOAD | (pipe & 0x07)), data, len);
}

void NRF24::Driver::disableAAforPayload()
{
    spiCmdTransfer(W_TX_PAYLOAD_NOACK);
}

void NRF24::Driver::reuseTxPayload()
{
    spiCmdTransfer(REUSE_TX_PL);
}

void NRF24::Driver::flushTxFifo()
{
    spiCmdTransfer(FLUSH_TX);
}

void NRF24::Driver::flushRxFifo()
{
    spiCmdTransfer(FLUSH_RX);
}

//endregion

/**
 * Get status functions
 */

//region Status functions

uint8_t NRF24::Driver::getLostPacketsCount()
{
    return (readRegister(OBSERVE_TX) & 0xF0) >> 4;
}

uint8_t NRF24::Driver::getRtCount()
{
    return readRegister(OBSERVE_TX) & 0x0F;
}

bool NRF24::Driver::isCarrierDetected()
{
    return (bool) (readRegister(RPD) & 0x01);
}

bool NRF24::Driver::isReuseTxPayloadActive()
{
    return (bool) (readRegister(FIFO_STATUS) & _BV(TX_REUSE));
}

NRF24::FifoStatus NRF24::Driver::getTxFifoStatus()
{
    uint8_t fifo_status = readRegister(FIFO_STATUS);
    if(fifo_status & _BV(TX_FULL))
        return FIFO_STATUS_FULL;
    else if(fifo_status & _BV(TX_EMPTY))
        return  FIFO_STATUS_EMPTY;
    else
        return FIFO_STATUS_OK;
}

NRF24::FifoStatus NRF24::Driver::getRxFifoStatus()
{
    uint8_t fifo_status = readRegister(FIFO_STATUS);
    if(fifo_status & _BV(RX_FULL))
        return FIFO_STATUS_FULL;
    else if(fifo_status & _BV(RX_EMPTY))
        return  FIFO_STATUS_EMPTY;
    else
        return FIFO_STATUS_OK;
}

void NRF24::Driver::resetCurrentStatus()
{
    writeRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}

//endregion

/**
 * Driver functions
 */

//region Driver functions

void NRF24::Driver::powerUp()
{
    writeRegister(CONFIG, readRegister(CONFIG) | _BV(PWR_UP));
}

void NRF24::Driver::powerDown() {
    writeRegister(CONFIG, readRegister(CONFIG) & ~_BV(PWR_UP));
}

void NRF24::Driver::begin()
{
    // State: "Power Down"

    powerUp();

    // State: "Crystal oscillator start up"

    delayMicroseconds(4500); // Tpd2stby = max(4.5ms)

    // State: "Standby-I"
}

void NRF24::Driver::start()
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

void NRF24::Driver::stop()
{
    // State: "RX Mode/TX Mode"

    ce(LOW);

    // State: "Standby-I"
}

void NRF24::Driver::end()
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

void NRF24::Driver::getCommStatus(bool *status)
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

bool NRF24::Driver::isPVariant()
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