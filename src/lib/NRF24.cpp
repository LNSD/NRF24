/**
 * @file NRF24.cpp
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
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);

    if(mode == Mode_PTX)
    {
        config.PRIM_RX = false;
    }
    else
    {
        config.PRIM_RX = true;
    }

    writeRegister(CONFIG, config.raw);
}

TransceiverMode NRF24::Driver::getTransceiverMode()
{
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);

    if(config.PRIM_RX)
    {
        return Mode_PRX;
    }
    else
    {
        return Mode_PTX;
    }
}

void NRF24::Driver::enableConstantCarrier()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    rfSetup.CONT_WAVE = true;
    writeRegister(RF_SETUP, rfSetup.raw);
}

void NRF24::Driver::disableConstantCarrier()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    rfSetup.CONT_WAVE = false;
    writeRegister(RF_SETUP, rfSetup.raw);
}

bool NRF24::Driver::isConstantCarrierEnabled()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    return rfSetup.CONT_WAVE;
}

void NRF24::Driver::forcePllLock()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    rfSetup.PLL_LOCK = true;
    writeRegister(RF_SETUP, rfSetup.raw);
}

void NRF24::Driver::disablePllLock()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    rfSetup.PLL_LOCK = false;
    writeRegister(RF_SETUP, rfSetup.raw);
}

bool NRF24::Driver::isPllLockForced()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    return rfSetup.PLL_LOCK;
}

void NRF24::Driver::setOutputRFPower(OutputPower level)
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    rfSetup.RF_PWR = (unsigned int) level;
    writeRegister(RF_SETUP, rfSetup.raw);
}

NRF24::OutputPower NRF24::Driver::getOutputRFPower()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);
    return (OutputPower) rfSetup.RF_PWR;
}

void NRF24::Driver::setDataRate(DataRate rate)
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);

    switch(rate)
    {
        case DataRate_250kbps:
            rfSetup.RF_DR_LOW = true;
            rfSetup.RF_DR_HIGH = false;
            break;
        case DataRate_2Mbps:
            rfSetup.RF_DR_LOW = false;
            rfSetup.RF_DR_HIGH = true;
            break;
        case DataRate_1Mbps:
            rfSetup.RF_DR_LOW = false;
            rfSetup.RF_DR_HIGH = false;
            break;
    }

    writeRegister(RF_SETUP, rfSetup.raw);
}

NRF24::DataRate NRF24::Driver::getDataRate()
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);

    if(rfSetup.RF_DR_LOW && !rfSetup.RF_DR_HIGH)
    {
        // '10' = 250Kbps
        return DataRate_250kbps;
    }
    else if(!rfSetup.RF_DR_LOW && rfSetup.RF_DR_HIGH)
    {
        // '01' = 2Mbps
        return DataRate_2Mbps;
    }
    else if(!rfSetup.RF_DR_LOW && !rfSetup.RF_DR_HIGH)
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

const uint8_t pipeRx[6] = {NRF24::RX_ADDR_P0, NRF24::RX_ADDR_P1, NRF24::RX_ADDR_P2, NRF24::RX_ADDR_P3, NRF24::RX_ADDR_P4, NRF24::RX_ADDR_P5};

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

const uint8_t pipe_payload[6] = {NRF24::RX_PW_P0, NRF24::RX_PW_P1, NRF24::RX_PW_P2, NRF24::RX_PW_P3, NRF24::RX_PW_P4, NRF24::RX_PW_P5};

void NRF24::Driver::setRxPipePayloadSize(RxPipe pipe, uint8_t size)
{
    writeRegister(pipe_payload[pipe], min(size, MAX_PAYLOAD_SIZE));
}

uint8_t NRF24::Driver::getRxPipePayloadSize(RxPipe pipe)
{
    return readRegister(pipe_payload[pipe]);
}

void NRF24::Driver::enableRxPipeDynamicPayloads(RxPipe pipe)
{
    uint8_t dynpd = readRegister(DYNPD);
    dynpd |= _BV(pipe);

    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    feature.EN_DPL = true;
    writeRegister(FEATURE, feature.raw);

    writeRegister(DYNPD, dynpd);
}

void NRF24::Driver::disableRxPipeDynamicPayloads(RxPipe pipe)
{
    uint8_t dynpd = readRegister(DYNPD);

    if(!(dynpd & ~_BV(pipe))) {
        Register::FEATURE feature;
        feature.raw = readRegister(FEATURE);
        feature.EN_DPL = false;
        writeRegister(FEATURE, feature.raw);
    }

    writeRegister(DYNPD, dynpd & ~_BV(pipe));
}

void NRF24::Driver::disableDynamicPayloads()
{
    writeRegister(DYNPD, 0x00);
}

NRF24::Register::DYNPD NRF24::Driver::whichRxPipeDynamicPayloadsAreEnabled()
{
    Register::FEATURE feature;
    Register::DYNPD dynpd;

    feature.raw = readRegister(FEATURE);

    if(feature.EN_DPL)
    {
        dynpd.raw = readRegister(DYNPD);
    }
    else
    {
        dynpd.raw = 0x00;
    }

    return dynpd;
}

void NRF24::Driver::enableCRC(CRCLength length)
{
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);
    config.EN_CRC = true;
    config.CRCO = (length == CRC_16);
    writeRegister(CONFIG, config.raw);
}

void NRF24::Driver::disableCRC()
{
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);
    config.EN_CRC = false;
    writeRegister(CONFIG, config.raw);
}

NRF24::CRCLength NRF24::Driver::getCRCConfig()
{
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);

    if(config.EN_CRC)
    {
        return (config.CRCO)? CRC_16 : CRC_8;
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

NRF24::Register::EN_AA NRF24::Driver::whichRxPipeAutoAckAreEnabled()
{
    Register::EN_AA enAA;
    enAA.raw = readRegister(EN_AA);
    return enAA;
}

void NRF24::Driver::setAutoRtDelay(uint16_t delay)
{
    Register::SETUP_RETR setupRetr;
    setupRetr.raw = readRegister(SETUP_RETR);

    if (delay < MIN_RT_DELAY) {
        setupRetr.ARD = 0x0;
    } else if (delay > MAX_RT_DELAY) {
        setupRetr.ARD = 0xF;
    } else {
        setupRetr.ARD = (delay/250 - 1);
    }

    writeRegister(SETUP_RETR, setupRetr.raw);
}

uint8_t NRF24::Driver::getAutoRtDelay()
{
    Register::SETUP_RETR setupRetr;
    setupRetr.raw = readRegister(SETUP_RETR);
    return setupRetr.ARD;
}

void NRF24::Driver::setAutoRtCount(uint8_t count)
{
    Register::SETUP_RETR setupRetr;
    setupRetr.raw = readRegister(SETUP_RETR);
    setupRetr.ARC = min(count, MAX_RT_COUNT);
    writeRegister(SETUP_RETR, setupRetr.raw);
}

uint8_t NRF24::Driver::getAutoRtCount()
{
    Register::SETUP_RETR setupRetr;
    setupRetr.raw = readRegister(SETUP_RETR);
    return setupRetr.ARC;
}

void NRF24::Driver::enableAckPayload()
{
    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    feature.EN_ACK_PAY = true;
    writeRegister(FEATURE, feature.raw);
}

void NRF24::Driver::disableAckPayload()
{
    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    feature.EN_ACK_PAY = false;
    writeRegister(FEATURE, feature.raw);
}

bool NRF24::Driver::isAckPayloadEnabled()
{
    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    return feature.EN_ACK_PAY;
}

void NRF24::Driver::enableDynamicAck()
{
    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    feature.EN_DYN_ACK = true;
    writeRegister(FEATURE, feature.raw);
}

void NRF24::Driver::disableDynamicAck()
{
    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    feature.EN_DYN_ACK = false;
    writeRegister(FEATURE, feature.raw);
}

bool NRF24::Driver::isDynamicAckEnabled()
{
    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    return feature.EN_DYN_ACK;
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

#ifdef NRF24_DEBUG_ENABLED

/**
 * Debug functions
 */

//region Debug functions

void inline NRF24::Debug::parseToBoolean(boolean *bit, uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        bit[i] = (byte & _BV(i)) > 0;
    }
}

void NRF24::Debug::debugConfigRegister(uint8_t content)
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

void NRF24::Debug::debugEnAARegister(uint8_t content)
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

void NRF24::Debug::debugEnRxAddrRegister(uint8_t content)
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

void NRF24::Debug::debugSetupAWRegister(uint8_t content)
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

void NRF24::Debug::debugSetupRetrRegister(uint8_t content)
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

void NRF24::Debug::debugRFChRegister(uint8_t content)
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

void NRF24::Debug::debugRFSetupRegister(uint8_t content)
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

void NRF24::Debug::debugStatusRegister(uint8_t content)
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

void NRF24::Debug::debugObserveTxRegister(uint8_t content)
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

void NRF24::Debug::debugRpdRegister(uint8_t content)
{
    // Debug info header
    Serial.print(" - DEBUG: RPD register content (0x");
    Serial.print(content, HEX);
    Serial.println(")");

    // RPD bit
    Serial.print("\t - RPD: ");
    Serial.println(_BV(RPD_BIT) & content, BIN);
}

void NRF24::Debug::debugRxPipeAddressRegister(uint8_t *content, NRF24::RxPipe pipe, uint8_t len)
{
    uint8_t length = (pipe<2)? len:1;

    // Debug info header
    Serial.print(" - DEBUG: RX_ADDR_P");
    Serial.print(pipe, DEC);
    Serial.print(" (");
    Serial.print(length, DEC);
    Serial.print((length>1) ? " bytes): ":" byte): ");

    for (int i = 0; i < length; ++i) {
        if (content[i]<=0xF) Serial.print(0, HEX);
        Serial.print(content[i], HEX);
        Serial.print((i<length-1) ? ":":"\n");
    }
}

void NRF24::Debug::debugTxAddressRegister(uint8_t *content, uint8_t len)
{
    // Debug info header
    Serial.print(" - DEBUG: TX_ADDR (");
    Serial.print(len, DEC);
    Serial.print(" bytes): ");

    for (int i = 0; i < len; ++i) {
        if (content[i] <= 0xF) Serial.print(0, HEX);
        Serial.print(content[i], HEX);
        Serial.print((i<len-1) ? ":":"\n");
    }
}

void NRF24::Debug::debugRxPipePayloadWidthRegister(uint8_t content, NRF24::RxPipe pipe)
{
    // Debug info header
    Serial.print(" - DEBUG: RX_PW_P");
    Serial.print(pipe, DEC);
    Serial.print(" register content: ");
    Serial.println(content, DEC);
}

void NRF24::Debug::debugFifoStatusRegister(uint8_t content)
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

void NRF24::Debug::debugDynpdRegister(uint8_t content)
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

void NRF24::Debug::debugFeatureRegister(uint8_t content)
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

#endif