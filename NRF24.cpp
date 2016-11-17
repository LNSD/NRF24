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
#include "Debug.h"

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

    // Transfer default configuration to nRF24
    transferConfiguration(Configuration());
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

    // Transfer configuration to NRF24
    transferConfiguration(configuration);
}

void NRF24::Driver::transferConfiguration(Configuration config)
{
    writeRegister(CONFIG, config._config.raw);
    writeRegister(EN_AA, config._enAA.raw);
    writeRegister(EN_RXADDR, config._enRxAddr.raw);
    writeRegister(SETUP_AW, config._setupAw.raw);
    writeRegister(SETUP_RETR, config._setupRetr.raw);
    writeRegister(RF_CH, config._rfCh.raw);
    writeRegister(RF_SETUP, config._rfSetup.raw);

    for (int p = 0; p < 6; ++p) {
        writeRegister(RX_PW_P0+p, config._rxPwPN[p].raw);
    }

    writeRegister(DYNPD, config._dynpd.raw);
    writeRegister(FEATURE, config._feature.raw);

    for (int p = 0; p < 6; p++) {
        if (p < 2) {
            setRxPipeAddress((RxPipe) p, config._rxPipeAddrLong[p], config._setupAw.AW);
        } else {
            setRxPipeAddress((RxPipe) p, &config._rxPipeAddrShort[p - 2], 1);
        }
    }

    setTxAddress(config._txAddr, config._setupAw.AW);
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
    config.PRIM_RX = (mode != Mode_PTX);
    writeRegister(CONFIG, config.raw);
}

NRF24::TransceiverMode NRF24::Driver::getTransceiverMode()
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

void NRF24::Driver::setDataRate(DataRate dataRate)
{
    Register::RF_SETUP rfSetup;
    rfSetup.raw = readRegister(RF_SETUP);

    switch(dataRate)
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

void NRF24::Driver::setAddressWidth(AddressWidth width)
{
    writeRegister(SETUP_AW, width);
}

NRF24::AddressWidth NRF24::Driver::getAddressWidth()
{
    return (AddressWidth) readRegister(SETUP_AW);
}

void NRF24::Driver::enableRxPipeAddress(RxPipe pipe)
{
    Register::EN_RXADDR enRxAddr;
    enRxAddr.raw = readRegister(EN_RXADDR);
    enRxAddr.raw |= _BV(pipe);
    writeRegister(EN_RXADDR, enRxAddr.raw);
}

void NRF24::Driver::disableRxPipeAddress(RxPipe pipe)
{
    Register::EN_RXADDR enRxAddr;
    enRxAddr.raw = readRegister(EN_RXADDR);
    enRxAddr.raw &= ~_BV(pipe);
    writeRegister(EN_RXADDR, enRxAddr.raw);}

NRF24::Register::EN_RXADDR NRF24::Driver::whichRxPipeAddrAreEnabled()
{
    Register::EN_RXADDR enRxAddr;
    enRxAddr.raw = readRegister(EN_RXADDR);
    return enRxAddr;
}

void NRF24::Driver::setTxAddress(uint8_t *addr, uint8_t len)
{
    writeRegister(TX_ADDR, addr, len);
}

void NRF24::Driver::getTxAddress(uint8_t *addr, uint8_t len)
{
    readRegister(TX_ADDR, addr, len);
}

void NRF24::Driver::setRxPipeAddress(RxPipe pipe, uint8_t *addr, uint8_t len)
{
    switch (pipe)
    {
        case RX_P0:
            writeRegister(RX_ADDR_P0, addr, len);
            break;
        case RX_P1:
            writeRegister(RX_ADDR_P1, addr, len);
            break;
        case RX_P2:
            writeRegister(RX_ADDR_P2, addr[0]);
            break;
        case RX_P3:
            writeRegister(RX_ADDR_P3, addr[0]);
            break;
        case RX_P4:
            writeRegister(RX_ADDR_P4, addr[0]);
            break;
        case RX_P5:
            writeRegister(RX_ADDR_P5, addr[0]);
            break;
    }
}

void NRF24::Driver::getRxPipeAddress(RxPipe pipe, uint8_t *addr, uint8_t len)
{
    switch (pipe)
    {
        case RX_P0:
            readRegister(RX_ADDR_P0, addr, len);
            break;
        case RX_P1:
            readRegister(RX_ADDR_P1, addr, len);
            break;
        case RX_P2:
            addr[0] = readRegister(RX_ADDR_P2);
            break;
        case RX_P3:
            addr[0] = readRegister(RX_ADDR_P3);
            break;
        case RX_P4:
            addr[0] = readRegister(RX_ADDR_P4);
            break;
        case RX_P5:
            addr[0] = readRegister(RX_ADDR_P5);
            break;
    }
}

void NRF24::Driver::setRxPipePayloadSize(RxPipe pipe, uint8_t size)
{
    switch (pipe)
    {
        case RX_P0:
            writeRegister(RX_PW_P0, min(size, MAX_PAYLOAD_SIZE));
            break;
        case RX_P1:
            writeRegister(RX_PW_P1, min(size, MAX_PAYLOAD_SIZE));
            break;
        case RX_P2:
            writeRegister(RX_PW_P2, min(size, MAX_PAYLOAD_SIZE));
            break;
        case RX_P3:
            writeRegister(RX_PW_P3, min(size, MAX_PAYLOAD_SIZE));
            break;
        case RX_P4:
            writeRegister(RX_PW_P4, min(size, MAX_PAYLOAD_SIZE));
            break;
        case RX_P5:
            writeRegister(RX_PW_P5, min(size, MAX_PAYLOAD_SIZE));
            break;
    }
}

uint8_t NRF24::Driver::getRxPipePayloadSize(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            return readRegister(RX_PW_P0);
        case RX_P1:
            return readRegister(RX_PW_P1);
        case RX_P2:
            return readRegister(RX_PW_P2);
        case RX_P3:
            return readRegister(RX_PW_P3);
        case RX_P4:
            return readRegister(RX_PW_P4);
        case RX_P5:
            return readRegister(RX_PW_P5);
    }
}

void NRF24::Driver::enableRxPipeDynamicPayloads(RxPipe pipe)
{
    Register::DYNPD dynpd;
    dynpd.raw = readRegister(DYNPD);
    dynpd.raw |= _BV(pipe);

    Register::FEATURE feature;
    feature.raw = readRegister(FEATURE);
    feature.EN_DPL = true;
    writeRegister(FEATURE, feature.raw);

    writeRegister(DYNPD, dynpd.raw);
}

void NRF24::Driver::disableRxPipeDynamicPayloads(RxPipe pipe)
{
    Register::DYNPD dynpd;
    dynpd.raw = readRegister(DYNPD);
    dynpd.raw &= ~_BV(pipe);

    if(!(dynpd.raw & ~_BV(pipe)))
    {
        Register::FEATURE feature;
        feature.raw = readRegister(FEATURE);
        feature.EN_DPL = false;
        writeRegister(FEATURE, feature.raw);
    }

    writeRegister(DYNPD, dynpd.raw);
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
    Register::EN_AA enAA;
    enAA.raw = readRegister(EN_AA);
    enAA.raw |= _BV(pipe);
    writeRegister(EN_AA, enAA.raw);
}

void NRF24::Driver::disableRxPipeAutoAck(RxPipe pipe)
{
    Register::EN_AA enAA;
    enAA.raw = readRegister(EN_AA);
    enAA.raw &= ~_BV(pipe);
    writeRegister(EN_AA, enAA.raw);
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
    setupRetr.ARD = constrain(delay, MIN_RT_DELAY, MAX_RT_DELAY)/250 - 1;
    writeRegister(SETUP_RETR, setupRetr.raw);
}

uint16_t NRF24::Driver::getAutoRtDelay()
{
    Register::SETUP_RETR setupRetr;
    setupRetr.raw = readRegister(SETUP_RETR);
    return 250*(setupRetr.ARD + 1);
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
    spiCmdTransfer((uint8_t) (W_ACK_PAYLOAD | (pipe & W_ACK_PAYLOAD_MASK)), data, len);
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
    Register::OBSERVE_TX observeTx;
    observeTx.raw = readRegister(OBSERVE_TX);
    return observeTx.PLOS_CNT;
}

uint8_t NRF24::Driver::getRtCount()
{
    Register::OBSERVE_TX observeTx;
    observeTx.raw = readRegister(OBSERVE_TX);
    return observeTx.ARC_CNT;
}

bool NRF24::Driver::isCarrierDetected()
{
    Register::RPD rpd;
    rpd.raw = readRegister(RPD);
    return rpd.RPD;
}

bool NRF24::Driver::isReuseTxPayloadActive()
{
    Register::FIFO_STATUS fifoStatus;
    fifoStatus.raw = readRegister(FIFO_STATUS);
    return fifoStatus.TX_REUSE;
}

NRF24::FifoStatus NRF24::Driver::getTxFifoStatus()
{
    Register::FIFO_STATUS fifoStatus;
    fifoStatus.raw = readRegister(FIFO_STATUS);

    if (fifoStatus.TX_FULL) {
        return FIFO_STATUS_FULL;
    } else if (fifoStatus.TX_EMPTY) {
        return FIFO_STATUS_EMPTY;
    } else {
        return FIFO_STATUS_OK;
    }
}

NRF24::FifoStatus NRF24::Driver::getRxFifoStatus()
{
    Register::FIFO_STATUS fifoStatus;
    fifoStatus.raw = readRegister(FIFO_STATUS);

    if (fifoStatus.RX_FULL) {
        return FIFO_STATUS_FULL;
    } else if (fifoStatus.RX_EMPTY) {
        return FIFO_STATUS_EMPTY;
    } else {
        return FIFO_STATUS_OK;
    }
}

void NRF24::Driver::resetCurrentStatus()
{
    Register::STATUS status;
    status.RX_DR = true;
    status.TX_DS = true;
    status.MAX_RT = true;
    writeRegister(STATUS, status.raw);
}

NRF24::Register::STATUS NRF24::Driver::getCommStatus()
{
    Register::STATUS status;

    // No Operation. Might be used to read the STATUS register.
    csn(LOW);
    status.raw = SPI.transfer(NOP);
    csn(HIGH);

    return status;
}

//endregion

/**
 * Driver functions
 */

//region Driver functions

void NRF24::Driver::powerUp()
{
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);
    config.PWR_UP = true;
    writeRegister(CONFIG, config.raw);
}

void NRF24::Driver::powerDown()
{
    Register::CONFIG config;
    config.raw = readRegister(CONFIG);
    config.PWR_UP = false;
    writeRegister(CONFIG, config.raw);
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

//endregion

/**
 * Util functions
 */

//region Util functions

bool NRF24::Driver::isPVariant()
{
    Register::RF_SETUP rfSetup, aux;
    rfSetup.raw = readRegister(RF_SETUP);

    aux = rfSetup;
    aux.RF_DR_HIGH = false;
    aux.RF_DR_LOW = true;
    writeRegister(RF_SETUP, aux.raw);
    aux.raw = readRegister(RF_SETUP);

    // Restore RF_SETUP original content
    writeRegister(RF_SETUP, rfSetup.raw);

    return aux.RF_DR_LOW;
}

//endregion