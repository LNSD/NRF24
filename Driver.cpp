/**
 * @file Driver.cpp
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "Driver.h"
#include "Arduino.h"
#include <SPI.h>
#include "NRF24L01P.h"

//region Private

/**
 * Low level signal control
 */

//region Low level signals control

inline void NRF24::Driver::csn(uint8_t val)
{
    digitalWrite(_csn, val);
}

inline void NRF24::Driver::ce(uint8_t val)
{
    digitalWrite(_ce, val);
}

//endregion

/**
 * SPI command
 */

//region SPI command functions

inline void NRF24::Driver::spiCmdTransfer(uint8_t cmd)
{
    csn(LOW);
    SPI.transfer(cmd);
    csn(HIGH);
}

inline void NRF24::Driver::spiCmdTransfer(uint8_t cmd, void* buf, size_t len)
{
    csn(LOW);
    SPI.transfer(cmd);
    SPI.transfer(buf, len);
    csn(HIGH);
}

//endregion

/**
 * SPI command-status
 */

//region SPI command-status

inline NRF24::Register::STATUS NRF24::Driver::spiCmdStatusTransfer(uint8_t cmd)
{
    csn(LOW);
    Register::STATUS status = { .raw = SPI.transfer(cmd) };
    csn(HIGH);

    return status;
}

inline NRF24::Register::STATUS NRF24::Driver::spiCmdStatusTransfer(uint8_t cmd, void* buf, size_t len)
{
    csn(LOW);
    Register::STATUS status = { .raw = SPI.transfer(cmd) };
    SPI.transfer(buf, len);
    csn(HIGH);

    return status;
}

//endregion

/**
 * Register manipulation
 */

//region Register manipulation

uint8_t NRF24::Driver::readRegister(uint8_t reg)
{
    uint8_t data;

    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), &data, 1);

    return data;
}

void NRF24::Driver::readRegister(uint8_t reg, uint8_t *buf, size_t len)
{
    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), buf, len);
}

void NRF24::Driver::writeRegister(uint8_t reg, uint8_t value)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), &value, 1);
}

void NRF24::Driver::writeRegister(uint8_t reg, uint8_t *buf, size_t len)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), buf, len);
}

//endregion

//endregion

//region Public

/**
 * Constructors
 */

//region Constructors
NRF24::Driver::Driver(uint8_t csn, uint8_t ce):
        _sck(13),
        _mosi(11),
        _miso(12),
        _csn(csn),
        _ce(ce)
{
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);
}

NRF24::Driver::Driver(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t csn, uint8_t ce):
        _sck(sck),
        _mosi(mosi),
        _miso(miso),
        _csn(csn),
        _ce(ce)
{
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);
}

//endregion

/**
 * Configuration
 */

//region Configuration

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
}

void NRF24::Driver::configure(uint32_t spi)
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
    SPI.beginTransaction(SPISettings(spi, MSBFIRST, SPI_MODE0));

    // Interface configure
    ce(LOW);
    csn(HIGH);
}

//endregion

/**
 * Chip control signals
 */

//region Chip control signals

void NRF24::Driver::chipEnable()
{
    ce(HIGH);
}

void NRF24::Driver::chipDisable()
{
    ce(LOW);
}

//endregion

/**
 * Register manipulation
 */

//region Register manipulation

NRF24::Register::CONFIG NRF24::Driver::readConfigRegister()
{
    return (Register::CONFIG) { .raw = readRegister(CONFIG) };
}

void NRF24::Driver::writeConfigRegister(Register::CONFIG reg)
{
    writeRegister(CONFIG, reg.raw);
}

NRF24::Register::EN_AA NRF24::Driver::readEnAARegister()
{
    return (Register::EN_AA) { .raw = readRegister(EN_AA) };
}

void NRF24::Driver::writeEnAARegister(Register::EN_AA reg)
{
    writeRegister(EN_AA, reg.raw);
}

NRF24::Register::EN_RXADDR NRF24::Driver::readEnRxAddrRegister()
{
    return (Register::EN_RXADDR) { .raw = readRegister(EN_RXADDR) };
}

void NRF24::Driver::writeEnRxAddrRegister(Register::EN_RXADDR reg)
{
    writeRegister(EN_RXADDR, reg.raw);
}

NRF24::Register::SETUP_AW NRF24::Driver::readSetupAWRegister()
{
    return (Register::SETUP_AW) { .raw = readRegister(SETUP_AW) };
}

void NRF24::Driver::writeSetupAWRegister(Register::SETUP_AW reg)
{
    writeRegister(SETUP_AW, reg.raw);
}

NRF24::Register::SETUP_RETR NRF24::Driver::readSetupRetrRegister()
{
    return (Register::SETUP_RETR) { .raw = readRegister(SETUP_RETR) };
}

void NRF24::Driver::writeSetupRetrRegister(Register::SETUP_RETR reg)
{
    writeRegister(SETUP_RETR, reg.raw);
}

NRF24::Register::RF_CH NRF24::Driver::readRFChannelRegister()
{
    return (Register::RF_CH) { .raw = readRegister(RF_CH) };
}

void NRF24::Driver::writeRFChannelRegister(Register::RF_CH reg)
{
    writeRegister(RF_CH, reg.raw);
}

NRF24::Register::RF_SETUP NRF24::Driver::readRFSetupRegister()
{
    return (Register::RF_SETUP) { .raw = readRegister(RF_SETUP) };
}

void NRF24::Driver::writeRFSetupRegister(Register::RF_SETUP reg)
{
    writeRegister(RF_SETUP, reg.raw);
}

NRF24::Register::STATUS NRF24::Driver::readStatusRegister()
{
    return (Register::STATUS) { .raw = readRegister(STATUS) };
}

void NRF24::Driver::writeStatusRegister(Register::STATUS reg)
{
    writeRegister(STATUS, reg.raw);
}

NRF24::Register::OBSERVE_TX NRF24::Driver::readObserveTxRegister()
{
    return (Register::OBSERVE_TX) { .raw = readRegister(OBSERVE_TX) };
}

void NRF24::Driver::writeObserveTxRegister(Register::OBSERVE_TX reg)
{
    writeRegister(OBSERVE_TX, reg.raw);
}

NRF24::Register::RPD NRF24::Driver::readRPDRegister()
{
    return (Register::RPD) { .raw = readRegister(RPD) };
}

void NRF24::Driver::writeRPDRegister(Register::RPD reg)
{
    writeRegister(RPD, reg.raw);
}

void NRF24::Driver::readRxPipeAddrRegister(uint8_t pipe, uint8_t* addr, size_t len)
{
    readRegister(RX_ADDR_P0 + min(pipe,5), addr, len);
}

void NRF24::Driver::writeRxPipeAddrRegister(uint8_t pipe, uint8_t* addr, size_t len)
{
    writeRegister(RX_ADDR_P0 + min(pipe,5), addr, len);
}

void NRF24::Driver::readTxAddrRegister(uint8_t* addr, size_t len)
{
    readRegister(TX_ADDR, addr, len);
}

void NRF24::Driver::writeTxAddrRegister(uint8_t* addr, size_t len)
{
    writeRegister(TX_ADDR, addr, len);
}

NRF24::Register::RX_PW_PN NRF24::Driver::readRxPWRegister(uint8_t pipe)
{
    return (Register::RX_PW_PN) { .raw = readRegister(RX_PW_P0 + min(pipe, 5)) };
}

void NRF24::Driver::writeRxPWRegister(uint8_t pipe, Register::RX_PW_PN reg)
{
    writeRegister(RX_PW_P0 + min(pipe, 5), reg.raw);
}

NRF24::Register::FIFO_STATUS NRF24::Driver::readFifoStatusRegister()
{
    return (Register::FIFO_STATUS) { .raw = readRegister(FIFO_STATUS) };
}

void NRF24::Driver::writeFifoStatusRegister(Register::FIFO_STATUS reg)
{
    writeRegister(FIFO_STATUS, reg.raw);
}

NRF24::Register::DYNPD NRF24::Driver::readDYNPDRegister()
{
    return (Register::DYNPD) { .raw = readRegister(DYNPD) };
}

void NRF24::Driver::writeDYNPDRegister(Register::DYNPD reg)
{
    writeRegister(DYNPD, reg.raw);
}

NRF24::Register::FEATURE NRF24::Driver::readFeatureRegister()
{
    return (Register::FEATURE) { .raw = readRegister(FEATURE) };
}

void NRF24::Driver::writeFeatureRegister(Register::FEATURE reg)
{
    writeRegister(FEATURE, reg.raw);
}

//endregion

/**
 * Commands
 */

//region Commands

void NRF24::Driver::readRxPayload(uint8_t* data, size_t len)
{
    spiCmdTransfer(R_RX_PAYLOAD, data, len);
}

void NRF24::Driver::writeTxPayload(uint8_t* data, size_t len)
{
    spiCmdTransfer(W_TX_PAYLOAD, data, len);
}

void NRF24::Driver::flushTxFifo()
{
    spiCmdTransfer(FLUSH_TX);
}

void NRF24::Driver::flushRxFifo()
{
    spiCmdTransfer(FLUSH_RX);
}

void NRF24::Driver::reuseTxPayload()
{
    spiCmdTransfer(REUSE_TX_PL);
}

uint8_t NRF24::Driver::readRxPayloadLength()
{
    uint8_t width;

    spiCmdTransfer(R_RX_PL_WID, &width, 1);

    return width;
}

void NRF24::Driver::writeACKPayload(uint8_t pipe, uint8_t* data, size_t len)
{
    spiCmdTransfer((uint8_t) (W_ACK_PAYLOAD | (pipe & W_ACK_PAYLOAD_MASK)), data, len);
}

void NRF24::Driver::writeTxPayloadNOACK(uint8_t *data, size_t len)
{
    spiCmdTransfer(W_TX_PAYLOAD_NOACK, data, len);
}

//endregion

//endregion