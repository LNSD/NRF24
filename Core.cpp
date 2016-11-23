/**
 * @file Core.cpp
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "Core.h"

/**
 * Constructors
 */

//region Constructors

NRF24::Radio::Radio(uint8_t csn, uint8_t ce):
        _irq(-1),
        _driver(csn, ce)
{}

NRF24::Radio::Radio(uint8_t csn, uint8_t ce, uint8_t irq):
        _irq(irq),
        _driver(csn, ce)
{
    //TODO Configure interrupt pin to IRQ
}

NRF24::Radio::Radio(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t csn, uint8_t ce):
        _irq(-1),
        _driver(sck, mosi, miso, csn, ce)
{}

NRF24::Radio::Radio(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t csn, uint8_t ce, uint8_t irq):
        _irq(irq),
        _driver(sck, mosi, miso, csn, ce)
{
    //TODO Configure interrupt pin to IRQ
}

//endregion

/**
 * Hardware setup
 */

//region Hardware setup

void NRF24::Radio::configure()
{
    _driver.configure();

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay( 5 ) ;
}

//endregion

/**
 * Getters and setters
 */

//region Getters and setters

void NRF24::Radio::setTransceiverMode(TransceiverMode mode)
{
    Register::CONFIG config = _driver.readConfigRegister();
    config.PRIM_RX = (mode != Mode_PTX);
    _driver.writeConfigRegister(config);
}

NRF24::TransceiverMode NRF24::Radio::getTransceiverMode()
{
    Register::CONFIG config = _driver.readConfigRegister();

    if(config.PRIM_RX)
    {
        return Mode_PRX;
    }
    else
    {
        return Mode_PTX;
    }
}

void NRF24::Radio::enableCRC(CRCLength length)
{
    Register::CONFIG config = _driver.readConfigRegister();
    config.EN_CRC = true;
    config.CRCO = (length == CRC_16);
    _driver.writeConfigRegister(config);
}

void NRF24::Radio::disableCRC()
{
    Register::CONFIG config = _driver.readConfigRegister();
    config.EN_CRC = false;
    _driver.writeConfigRegister(config);
}

NRF24::CRCLength NRF24::Radio::getCRCConfig()
{
    Register::CONFIG config = _driver.readConfigRegister();

    if(config.EN_CRC)
    {
        return (config.CRCO)? CRC_16 : CRC_8;
    }
    else
    {
        return CRC_DISABLED;
    }
}


void NRF24::Radio::enableRxPipeAutoAck(uint8_t pipe)
{
    Register::EN_AA enAA = _driver.readEnAARegister();
    enAA.raw |= _BV(pipe);
    _driver.writeEnAARegister(enAA);
}

void NRF24::Radio::disableRxPipeAutoAck(uint8_t pipe)
{
    Register::EN_AA enAA = _driver.readEnAARegister();
    enAA.raw &= ~_BV(pipe);
    _driver.writeEnAARegister(enAA);
}


void NRF24::Radio::enableRxPipeAddress(uint8_t pipe)
{
    Register::EN_RXADDR enRxAddr = _driver.readEnRxAddrRegister();
    enRxAddr.raw |= _BV(pipe);
    _driver.writeEnRxAddrRegister(enRxAddr);
}

void NRF24::Radio::disableRxPipeAddress(uint8_t pipe)
{
    Register::EN_RXADDR enRxAddr = _driver.readEnRxAddrRegister();
    enRxAddr.raw &= ~_BV(pipe);
    _driver.writeEnRxAddrRegister(enRxAddr);
}

void NRF24::Radio::disableAutoAck()
{
    _driver.writeEnRxAddrRegister((Register::EN_RXADDR) { .raw = 0x00 });
}


void NRF24::Radio::setAddressWidth(uint8_t width)
{
    Register::SETUP_AW setupAw = { .AW = constrain(width, 3, 5) };
    _driver.writeSetupAWRegister(setupAw);
}

uint8_t NRF24::Radio::getAddressWidth()
{
    Register::SETUP_AW setupAw = _driver.readSetupAWRegister();
    return setupAw.AW;
}


void NRF24::Radio::setAutoRetransmitDelay(uint16_t delay)
{
    Register::SETUP_RETR setupRetr = _driver.readSetupRetrRegister();
    setupRetr.ARD = constrain(delay, MIN_RT_DELAY, MAX_RT_DELAY)/250 - 1;
    _driver.writeSetupRetrRegister(setupRetr);
}

uint16_t NRF24::Radio::getAutoRetransmitDelay()
{
    Register::SETUP_RETR setupRetr = _driver.readSetupRetrRegister();
    return 250*(setupRetr.ARD + 1);
}

void NRF24::Radio::setAutoRetransmitCount(uint8_t count)
{
    Register::SETUP_RETR setupRetr = _driver.readSetupRetrRegister();
    setupRetr.ARC = min(count, MAX_RT_COUNT);
    _driver.writeSetupRetrRegister(setupRetr);
}

uint8_t NRF24::Radio::getAutoRetransmitCount()
{
    Register::SETUP_RETR setupRetr = _driver.readSetupRetrRegister();
    return setupRetr.ARC;
}


void NRF24::Radio::setRFChannel(uint8_t channel)
{
    Register::RF_CH rfCh = { .RF_CH = min(channel, MAX_RF_CHANNEL) };
    _driver.writeRFChannelRegister(rfCh);
}

uint8_t NRF24::Radio::getRFChannel()
{
    Register::RF_CH rfCh = _driver.readRFChannelRegister();
    return rfCh.RF_CH;
}


void NRF24::Radio::enableConstantCarrier()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    rfSetup.CONT_WAVE = true;
    _driver.writeRFSetupRegister(rfSetup);
}

void NRF24::Radio::disableConstantCarrier()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    rfSetup.CONT_WAVE = false;
    _driver.writeRFSetupRegister(rfSetup);
}

void NRF24::Radio::forcePllLock()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    rfSetup.PLL_LOCK = true;
    _driver.writeRFSetupRegister(rfSetup);
}

void NRF24::Radio::disablePllLock()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    rfSetup.PLL_LOCK = false;
    _driver.writeRFSetupRegister(rfSetup);
}

void NRF24::Radio::setDataRate(DataRate dataRate)
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();

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

    _driver.writeRFSetupRegister(rfSetup);
}

NRF24::DataRate NRF24::Radio::getDataRate()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();

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

void NRF24::Radio::setOutputRFPower(OutputPower level)
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    rfSetup.RF_PWR = (unsigned int) level;
    _driver.writeRFSetupRegister(rfSetup);
}

NRF24::OutputPower NRF24::Radio::getOutputRFPower()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    return (OutputPower) rfSetup.RF_PWR;
}


void NRF24::Radio::setRxPipeAddress(uint8_t pipe, uint8_t* addr, uint8_t len)
{
    if (pipe < 2)
    {
        _driver.writeRxPipeAddrRegister(pipe, addr, len);
    }
    else
    {
        _driver.writeRxPipeAddrRegister(pipe, &addr[0], 1);
    }
}

void NRF24::Radio::getRxPipeAddress(uint8_t pipe, uint8_t* addr, uint8_t len)
{
    if (pipe < 2)
    {
        _driver.readRxPipeAddrRegister(pipe, addr, len);
    }
    else
    {
        _driver.readRxPipeAddrRegister(pipe, &addr[0], 1);
    }
}


void NRF24::Radio::setTxAddress(uint8_t* addr, uint8_t len)
{
    _driver.writeTxAddrRegister(addr, len);
}

void NRF24::Radio::getTxAddress(uint8_t* addr, uint8_t len)
{
    _driver.readTxAddrRegister(addr, len);
}


void NRF24::Radio::setRxPipePayloadLength(uint8_t pipe, uint8_t size)
{
    Register::RX_PW_PN rxPwPn = { .RX_PW_PN = min(size, MAX_PAYLOAD_SIZE) };
     _driver.writeRxPWRegister(pipe, rxPwPn);
}

uint8_t NRF24::Radio::getRxPipePayloadLength(uint8_t pipe)
{
    Register::RX_PW_PN rxPwPn = _driver.readRxPWRegister(pipe);
    return rxPwPn.RX_PW_PN;
}


void NRF24::Radio::enableRxPipeDynamicPayloads(uint8_t pipe)
{
    Register::DYNPD dynpd = _driver.readDYNPDRegister();
    dynpd.raw |= _BV(pipe);

    Register::FEATURE feature = _driver.readFeatureRegister();
    feature.EN_DPL = true;
    _driver.writeFeatureRegister(feature);

    _driver.writeDYNPDRegister(dynpd);
}

void NRF24::Radio::disableRxPipeDynamicPayloads(uint8_t pipe)
{
    Register::DYNPD dynpd = _driver.readDYNPDRegister();
    dynpd.raw &= ~_BV(pipe);

    if(!(dynpd.raw & ~_BV(pipe)))
    {
        Register::FEATURE feature = _driver.readFeatureRegister();
        feature.EN_DPL = false;
        _driver.writeFeatureRegister(feature);
    }

    _driver.writeDYNPDRegister(dynpd);
}

void NRF24::Radio::disableDynamicPayloads()
{
    Register::FEATURE feature = _driver.readFeatureRegister();
    feature.EN_DPL = false;
    _driver.writeFeatureRegister(feature);

    Register::DYNPD dynpd = _driver.readDYNPDRegister();
    dynpd.raw = 0x00;
    _driver.writeDYNPDRegister(dynpd);
}


void NRF24::Radio::enableAckPayload()
{
    Register::FEATURE feature = _driver.readFeatureRegister();
    feature.EN_ACK_PAY = true;
    _driver.writeFeatureRegister(feature);
}

void NRF24::Radio::disableAckPayload()
{
    Register::FEATURE feature = _driver.readFeatureRegister();
    feature.EN_ACK_PAY = false;
    _driver.writeFeatureRegister(feature);
}

void NRF24::Radio::enableDynamicAck()
{
    Register::FEATURE feature = _driver.readFeatureRegister();
    feature.EN_DYN_ACK = true;
    _driver.writeFeatureRegister(feature);
}

void NRF24::Radio::disableDynamicAck()
{
    Register::FEATURE feature = _driver.readFeatureRegister();
    feature.EN_DYN_ACK = false;
    _driver.writeFeatureRegister(feature);
}

//endregion

/**
 * Command functions
 */

//region Command functions

uint8_t NRF24::Radio::getPayloadLength()
{
    return _driver.readRxPayloadLength();
}

void NRF24::Radio::readPayload(uint8_t* data, uint8_t len)
{
    _driver.readRxPayload(data, len);
}

void NRF24::Radio::writePayload(uint8_t* data, uint8_t len)
{
    _driver.writeTxPayload(data,len);
}

void NRF24::Radio::writePipeAckPayload(uint8_t pipe, uint8_t* data, uint8_t len)
{
    _driver.writeACKPayload(pipe, data, len);
}

void NRF24::Radio::writePayloadNoAckPacket(uint8_t *data, uint8_t len)
{
    _driver.writeTxPayloadNOACK(data, len);
}

void NRF24::Radio::reuseTxPayload()
{
    _driver.reuseTxPayload();
}

void NRF24::Radio::flushTxFifo()
{
    _driver.flushTxFifo();
}

void NRF24::Radio::flushRxFifo()
{
    _driver.flushRxFifo();
}

//endregion

/**
 * Get status functions
 */

//region Status functions

uint8_t NRF24::Radio::getLostPacketsCount()
{
    Register::OBSERVE_TX observeTx = _driver.readObserveTxRegister();
    return observeTx.PLOS_CNT;
}

uint8_t NRF24::Radio::getRetransmittedPacketsCount()
{
    Register::OBSERVE_TX observeTx = _driver.readObserveTxRegister();
    return observeTx.ARC_CNT;
}

bool NRF24::Radio::isReuseTxPayloadActive()
{
    Register::FIFO_STATUS fifoStatus = _driver.readFifoStatusRegister();
    return fifoStatus.TX_REUSE;
}

NRF24::FifoStatus NRF24::Radio::getTxFifoStatus()
{
    Register::FIFO_STATUS fifoStatus = _driver.readFifoStatusRegister();

    if (fifoStatus.TX_FULL) {
        return FIFO_STATUS_FULL;
    } else if (fifoStatus.TX_EMPTY) {
        return FIFO_STATUS_EMPTY;
    } else {
        return FIFO_STATUS_OK;
    }
}

NRF24::FifoStatus NRF24::Radio::getRxFifoStatus()
{
    Register::FIFO_STATUS fifoStatus = _driver.readFifoStatusRegister();

    if (fifoStatus.RX_FULL) {
        return FIFO_STATUS_FULL;
    } else if (fifoStatus.RX_EMPTY) {
        return FIFO_STATUS_EMPTY;
    } else {
        return FIFO_STATUS_OK;
    }
}

void NRF24::Radio::getStatus(bool *dataReady, bool *dataSent, bool *maxRt)
{
    Register::STATUS status = _driver.readStatusRegister();
    *dataReady = status.RX_DR;
    *dataSent = status.TX_DS;
    *maxRt = status.MAX_RT;
}

void NRF24::Radio::clearStatus()
{
    Register::STATUS status;
    status.RX_DR = true;
    status.TX_DS = true;
    status.MAX_RT = true;
    _driver.writeStatusRegister(status);
}

//endregion

/**
 * Driver
 */

//region Driver functions

void NRF24::Radio::powerUp()
{
    Register::CONFIG config = _driver.readConfigRegister();
    config.PWR_UP = true;
    _driver.writeConfigRegister(config);
}

void NRF24::Radio::powerDown()
{
    Register::CONFIG config = _driver.readConfigRegister();
    config.PWR_UP = false;
    _driver.writeConfigRegister(config);
}

void NRF24::Radio::begin()
{
    // State: "Power Down"

    powerUp();

    // State: "Crystal oscillator start up"

    delayMicroseconds(4500); // Tpd2stby = max(4.5ms)

    // State: "Standby-I"
}

void NRF24::Radio::start()
{
    // State: "Standby-I"

    _driver.chipEnable();

    /* State:
     *  if (Mode_PRX) -> "RX Mode"
     *  if (Mode_PTX):
     *      if (TX_FIFO_EMPTY) -> "Standby-II"
     *      else -(130us)-> "TX Mode"
     */
}

void NRF24::Radio::stop()
{
    // State: "RX Mode/TX Mode"

    _driver.chipDisable();

    // State: "Standby-I"
}

void NRF24::Radio::end()
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

bool NRF24::Radio::isCarrierDetected()
{
    Register::RPD rpd = _driver.readRPDRegister();
    return rpd.RPD;
}

bool NRF24::Radio::isPVariant()
{
    Register::RF_SETUP rfSetup = _driver.readRFSetupRegister();
    Register::RF_SETUP aux = rfSetup;

    aux.RF_DR_HIGH = false;
    aux.RF_DR_LOW = true;
    _driver.writeRFSetupRegister(aux);
    aux = _driver.readRFSetupRegister();

    // Restore RF_SETUP original content
    _driver.writeRFSetupRegister(rfSetup);

    return aux.RF_DR_LOW;
}

//endregion