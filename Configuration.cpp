/**
 * @file Configuration.cpp
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>
#include "Configuration.h"

NRF24::Configuration::Configuration()
{
    this->setTransceiverMode(Mode_PTX);
    this->setCRC(CRC_16);
    for (int p = 0; p < 6; ++p)
    {
        this->enableRxPipeAutoAck((RxPipe) p);
    }
    this->disableAllRxPipeAddresses();
    this->setAddressWidth(Width_5Bytes);
    this->setAutoRtCount(MAX_RT_COUNT);
    this->setAutoRtDelay(1500);
    this->setRFChannel(2);
    this->setOutputPower(OutputPower_0dBm);
    this->setDataRate(DataRate_1Mbps);
    this->disableConstantCarrier();
    this->disablePllLock();
    this->disableDynamicPayloads();
    this->disableAckPayload();
    this->disableDynamicAck();
}

NRF24::Configuration::Configuration(TransceiverMode mode)
{
    this->setTransceiverMode(mode);

    this->setCRC(CRC_16);
    for (int p = 0; p < 6; ++p)
    {
        this->enableRxPipeAutoAck((RxPipe) p);
    }
    this->disableAllRxPipeAddresses();
    this->setAddressWidth(Width_5Bytes);
    this->setAutoRtCount(MAX_RT_COUNT);
    this->setAutoRtDelay(1500);
    this->setRFChannel(2);
    this->setOutputPower(OutputPower_0dBm);
    this->setDataRate(DataRate_1Mbps);
    this->disableConstantCarrier();
    this->disablePllLock();
    this->disableDynamicPayloads();
    this->disableAckPayload();
    this->disableDynamicAck();
}

NRF24::Configuration::Configuration(TransceiverMode mode, OutputPower level, DataRate dataRate)
{
    this->setTransceiverMode(mode);
    this->setCRC(CRC_16);
    for (int p = 0; p < 6; ++p)
    {
        this->enableRxPipeAutoAck((RxPipe) p);
    }
    this->disableAllRxPipeAddresses();
    this->setAddressWidth(Width_5Bytes);
    this->setAutoRtCount(MAX_RT_COUNT);
    this->setAutoRtDelay(1500);
    this->setRFChannel(2);
    this->setOutputPower(level);
    this->setDataRate(dataRate);
    this->disableConstantCarrier();
    this->disablePllLock();
    this->disableDynamicPayloads();
    this->disableAckPayload();
    this->disableDynamicAck();
};

void NRF24::Configuration::setTransceiverMode(TransceiverMode mode)
{
    _config.PRIM_RX = (mode != Mode_PTX);
}

void NRF24::Configuration::setOutputPower(OutputPower level)
{
    _rfSetup.RF_PWR = (unsigned int) level;
}

void NRF24::Configuration::setDataRate(DataRate dataRate)
{
    switch(dataRate)
    {
        case DataRate_250kbps:
            _rfSetup.RF_DR_LOW = true;
            _rfSetup.RF_DR_HIGH = false;
            break;
        case DataRate_2Mbps:
            _rfSetup.RF_DR_LOW = false;
            _rfSetup.RF_DR_HIGH = true;
            break;
        case DataRate_1Mbps:
            _rfSetup.RF_DR_LOW = false;
            _rfSetup.RF_DR_HIGH = false;
            break;
    }
}

void NRF24::Configuration::setRFChannel(uint8_t channel)
{
    _rfCh.RF_CH = min(channel, MAX_RF_CHANNEL);
}

void NRF24::Configuration::enableConstantCarrier()
{
    _rfSetup.CONT_WAVE = true;
}

void NRF24::Configuration::disableConstantCarrier()
{
    _rfSetup.CONT_WAVE = false;
}

void NRF24::Configuration::forcePllLock()
{
    _rfSetup.PLL_LOCK = true;
}

void NRF24::Configuration::disablePllLock()
{
    _rfSetup.PLL_LOCK = false;
}

void NRF24::Configuration::setCRC(CRCLength length)
{
    _config.EN_CRC = (length != CRC_DISABLED);
    _config.CRCO = (length == CRC_16);
}

void NRF24::Configuration::setAddressWidth(AddressWidth width)
{
    _setupAw.AW = width;
}

void NRF24::Configuration::enableRxPipeAddress(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            _enRxAddr.ERX_P0 = true;
            break;
        case RX_P1:
            _enRxAddr.ERX_P1 = true;
            break;
        case RX_P2:
            _enRxAddr.ERX_P2 = true;
            break;
        case RX_P3:
            _enRxAddr.ERX_P3 = true;
            break;
        case RX_P4:
            _enRxAddr.ERX_P4 = true;
            break;
        case RX_P5:
            _enRxAddr.ERX_P5 = true;
            break;
    }
}

void NRF24::Configuration::disableRxPipeAddress(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            _enRxAddr.ERX_P0 = false;
            _rxPwPN[0].RX_PW_PN = PIPE_NOT_USED;
            break;
        case RX_P1:
            _enRxAddr.ERX_P1 = false;
            _rxPwPN[1].RX_PW_PN = PIPE_NOT_USED;
            break;
        case RX_P2:
            _enRxAddr.ERX_P2 = false;
            _rxPwPN[2].RX_PW_PN = PIPE_NOT_USED;
            break;
        case RX_P3:
            _enRxAddr.ERX_P3 = false;
            _rxPwPN[3].RX_PW_PN = PIPE_NOT_USED;
            break;
        case RX_P4:
            _enRxAddr.ERX_P4 = false;
            _rxPwPN[4].RX_PW_PN = PIPE_NOT_USED;
            break;
        case RX_P5:
            _enRxAddr.ERX_P5 = false;
            _rxPwPN[5].RX_PW_PN = PIPE_NOT_USED;
            break;
    }
}

void NRF24::Configuration::disableAllRxPipeAddresses()
{
    _enRxAddr.raw = 0x00;
    for (int p = 0; p < sizeof(_rxPwPN); ++p)
    {
        _rxPwPN[p].RX_PW_PN = PIPE_NOT_USED;
    }
}

void NRF24::Configuration::setRxPipePayloadSize(RxPipe pipe, uint8_t size)
{
    _rxPwPN[pipe].RX_PW_PN = min(size, MAX_PAYLOAD_SIZE);
}

void NRF24::Configuration::setRxPipeAddress(RxPipe pipe, uint8_t* address)
{
    if (pipe < 2) {
        memcpy(_rxPipeAddrLong[pipe], address, _setupAw.AW);
    } else {
        _rxPipeAddrShort[pipe] = address[0];
    }
}

void NRF24::Configuration::setTxAddress(uint8_t *address)
{
    memcpy(_txAddr, address, _setupAw.AW);
}

void NRF24::Configuration::enableRxPipeAutoAck(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            _enAA.ENAA_P0 = true;
            break;
        case RX_P1:
            _enAA.ENAA_P1 = true;
            break;
        case RX_P2:
            _enAA.ENAA_P2 = true;
            break;
        case RX_P3:
            _enAA.ENAA_P3 = true;
            break;
        case RX_P4:
            _enAA.ENAA_P4 = true;
            break;
        case RX_P5:
            _enAA.ENAA_P5 = true;
            break;
    }
}

void NRF24::Configuration::disableRxPipeAutoAck(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            _enAA.ENAA_P0 = false;
            break;
        case RX_P1:
            _enAA.ENAA_P1 = false;
            break;
        case RX_P2:
            _enAA.ENAA_P2 = false;
            break;
        case RX_P3:
            _enAA.ENAA_P3 = false;
            break;
        case RX_P4:
            _enAA.ENAA_P4 = false;
            break;
        case RX_P5:
            _enAA.ENAA_P5 = false;
            break;
    }
}

void NRF24::Configuration::disableAutoAck()
{
    _enAA.raw = 0x00;
}

void NRF24::Configuration::setAutoRtDelay(uint16_t delay)
{
    _setupRetr.ARD = constrain(delay, MIN_RT_DELAY, MAX_RT_DELAY)/250 - 1;
}

void NRF24::Configuration::setAutoRtCount(uint8_t count)
{
    _setupRetr.ARC = min(count, MAX_RT_COUNT);
}

void NRF24::Configuration::enableRxPipeDynamicPayload(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            _dynpd.DPL_P0 = true;
            break;
        case RX_P1:
            _dynpd.DPL_P1 = true;
            break;
        case RX_P2:
            _dynpd.DPL_P2 = true;
            break;
        case RX_P3:
            _dynpd.DPL_P3 = true;
            break;
        case RX_P4:
            _dynpd.DPL_P4 = true;
            break;
        case RX_P5:
            _dynpd.DPL_P5 = true;
            break;
    }
}

void NRF24::Configuration::disableRxPipeDynamicPayload(RxPipe pipe)
{
    switch (pipe)
    {
        case RX_P0:
            _dynpd.DPL_P0 = false;
            break;
        case RX_P1:
            _dynpd.DPL_P1 = false;
            break;
        case RX_P2:
            _dynpd.DPL_P2 = false;
            break;
        case RX_P3:
            _dynpd.DPL_P3 = false;
            break;
        case RX_P4:
            _dynpd.DPL_P4 = false;
            break;
        case RX_P5:
            _dynpd.DPL_P5 = false;
            break;
    }
}

void NRF24::Configuration::disableDynamicPayloads()
{
    _dynpd.raw = 0x00;
}

void NRF24::Configuration::enableAckPayload()
{
    _feature.EN_DYN_ACK = true;
}

void NRF24::Configuration::disableAckPayload()
{
    _feature.EN_DYN_ACK = false;
}

void NRF24::Configuration::enableDynamicAck()
{
    _feature.EN_DYN_ACK = true;
}

void NRF24::Configuration::disableDynamicAck()
{
    _feature.EN_DYN_ACK = false;
}
