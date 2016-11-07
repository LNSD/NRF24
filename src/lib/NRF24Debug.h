/**
 * NRF24Debug.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24DEBUG_H
#define NRF24DEBUG_H

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "NRF24L01P.h"
#include "NRF24.h"

class NRF24Debug
{
private:

    /**
     * Parse byte bits into boolean array
     * @param bit Boolean bit array
     * @param byte Byte to parse
     */
    static void parseToBoolean(boolean *bit, uint8_t byte);

public:

    /**
     * Parse CONFIG register content and show debug info
     * @param content Register content
     */
    static void debugConfigRegister(uint8_t content);

    /**
     * Parse EN_AA register content and show debug info
     * @param content Register content
     */
    static void debugEnAARegister(uint8_t content);

    /**
     * Parse EN_RXADDR register content and show debug info
     * @param content Register content
     */
    static void debugEnRxAddrRegister(uint8_t content);

    /**
     * Parse SETUP_AW register content and show debug info
     * @param content Register content
     */
    static void debugSetupAWRegister(uint8_t content);

    /**
     * Parse SETUP_RETR register content and show debug info
     * @param content Register content
     */
    static void debugSetupRetrRegister(uint8_t content);

    /**
     * Parse RF_CH register content and show debug info
     * @param content Register content
     */
    static void debugRFChRegister(uint8_t content);

    /**
     * Parse RF_SETUP register content and show debug info
     * @param content Register content
     */
    static void debugRFSetupRegister(uint8_t content);

    /**
     * Parse STATUS register content and show debug info
     * @param content Register content
     */
    static void debugStatusRegister(uint8_t content);

    /**
     * Parse OBSERVE_TX register content and show debug info
     * @param content Register content
     */
    static void debugObserveTxRegister(uint8_t content);

    /**
     * Parse RPD register content and show debug info
     * @param content Register content
     */
    static void debugRpdRegister(uint8_t content);

    /**
     * Parse RX_ADDR_P# register content and show debug info. Long address (5 bytes max)
     * @param content Register content
     * @param pipe Pipe number
     */
    static void debugRxPipeAddressRegister(uint8_t *content, NRF24::RxPipe pipe, uint8_t len);

    /**
     * Parse TX_ADDR register content and show debug info
     * @param content Register content
     */
    static void debugTxAddressRegister(uint8_t *content, uint8_t len);

    /**
     * Parse RX_PW_P# registers content and show debug info
     * @param content Register content
     * @param pipe Pipe number
     */
    static void debugRxPipePayloadWidthRegister(uint8_t content, NRF24::RxPipe pipe);

    /**
     * Parse FIFO_STATUS register content and show debug info
     * @param content Register content
     */
    static void debugFifoStatusRegister(uint8_t content);

    /**
     * Parse DYNPD register content and show debug info
     * @param content Register content
     */
    static void debugDynpdRegister(uint8_t content);

    /**
     * Parse FEATURE register content and show debug info
     * @param content Register content
     */
    static void debugFeatureRegister(uint8_t content);
};

#endif //NRF24DEBUG_H
