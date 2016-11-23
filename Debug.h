/**
 * @file Debug.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24_DEBUG_H
#define NRF24_DEBUG_H

#include <stdint.h>
#include "Core.h"

namespace NRF24
{
    /**
     * Debug helper class
     */
    class Debugger
    {
    private:
        Driver* _driver;
        
    public:

        /**
         * @name Constructors
         */

        /**
         * Constructor
         *
         * @param driver Pointer to driver object
         */
        Debugger(Driver* driver);

        /**
         * Constructor
         *
         * @param radio Pointer to radio object
         */
        Debugger(Radio* radio);


        /**
         * @name Register parsing
         */

        /**
         * Parse CONFIG register content
         */
        void parseConfigRegisterContent();

        /**
         * Parse EN_AA register content
         */
        void parseEnAARegisterContent();

        /**
         * Parse EN_RXADDR register content
         */
        void parseEnRxAddrRegisterContent();

        /**
         * Parse SETUP_AW register content
         */
        void parseSetupAWRegisterContent();

        /**
         * Parse SETUP_RETR register content
         */
        void parseSetupRetrRegisterContent();

        /**
         * Parse RF_CH register content
         */
        void parseRFChRegisterContent();

        /**
         * Parse RF_SETUP register content
         */
        void parseRFSetupRegisterContent();

        /**
         * Parse STATUS register content
         */
        void parseStatusRegisterContent();

        /**
         * Parse OBSERVE_TX register content
         */
        void parseObserveTxRegisterContent();

        /**
         * Parse RPD register content
         */
        void parseRPDRegisterContent();

        /**
         * Parse RX_ADDR_P# register content. Long address (5 bytes max)
         * @param pipe Pipe number
         * @param len Address width
         */
        void parseRxPipeAddressRegisterContent(uint8_t pipe, size_t len);

        /**
         * Parse TX_ADDR register content
         * @param len Address width
         */
        void parseTxAddressRegisterContent(size_t len);

        /**
         * Parse RX_PW_P# registers content
         * @param pipe Pipe number
         */
        void parseRxPipePayloadWidthRegisterContent(uint8_t pipe);

        /**
         * Parse FIFO_STATUS register content
         */
        void parseFifoStatusRegisterContent();

        /**
         * Parse DYNPD register content
         */
        void parseDYNPDRegisterContent();

        /**
         * Parse FEATURE register content
         */
        void parseFeatureRegisterContent();
    };
}

#endif //NRF24_DEBUG_H
