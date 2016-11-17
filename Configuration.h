/**
 * @file Configuration.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24_CONFIGURATION_H
#define NRF24_CONFIGURATION_H

#include <stdint.h>
#include "Arduino.h"

#include "Definitions.h"

namespace NRF24
{
    /**
     * @name Configuration holder class
     */
    class Configuration
    {
    public:

        /**
         * @name Constructors
         */

        /**
         * Arduino Constructor
         *
         * Empty constructor. Creates a new instance of this configuration holder.
         * @note Uses default configuration (TX transceiver mode);
         *
         * @note Default configuration:
         *
         *      setTransceiverMode(Mode_PTX);
         *      setCRC(CRC_16);
         *      for (int p = 0; p < 6; ++p)
         *      {
         *          enableRxPipeAutoAck((RxPipe) p);
         *      }
         *      disableAllRxPipeAddresses();
         *      setAddressWidth(Width_5Bytes);
         *      setAutoRtCount(MAX_RT_COUNT);
         *      setAutoRtDelay(1500);
         *      setRFChannel(2);
         *      setOutputPower(OutputPower_0dBm);
         *      setDataRate(DataRate_1Mbps);
         *      disableConstantCarrier();
         *      disablePllLock();
         *      disableDynamicPayloads();
         *      disableAckPayload();
         *      disableDynamicAck();
         */
        Configuration();

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this configuration holder.
         *
         * @param mode Transceiver mode
         */
        Configuration(TransceiverMode mode);

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this configuration holder.
         *
         * @param mode Transceiver mode
         * @param level Output power level
         * @param dataRate Communication data rate
         */
        Configuration(TransceiverMode mode, OutputPower level, DataRate dataRate);

    public:

        /**
         * @name Configuration setters
         */

        /**
         * Set transceiver mode
         * @param mode Transceiver mode
         */
        void setTransceiverMode(TransceiverMode mode);

        /**
         * Set output power
         * @param level Output power
         */
        void setOutputPower(OutputPower level);

        /**
         * Set communication data rate
         * @param dataRate Communication data rate
         */
        void setDataRate(DataRate dataRate);

        /**
         * Set RF channel
         * @param channel RF channel
         */
        void setRFChannel(uint8_t channel);

        /**
         * Enable constant carrier
         */
        void enableConstantCarrier();

        /**
         * Disable constant carrier
         */
        void disableConstantCarrier();

        /**
         * Force PLL lock
         */
        void forcePllLock();

        /**
         * Disable PLL lock
         */
        void disablePllLock();

        /**
         * Set CRC length
         * @param length Length
         */
        void setCRC(CRCLength length);

        /**
         * Set address width
         * @param width Address width
         */
        void setAddressWidth(AddressWidth width);

        /**
         * Enable Rx Pipe
         * @param pipe Rx Pipe
         */
        void enableRxPipeAddress(RxPipe pipe);

        /**
         * Disable Rx Pipe
         * @param pipe Rx Pipe
         */
        void disableRxPipeAddress(RxPipe pipe);

        /**
         * Disable all Rx Pipes
         */
        void disableAllRxPipeAddresses();

        /**
         * Set Rx Pipe payload size
         * @param pipe Rx Pipe
         * @param size Payload size
         */
        void setRxPipePayloadSize(RxPipe pipe, uint8_t size);

        /**
         * Set Rx Pipe address
         * @param pipe Rx Pipe
         * @param address Address bytes
         */
        void setRxPipeAddress(RxPipe pipe, uint8_t* address);

        /**
         * Set Tx address
         * @param address Address bytes
         */
        void setTxAddress(uint8_t *address);

        /**
         * Enable Rx Pipe auto ACK
         */
        void enableRxPipeAutoAck(RxPipe pipe);

        /**
         * Disable Rx Pipe auto ACK
         */
        void disableRxPipeAutoAck(RxPipe pipe);

        /**
         * Disable all Rx Pipes auto ACK
         */
        void disableAutoAck();

        /**
         * Set auto retransmissions delay
         * @param delay Auto retransmission delay (ms)
         */
        void setAutoRtDelay(uint16_t delay);

        /**
         * Set MAX retransmissions count
         * @param count Retransmission count
         */
        void setAutoRtCount(uint8_t count);

        /**
         * Enable Rx Pipe dynamic payloads
         * @param pipe Rx Pipe
         */
        void enableRxPipeDynamicPayload(RxPipe pipe);

        /**
         * Disable Rx Pipe dynamic payloads
         * @param pipe
         */
        void disableRxPipeDynamicPayload(RxPipe pipe);

        /**
         * Disable all Rx Pipes dynamic payloads
         */
        void disableDynamicPayloads();

        /**
         * Enable ACK payload
         */
        void enableAckPayload();

        /**
         * Disable ACK payloads
         */
        void disableAckPayload();

        /**
         * Enable dynamic ACKs
         */
        void enableDynamicAck();

        /**
         * Disable dynamic ACKs
         */
        void disableDynamicAck();

    private:

        Register::CONFIG _config = {.raw = 0x0C};
        Register::EN_AA _enAA = {.raw = 0x3F};
        Register::EN_RXADDR _enRxAddr = {.raw = 0x00};
        Register::SETUP_AW _setupAw = {.raw = 0x3};
        Register::SETUP_RETR _setupRetr = {.raw = 0x5F} ;
        Register::RF_CH _rfCh = {.raw = 0x02};
        Register::RF_SETUP _rfSetup = {.raw = 0x06};
        Register::RX_PW_PN _rxPwPN[6] = {{.raw = 0x00}, {.raw = 0x00}, {.raw = 0x00}, {.raw = 0x00}, {.raw = 0x00}, {.raw = 0x00}};
        Register::DYNPD _dynpd = {.raw = 0x00};
        Register::FEATURE _feature = {.raw = 0x00};

        uint8_t _txAddr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
        uint8_t _rxPipeAddrLong[2][5] = {{ 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 }, { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 }};
        uint8_t _rxPipeAddrShort[4] = { 0xC3, 0xC4, 0xC5, 0xC6};

        friend class Driver;
   };
}

#endif //NRF24_CONFIGURATION_H
