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
         */
        Configuration()
        {
            setDefaultConfiguration();
        };

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this configuration holder.
         *
         * @param mode Transceiver mode
         */
        Configuration(TransceiverMode mode)
        {
            setDefaultConfiguration();

            this->setTransceiverMode(mode);
        };

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this configuration holder.
         *
         * @param mode Transceiver mode
         * @param level Output power level
         * @param dataRate Communication data rate
         */
        Configuration(TransceiverMode mode, OutputPower level, DataRate dataRate)
        {
            setDefaultConfiguration();

            this->setTransceiverMode(mode);
            this->setOutputPower(level);
            this->setDataRate(dataRate);
        };

    private:

        /**
         * Set default configuration to this configuration holder instance
         */
        inline void setDefaultConfiguration()
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

    public:

        /**
         * @name Configuration setters
         */

        /**
         * Set transceiver mode
         * @param mode Transceiver mode
         */
        void setTransceiverMode(TransceiverMode mode)
        {
            _config.PRIM_RX = (mode != Mode_PTX);
        }

        /**
         * Set output power
         * @param level Output power
         */
        void setOutputPower(OutputPower level)
        {
            _rfSetup.RF_PWR = (unsigned int) level;
        }

        /**
         * Set communication data rate
         * @param dataRate Communication data rate
         */
        void setDataRate(DataRate dataRate)
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

        /**
         * Set RF channel
         * @param channel RF channel
         */
        void setRFChannel(uint8_t channel)
        {
            _rfCh.RF_CH = min(channel, MAX_RF_CHANNEL);
        }

        /**
         * Enable constant carrier
         */
        void enableConstantCarrier()
        {
            _rfSetup.CONT_WAVE = true;
        }

        /**
         * Disable constant carrier
         */
        void disableConstantCarrier()
        {
            _rfSetup.CONT_WAVE = false;
        }

        /**
         * Force PLL lock
         */
        void forcePllLock()
        {
            _rfSetup.PLL_LOCK = true;
        }

        /**
         * Disable PLL lock
         */
        void disablePllLock()
        {
            _rfSetup.PLL_LOCK = false;
        }

        /**
         * Set CRC length
         * @param length Length
         */
        void setCRC(CRCLength length)
        {
            _config.EN_CRC = (length != CRC_DISABLED);
            _config.CRCO = (length == CRC_16);
        }

        /**
         * Set address width
         * @param width Address width
         */
        void setAddressWidth(AddressWidth width)
        {
            _setupAw.AW = width;
        }

        /**
         * Enable Rx Pipe
         * @param pipe Rx Pipe
         */
        void enableRxPipeAddress(RxPipe pipe)
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

        /**
         * Disable Rx Pipe
         * @param pipe Rx Pipe
         */
        void disableRxPipeAddress(RxPipe pipe)
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

        /**
         * Disable all Rx Pipes
         */
        void disableAllRxPipeAddresses()
        {
            _enRxAddr.raw = 0x00;
            for (int p = 0; p < sizeof(_rxPwPN); ++p)
            {
                _rxPwPN[p].RX_PW_PN = PIPE_NOT_USED;
            }
        }

        /**
         * Set Rx Pipe payload size
         * @param pipe Rx Pipe
         * @param size Payload size
         */
        void setRxPipePayloadSize(RxPipe pipe, uint8_t size)
        {
            _rxPwPN[pipe].RX_PW_PN = min(size, MAX_PAYLOAD_SIZE);
        }

        /**
         * Set Rx Pipe address
         * @param pipe Rx Pipe
         * @param address Address bytes
         */
        void setRxPipeAddress(RxPipe pipe, uint8_t* address)
        {
            if (pipe < 2) {
                memcpy(_rxPipeAddrLong[pipe], address, _setupAw.AW);
            } else {
                _rxPipeAddrShort[pipe] = address[0];
            }
        }

        /**
         * Set Tx address
         * @param address Address bytes
         */
        void setTxAddress(uint8_t *address)
        {
            memcpy(_txAddr, address, _setupAw.AW);
        }

        /**
         * Enable Rx Pipe auto ACK
         */
        void enableRxPipeAutoAck(RxPipe pipe)
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

        /**
         * Disable Rx Pipe auto ACK
         */
        void disableRxPipeAutoAck(RxPipe pipe)
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

        /**
         * Disable all Rx Pipes auto ACK
         */
        void disableAutoAck()
        {
            _enAA.raw = 0x00;
        }

        /**
         * Set auto retransmissions delay
         * @param delay Auto retransmission delay (ms)
         */
        void setAutoRtDelay(uint16_t delay)
        {
            _setupRetr.ARD = constrain(delay, MIN_RT_DELAY, MAX_RT_DELAY)/250 - 1;
        }

        /**
         * Set MAX retransmissions count
         * @param count Retransmission count
         */
        void setAutoRtCount(uint8_t count)
        {
            _setupRetr.ARC = min(count, MAX_RT_COUNT);
        }

        /**
         * Enable Rx Pipe dynamic payloads
         * @param pipe Rx Pipe
         */
        void enableRxPipeDynamicPayload(RxPipe pipe)
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

        /**
         * Disable Rx Pipe dynamic payloads
         * @param pipe
         */
        void disableRxPipeDynamicPayload(RxPipe pipe)
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

        /**
         * Disable all Rx Pipes dynamic payloads
         */
        void disableDynamicPayloads()
        {
            _dynpd.raw = 0x00;
        }

        /**
         * Enable ACK payload
         */
        void enableAckPayload()
        {
            _feature.EN_DYN_ACK = true;
        }

        /**
         * Disable ACK payloads
         */
        void disableAckPayload()
        {
            _feature.EN_DYN_ACK = false;
        }

        /**
         * Enable dynamic ACKs
         */
        void enableDynamicAck()
        {
            _feature.EN_DYN_ACK = true;
        }

        /**
         * Disable dynamic ACKs
         */
        void disableDynamicAck()
        {
            _feature.EN_DYN_ACK = false;
        }

    private:

        Register::CONFIG _config;
        Register::EN_AA _enAA;
        Register::EN_RXADDR _enRxAddr;
        Register::SETUP_AW _setupAw;
        Register::SETUP_RETR _setupRetr;
        Register::RF_CH _rfCh;
        Register::RF_SETUP _rfSetup;
        Register::RX_PW_PN _rxPwPN[6];
        Register::DYNPD _dynpd;
        Register::FEATURE _feature;

        uint8_t _txAddr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
        uint8_t _rxPipeAddrLong[2][5] = {{ 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 },
                                         { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 }};
        uint8_t _rxPipeAddrShort[4] = { 0xC3, 0xC4, 0xC5, 0xC6 };

        friend class Driver;
    };
}

#endif //NRF24_CONFIGURATION_H
