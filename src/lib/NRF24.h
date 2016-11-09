/**
 * @file NRF24.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24_H
#define NRF24_H

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include "NRF24L01P.h"

namespace NRF24
{
    /**
     * @name Constants
     */

    /**
     * Max Payload Size. Max size: 32 bytes wide
     */
    const static uint8_t MAX_PAYLOAD_SIZE = 32;

    /**
     * Max RF cCannel. 128 channels available
     */
    const static uint8_t MAX_RF_CHANNEL = 127;

    /**
     * Min Automatic Retransmission delay: Wait 250us
     */
    const static uint16_t MIN_RT_DELAY = 250;

    /**
     * Max Automatic Retransmission delay: Wait 4000us
     */
    const static uint16_t MAX_RT_DELAY = 4000;

    /**
     * Max Automatic Retransmission count: Up to 15 Re-Transmit on fail of AA
     */
    const static uint8_t MAX_RT_COUNT = 15;

    /**
     * Numbet of bytes in RX payload in data pipe: 0 - Pipe not used
     */
    const static uint8_t PIPE_NOT_USED = 0;

    /**
     * @name Enum definitions
     */

    /**
     * Transmision mode
     * @note For use with {@link setTransceiverMode()}
     */
    typedef enum {
        Mode_PTX = 0,
        Mode_PRX = 1
    } TransceiverMode;

    /**
     * Power Amplifier output level
     * @note For use with {@link setOutputRfPower()}
     */
    typedef enum {
        OutputPower_M18dBm = 0,	// -18dBm MIN
        OutputPower_M12dBm = 1,	// -12dBm LOW
        OutputPower_M6dBm  = 2,	// -6dBm HIGH
        OutputPower_0dBm   = 3	// 	0dBm MAX
    } OutputPower;

    /**
    * Data rate. How fast data moves through the air.
    * @note For use with {@link setDataRate()}
    */
    typedef enum {
        DataRate_1Mbps = 0,	// 1Mbps
        DataRate_2Mbps,		// 2Mbps
        DataRate_250kbps	// 250kbps
    } DataRate;

    /**
    * CRC Length. How big (if any) of a CRC is included.
    * @note For use with {@link enableCRC()}
    */
    typedef enum {
        CRC_8 = 0,
        CRC_16,
        CRC_DISABLED
    } CRCLength;

    /**
     * Address width definition
     * @note For use with {@link setAddressWidth()}
     */
    typedef enum {
        Width_3Bytes = 1,
        Width_4Bytes = 2,
        Width_5Bytes = 3
    } AddressWidth;

    /**
    * RX pipes definition
    */
    typedef enum {
        RX_P0 = 0,
        RX_P1 = 1,
        RX_P2 = 2,
        RX_P3 = 3,
        RX_P4 = 4,
        RX_P5 = 5
    } RxPipe;

    /**
     * Fifo status definition
     */
    typedef enum {
        FIFO_STATUS_EMPTY = 0,
        FIFO_STATUS_OK,
        FIFO_STATUS_FULL
    } FifoStatus;

    /**
     * @name Register definitions
     */
    namespace Register
    {
        /**
         * Configuration register
         */
        typedef union {
            struct {
                bool PRIM_RX     :1;
                bool PWR_UP      :1;
                bool CRCO        :1;
                bool EN_CRC      :1;
                bool MASK_MAX_RT :1;
                bool MASK_TX_DS  :1;
                bool MASK_RX_DR  :1;
                bool             :1; /* reserved */
            };
            uint8_t raw;
        } CONFIG;

        /**
         * Enable 'Auto Acknowledgement' Function
         * @note Disable this functionality to be compatible with nRF2401
         */
        typedef union {
            struct {
                bool ENAA_P0 :1;
                bool ENAA_P1 :1;
                bool ENAA_P2 :1;
                bool ENAA_P3 :1;
                bool ENAA_P4 :1;
                bool ENAA_P5 :1;
                bool         :2; /* reserved */
            };
            uint8_t raw;
        } EN_AA;

        /**
         * Enable RX Address
         */
        typedef union {
            struct {
                bool ERX_P0 :1;
                bool ERX_P1 :1;
                bool ERX_P2 :1;
                bool ERX_P3 :1;
                bool ERX_P4 :1;
                bool ERX_P5 :1;
                bool        :2; /* reserved */
            };
            uint8_t raw;
        } EN_RXADDR;

        /**
         * Setup of Address Widths (common for all data pipes)
         */
        typedef union {
            struct {
                unsigned int AW :2;
                unsigned int    :6; /* reserved */
            };
            uint8_t raw;
        } SETUP_AW;

        /**
         * Setup of Automatic Retransmission
         */
        typedef union {
            struct {
                unsigned int ARD :4;
                unsigned int ARC :4;
            };
            uint8_t raw;
        } SETUP_RETR;

        /**
         * RF Channel
         */
        typedef union {
            struct {
                unsigned int RF_CH :7;
                bool               :1; /* reserved */
            };
            uint8_t raw;
        } RF_CH;

        /**
         * RF Setup Register
         */
        typedef union {
            struct {
                bool LNA_HCURR      :1; /* obsolete (nRF24L01 bit mnemonic) */
                unsigned int RF_PWR :2;
                bool RF_DR_HIGH     :1;
                bool PLL_LOCK       :1;
                bool RF_DR_LOW      :1;
                bool                :1; /* reserved */
                bool CONT_WAVE      :1;
            };
            uint8_t raw;
        } RF_SETUP;

        /**
         * Status Register
         */
        typedef union {
            struct {
                bool TX_FULL         :1;
                unsigned int RX_P_NO :3;
                bool MAX_RT          :1;
                bool TX_DS           :1;
                bool RX_DR           :1;
                bool                 :1; /* reserved */
            };
            uint8_t raw;
        } STATUS;

        /**
         * Transmit observe register
         */
        typedef union {
            struct {
                unsigned int ARC_CNT  :4;
                unsigned int PLOS_CNT :4;
            };
            uint8_t raw;
        } OBSERVE_TX;

        /**
         * Received Power Detector (Carrier Detect)
         */
        typedef union {
            struct {
                bool RPD :1;
                bool     :7; /* reserved */
            };
            uint8_t raw;
        } RPD;

        /**
         * Rx Pipe Payload Width
         */
        typedef union {
            struct {
                unsigned int RX_PW_PN :6;
                bool                  :2; /* reserved */
            };
            uint8_t raw;
        } RX_PW_PN;

        /**
         * FIFO Status Register
         */
        typedef union {
            struct {
                bool RX_EMPTY :1;
                bool RX_FULL  :1;
                bool          :2; /* reserved */
                bool TX_EMPTY :1;
                bool TX_FULL  :1;
                bool TX_REUSE :1;
                bool          :1; /* reserved */
            };
            uint8_t raw;
        } FIFO_STATUS;

        /**
         * Enable dynamic payload length
         */
        typedef union {
            struct {
                bool DPL_P0 :1;
                bool DPL_P1 :1;
                bool DPL_P2 :1;
                bool DPL_P3 :1;
                bool DPL_P4 :1;
                bool DPL_P5 :1;
                bool        :2; /* reserved */
            };
            uint8_t raw;
        } DYNPD;

        /**
         * Feature Register
         */
        typedef union {
            struct {
                bool EN_DYN_ACK :1;
                bool EN_ACK_PAY :1;
                bool EN_DPL     :1;
                bool            :5; /* reserved */
            };
            uint8_t raw;
        } FEATURE;
    }

    /**
     * Configuration holder class
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

    /**
     * Driver for nRF24L01(+) 2,4GHz Wireless Transceiver
     */
    class Driver
    {
    public:

        /**
         * @name Constructors and configure subroutines
         */

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this driver. Before using, create an instance and pass
         * the unique pins that this chip is attached to.
         *
         * Instantiates a new NRF24 class using hardware SPI.
         *
         * @param csn SPI chip select pin (CS/SSEL)
         * @param ce SPI chip enable pin (CE)
         */
        Driver(uint8_t csn, uint8_t ce);              // Hardware SPI

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this driver. Before using, create an instance and pas
         * the unique pins that this chip is attached to.
         *
         * Instantiates a new NRF24 class using hardware SPI and IRQ.
         *
         * @param csn SPI chip select pin (CS/SSEL)
         * @param ce SPI chip enable pin (CE)
         * @param irq SPI chip interrupt pin (IRQ)
        */
        Driver(uint8_t csn, uint8_t c, uint8_t irq);  // Hardware SPI + IRQ

        /**
         * Setup the hardware
         */
        void configure();

        /**
         * Setup the hardware
         * @param configuration NRF24 configuration holder
         */
        void configure(Configuration configuration);

    private:
        /**
         * Transfer configuration parameters
         */
        inline void transferConfiguration(Configuration config);

    public:

        /**
         * @name Register read and write functions
         */

        /**
         * Read nRF24 register
         * @param reg Register to read
         * @return Register content
         */
        uint8_t readRegister(uint8_t reg);

        /**
         * Read nRF24 multi-byte register
         * @param reg Register to read
         * @param buf Array of data received
         * @param len Array length
        */
        void readRegister(uint8_t reg, uint8_t *buf, uint8_t len);

        /**
         * Write nRF24 register
         * @param reg Register to write
         * @param value Data to write
         */
        void writeRegister(uint8_t reg, uint8_t value);

        /**
         * Write nRF24 multi-byte register
         * @param reg Register to write
         * @param buf Array of data to write
         * @param len Array length
         */
        void writeRegister(uint8_t reg, uint8_t *buf, uint8_t len);

        /**
         * @name Configuration functions. Getters and setters
         */

        /**
         * Set transceiver mode
         * @param mode Transceiver mode
         */
        void setTransceiverMode(TransceiverMode mode);

        /**
         * Get current transceiver mode
         * @return Transceiver mode
         */
        TransceiverMode getTransceiverMode();

        /**
         * Enable constant carrier
         */
        void enableConstantCarrier();

        /**
         * Disable constant carrier
         */
        void disableConstantCarrier();

        /**
         * Check if constant carrier is enabled
         */
        bool isConstantCarrierEnabled();

        /**
         * Force PLL Lock signal
         * @note Only used in test
         */
        void forcePllLock();

        /**
         * Disable PLL Lock signal
         */
        void disablePllLock();

        /**
         * Check if PLL Lock is forced
         */
        bool isPllLockForced();

        /**
         * Set transceiver's output power level
         * @param level Output power level
         */
        void setOutputRFPower(OutputPower level);

        /**
         * Get current transceiver's output power level
         * @return Current output power
         */
        OutputPower getOutputRFPower();

        /**
         * Set transceiver's datarate
         * @param rate Datarate
         */
        void setDataRate(DataRate dataRate);

        /**
         * Get current transceiver's datarate
         * @return Current datarate
         */
        DataRate getDataRate();

        /**
         * Set transceiver's RF channel
         * @param channel RF channel
         */
        void setRFChannel(uint8_t channel);

        /**
         * Get transceiver's current RF channel
         * @return Current RF channel
         */
        uint8_t getRFChannel();

        /**
         * Set address width. Address width: 3 - 5 bytes.
         * @param width Address width
         */
        void setAddressWidth(AddressWidth width);

        /**
         * Get current address width
         * @return Current address width
         */
        AddressWidth getAddressWidth();

        /**
         * Enable RX pipe
         * @param pipe RX pipe
         */
        void enableRxPipeAddress(RxPipe pipe);

        /**
         * Disable RX pipe
         * @param pipe RX pipe
         */
        void disableRxPipeAddress(RxPipe pipe);

        /**
         * Get list of RX pipes status
         * @param pipes Boolean array holding status of all the RX pipes
         */
        Register::EN_RXADDR whichRxPipeAddrAreEnabled();

        /**
         * Set destination address (TX address)
         * @param addr Address
         * @param len Address length
         */
        void setTxAddress(uint8_t *addr, uint8_t len);

        /**
         * Get current destination address (TX address)
         * @param addr Current address
         * @param len Adress length
         */
        void getTxAddress(uint8_t *addr, uint8_t len);

        /**
         * Set input pipe address (RX pipe address)
         * @param pipe RX pipe
         * @param addr Pipe address
         * @param len Address length
         */
        void setRxPipeAddress(RxPipe pipe, uint8_t *addr, uint8_t len);

        /**
         * Get current input pipe address (RX pipe address)
         * @param pipe RX pipe
         * @param addr Pipe address
         * @param len Address length
         */
        void getRxPipeAddress(RxPipe pipe, uint8_t *addr, uint8_t len);

        /**
         * Set input pipe payload size. Max payload size: 32 bytes
         * @param pipe RX pipe
         * @param size Payload size
         */
        void setRxPipePayloadSize(RxPipe pipe, uint8_t size);

        /**
         * Get current input pipe payload size
         * @param pipe pipe
         * @return Payload size
         */
        uint8_t getRxPipePayloadSize(RxPipe pipe);

        /**
         * Enable input pipe dynamic payloads
         * @param pipe RX pipe
         */
        void enableRxPipeDynamicPayloads(RxPipe pipe);

        /**
         * Disable input pipe dynamic payloads
         * @param pipe RX pipe
         */
        void disableRxPipeDynamicPayloads(RxPipe pipe);

        /**
         * Disable all pipes dynamic payloads
         */
        void disableDynamicPayloads();

        /**
         * Check which inout pipe dynamic payloads are enabled
         * TODO return
         */
        Register::DYNPD whichRxPipeDynamicPayloadsAreEnabled();

        /**
         * Enable CRC and set length
         * @param length CRC length
         */
        void enableCRC(CRCLength length);

        /**
         * Disable CRC
         */
        void disableCRC();

        /**
         * Get current CRC configuration
         * @return Current CRC configuration
         */
        CRCLength getCRCConfig();

        /**
         * Enable input pipe auto ACK
         * @param pipe RX pipe
         */
        void enableRxPipeAutoAck(RxPipe pipe);

        /**
         * Disable input pipe auto ACK
         * @param pipe RX pipe
         */
        void disableRxPipeAutoAck(RxPipe pipe);

        /**
         * Get which input pipes have auto ACK enabled
         * TODO return
         */
        Register::EN_AA whichRxPipeAutoAckAreEnabled();

        /**
         * Set autoretransmission delay
         * @param delay Autoretransmission delay
         */
        void setAutoRtDelay(uint16_t delay);

        /**
         * Get current autoretransmission delay
         * @return Current autoretranmission delay (ms)
         */
        uint16_t getAutoRtDelay();

        /**
         * Set max autoretransmission retries
         * @param count Max autoretransmission retries
         */
        void setAutoRtCount(uint8_t count);

        /**
         * Get current max autoretransmission retries
         * @return Current max autoretransmission retries
         */
        uint8_t getAutoRtCount();

        /**
         * Enable ACK payload
         */
        void enableAckPayload();

        /**
         * Disable ACK payload
         */
        void disableAckPayload();

        /**
         * Get current ACK payload configuration
         */
        bool isAckPayloadEnabled();

        /**
         * Enable dynamic ACK
         */
        void enableDynamicAck();

        /**
         * Disable dynamic ACK
         */
        void disableDynamicAck();

        /**
         * Get current dynamic ACK configuration
         */
        bool isDynamicAckEnabled();

        /**
         * @name Command functions
         */

        /**
         * Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
         * @Note Flush RX FIFO if the read value is larger than 32 bytes
         * @return RX payload width
         */
        uint8_t getRxPayloadLength();

        /**
         * Read RX-payload: 1 - 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
         * @param data Payload buffer
         * @param len Payload length
         */
        void readRxPayload(uint8_t* data, uint8_t len);

        /**
         * Write TX-payload: 1 - 32 bytes. A read operation always starts at byte 0 used in TX payload.
         * @param data Payload buffer
         * @param len Payload length
         */
        void writeTxPayload(uint8_t* data, uint8_t len);

        /**
         * Used in RX mode. Write Payload to be transmitted together with ACK packet on chosen pipe.
         * @note Maximum three ACK packet payloads can be pending. Payloads with same # are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0.
         * @param pipe Pipe where payload is written
         * @param data Payload buffer
         * @param len Payload length
         */
        void writePipeACKPayload(RxPipe pipe, uint8_t* data, uint8_t len);

        /**
         * Used in TX mode. Disables AUTOACK on this specific packet.
         */
        void disableAAforPayload();

        /**
         * Used for a PTX device. Reuse last transmitted payload.
         * @note TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deacti- vated during package transmission.
         */
        void reuseTxPayload();

        /**
         * Flush TX FIFO, used in TX mode.
         */
        void flushTxFifo();

        /**
         * Flush RX FIFO, used in RX mode.
         * @note Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
         */
        void flushRxFifo();

        /**
         * @name Status functions
         */

        /**
         * Get count of lost packets. The counter is overflow pro- tected to 15, and discontinues at max until reset.
         * @note The counter is reset by writing to RF_CH.
         * @return Number of packets
         */
        uint8_t getLostPacketsCount();

        /**
         * Get count of retransmitted packets. The counter is reset when transmission of a new packet starts.
         * @return Number of packets
         */
        uint8_t getRtCount();

        /**
         * Check if carrier is detected
         * @note nRF24L01+ must be in receive mode.
         */
        bool isCarrierDetected();

        /**
         * Check if TX payload reuse is active (It should be active until W_TX_PAYLOAD or FLUSH TX is executed).
         */
        bool isReuseTxPayloadActive();

        /**
         * Get current TX FIFO status
         * @return FIFO status
         */
        FifoStatus getTxFifoStatus();

        /**
         * Get current RX FIFO status
         * @return FIFO status
         */
        FifoStatus getRxFifoStatus();

        /**
         * Reset RX_DR, TX_DS and MAX_RT bits.
         * @note Write 1 to clear bits.
         */
        void resetCurrentStatus();

        /**
         * Get current communication status
         * @param status
         */
        Register::STATUS getCommStatus();

        /**
         * @name Driver functions
         */

        /**
         * Set the PWR_UP bit in the CONFIG register high to enter standby-I mode.
         */
        void powerUp();

        /**
         * Set the PWR_UP bit in the CONFIG register low to entert Power down mode.
         */
        void powerDown();

        /**
         * Wake up transmitter. Move to "Standby-I" state
         */
        void begin();

        /**
         * Begin transmission. Move to "RX/TX Mode" state
         */
        void start();

        /**
         * Go back to "Standby-I" state
         */
        void stop();

        /**
         * Go to "Power down" state
         */
        void end();

        /**
         * @name Interrupt related functions
         */

        /**
         * @name Util functions
         */

        /**
         * Check if NRF24 chip is P variant or not
         * @return Whether is P variant ot not
         */
        bool isPVariant();

    private:
        uint8_t _sck, _mosi, _miso, _csn;
        uint8_t _ce, _irq;

        /**
         * @name Low-level signal & SPI-specific functions
         */

        /**
         * Low-level CSN signal enable/disable
         * @param level Output signal level
         */
        inline void csn(uint8_t val);

        /**
         * Low-level CE signal enable/disable
         * @param level Output signal level
         */
        inline void ce(uint8_t val);

        /**
         * Low-level SPI command wrapper
         * @param cmd SPI command
         */
        inline void spiCmdTransfer(uint8_t cmd);

        /**
         * Low-level SPI command wrapper
         * @param cmd Preceding SPI command
         * @param buf Array of data to be transferred
         * @param len Array data length
         */
        inline void spiCmdTransfer(uint8_t cmd, void *buf, size_t len);

    };

    /**
     * Debug class
     */
    class Debug
    {
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
        static void debugRxPipeAddressRegister(uint8_t *content, RxPipe pipe, uint8_t len);

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
        static void debugRxPipePayloadWidthRegister(uint8_t content, RxPipe pipe);

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
}

#endif //NRF24_H
