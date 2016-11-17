/**
 * @file Definitions.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24_DEFINITIONS_H
#define NRF24_DEFINITIONS_H

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
     * @name NRF24 Register Definitions
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
}

#endif //NRF24_DEFINITIONS_H