/**
 * @file Registers.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24_REGISTERS_H
#define NRF24_REGISTERS_H

#include "Arduino.h"

namespace NRF24
{
    /**
     * @name NRF24 Register Definitions
     */

    namespace Register
    {
        /**
         * Configuration register
         */
        union CONFIG
        {
            struct
            {
                /**
                 * RX/TX mode control
                 * @retval FALSE TX mode
                 * @retval TRUE RX mode
                 */
                bool PRIM_RX     :1;

                /**
                 * Power up/down control
                 * @retval FALSE Power down
                 * @retval TRUE Power up
                 */
                bool PWR_UP      :1;

                /**
                 * CRC encoding scheme
                 * @retval FALSE 1 byte CRC scheme
                 * @retval TRUE 2 bytes CRC scheme
                 */
                bool CRCO        :1;

                /**
                 * Enable CRC
                 * @note Forced high if one of the bits in {@link EN_AA} is high.
                 */
                bool EN_CRC      :1;

                /**
                 * Mask interrupt caused by MAX_RT
                 * @retval FALSE Reflect MAX_RT as active low interrupt on the IRQ pin
                 * @retval TRUE Interrupt not reflected on the IRQ pin
                 */
                bool MASK_MAX_RT :1;

                /**
                 * Mask interrupt caused by TX_DS
                 * @retval FALSE Reflect TX_DS as active low interrupt on the IRQ pin
                 * @retval TRUE Interrupt not reflected on the IRQ pin
                 */
                bool MASK_TX_DS  :1;

                /**
                 * Mask interrupt caused by RX_DR
                 * @retval FALSE Reflect RX_DR as active low interrupt on the IRQ pin
                 * @retval TRUE Interrupt not reflected on the IRQ pin
                 */
                bool MASK_RX_DR  :1;    /**< Mask interrupt caused by RX_DR */

                /**
                 * Reserved
                 */
                bool             :1; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Enable 'Auto Acknowledgement' Function register
         * @note Disable this functionality to be compatible with nRF2401
         */
        union EN_AA
        {
            struct
            {
                /**
                 * Enable 'Auto Acknowledgement' for data pipe 0
                 */
                bool ENAA_P0 :1;

                /**
                 * Enable 'Auto Acknowledgement' for data pipe 1
                 */
                bool ENAA_P1 :1;

                /**
                 * Enable 'Auto Acknowledgement' for data pipe 2
                 */
                bool ENAA_P2 :1;

                /**
                 * Enable 'Auto Acknowledgement' for data pipe 3
                 */
                bool ENAA_P3 :1;

                /**
                 * Enable 'Auto Acknowledgement' for data pipe 4
                 */
                bool ENAA_P4 :1;

                /**
                 * Enable 'Auto Acknowledgement' for data pipe 5
                 */
                bool ENAA_P5 :1;

                /** Reserved */
                bool         :2; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Enable RX Address register
         */
        union EN_RXADDR
        {
            struct
            {
                /**
                 * Enable data pipe 0
                 */
                bool ERX_P0 :1;

                /**
                 * Enable data pipe 1
                 */
                bool ERX_P1 :1;

                /**
                 * Enable data pipe 2
                 */
                bool ERX_P2 :1;

                /**
                 * Enable data pipe 3
                 */
                bool ERX_P3 :1;

                /**
                 * Enable data pipe 4
                 */
                bool ERX_P4 :1;

                /**
                 * Enable data pipe 5
                 */
                bool ERX_P5 :1;

                /**
                 * Reserved
                 */
                bool        :2; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Address Widths setup register
         * @note Common for all data pipes
         */
        union SETUP_AW
        {
            struct
            {
                /**
                 * RX/TX Address field width
                 *
                 * @retval 0 Illegal value
                 * @retval 1 3 bytes address width
                 * @retval 2 4 bytes address width
                 * @retval 3 5 bytes address width
                 *
                 * @note LSByte is used if address width is below 5 bytes
                 */
                unsigned int AW :2;

                /**
                 * Reserved
                 */
                unsigned int    :6; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Automatic Retransmission setup register
         */
        union SETUP_RETR
        {
            struct
            {
                /**
                 * Auto Retransmit Count
                 *
                 * @retval 0000 Re-Transmit disabled
                 * @retval 0001 Up to 1 Re-Transmit on fail of AA
                 * @retval ......
                 * @retval 1111 Up to 15 Re-Transmit on fail of AA
                 */
                unsigned int ARC :4;

                /**
                 * Auto Retransmit Delay
                 *
                 * @retval 0000 Wait 250μS
                 * @retval 0001 Wait 500μS
                 * @retval 0010 Wait 750μS
                 * @retval ......
                 * @retval 1111 Wait 4000μS
                 *
                 * @note Delay defined from end of transmission to start of next transmission
                 */
                unsigned int ARD :4;
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * RF Channel register
         */
        union RF_CH
        {
            struct
            {
                /**
                 * Sets the frequency channel nRF24L01+ operates on
                 */
                unsigned int RF_CH :7;

                /**
                 * Reserved
                 */
                bool               :1; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * RF Setup register
         */
        union RF_SETUP
        {
            struct
            {
                /**
                 * Obsolete (nRF24L01 bit mnemonic)
                 */
                bool LNA_HCURR      :1; /* obsolete */

                /**
                 * Set RF output power in TX Mode
                 *
                 * @retval 0 -18dBm
                 * @retval 1 -12dBm
                 * @retval 2 -6dBm
                 * @retval 3 0dBm
                 */
                unsigned int RF_PWR :2;

                /**
                 * Select between the high speed data rates.
                 * This bit is don’t care if {@link RF_DR_LOW} is set.
                 *
                 *  <h3>Encoding:</h3>
                 *
                 *  <table>
                 *    <tr>
                 *      <th>RF_DR_LOW</th>
                 *      <th>RF_DR_HIGH</th>
                 *      <th>Description</th>
                 *    </tr>
                 *    <tr>
                 *      <td>FALSE</td>
                 *      <td>FALSE</td>
                 *      <td>1 Mbps datarate</td>
                 *    </tr>
                 *    <tr>
                 *      <td>FALSE</td>
                 *      <td>TRUE</td>
                 *      <td>2 Mbps datarate</td>
                 *    </tr>
                 *    <tr>
                 *      <td>TRUE</td>
                 *      <td>FALSE</td>
                 *      <td>250 kbps datarate</td>
                 *    </tr>
                 *    <tr>
                 *      <td>TRUE</td>
                 *      <td>TRUE</td>
                 *      <td>Reserved</td>
                 *    </tr>
                 *  </table>
                 */
                bool RF_DR_HIGH     :1;

                /**
                 * Force PLL lock signal
                 */
                bool PLL_LOCK       :1;

                /**
                 * Set RF Data Rate to 250kbps
                 * @note See {@link RF_DR_HIGH} for encoding.
                 */
                bool RF_DR_LOW      :1;

                /**
                 * Reserved
                 */
                bool                :1; /* reserved */

                /**
                 * Enables continuous carrier transmit when high
                 */
                bool CONT_WAVE      :1;
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Status register
         */
        union STATUS
        {
            struct
            {
                /**
                 * TX FIFO full flag.
                 *
                 * @retval FALSE Available locations in TX FIFO.
                 * @retval TRUE TX FIFO full.
                 */
                bool TX_FULL         :1;

                /**
                 * Data pipe number for the payload available for reading from RX_FIFO
                 *
                 * @retval 000-101 Data Pipe Number
                 * @retval 110 Not Used
                 * @retval 111 RX FIFO Empty
                 */
                unsigned int RX_P_NO :3;

                /**
                 * Maximum number of TX retransmits interrupt.
                 * If MAX_RT is asserted it must be cleared to enable further communication.
                 *
                 * @note Write 1 to clear bit.
                 */
                bool MAX_RT          :1;

                /**
                 * Data Sent TX FIFO interrupt.
                 * Asserted when packet transmitted on TX. If AUTO_ACK is activated,
                 * this bit is set high only when ACK is received.
                 *
                 * @note Write 1 to clear bit.
                 */
                bool TX_DS           :1;

                /**
                 * Data Ready RX FIFO interrupt.
                 * Asserted when new data arrives RX FIFO.
                 *
                 * @note Write 1 to clear bit.
                 */
                bool RX_DR           :1;

                /**
                 * Reserved
                 */
                bool                 :1; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Transmit Observe register
         */
        union OBSERVE_TX
        {
            struct
            {
                /**
                 * Count retransmitted packets.
                 *
                 * The counter is reset when transmission of a new packet starts.
                 */
                unsigned int ARC_CNT  :4;

                /**
                 * Count lost packets.
                 *
                 * The counter is overflow protected to 15, and discontinues at max until reset.
                 *
                 * @note The counter is reset by writing to RF_CH.
                 */
                unsigned int PLOS_CNT :4;
            };


            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Received Power Detector (Carrier Detect)
         */
        union RPD
        {
            struct
            {
                /**
                 * Received Power Detector.
                 *
                 * @note
                 *      This register is called CD (Carrier Detect) in the nRF24L01.
                 *      The name is different in nRF24L01+ due to the different input power level threshold for this bit.
                 */
                bool RPD :1;

                /**
                 * Reserved
                 */
                bool     :7; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Rx Pipe Payload Width register
         */
        union RX_PW_PN
        {
            struct
            {
                /**
                 * Number of bytes in RX payload in data pipe N (1 to 32 bytes).
                 *
                 * @retval 0 Pipe not used
                 */
                unsigned int RX_PW_PN :6;

                /**
                 * Reserved
                 */
                bool                  :2; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * FIFO Status register
         */
        union FIFO_STATUS
        {
            struct
            {
                /**
                 * RX FIFO empty flag.
                 *
                 * @retval FALSE Data in RX FIFO.
                 * @retval TRUE RX FIFO empty.
                 */
                bool RX_EMPTY :1;

                /**
                 * RX FIFO full flag.
                 *
                 * @retval FALSE Available locations in RX FIFO.
                 * @retval TRUE RX FIFO full.
                 */
                bool RX_FULL  :1;

                /**
                 * Reserved
                 */
                bool          :2; /* reserved */

                /**
                 * TX FIFO empty flag.
                 *
                 * @retval FALSE Data in TX FIFO.
                 * @retval TRUE TX FIFO empty.
                 */
                bool TX_EMPTY :1;

                /**
                 * TX FIFO full flag.
                 *
                 * @retval FALSE Available locations in TX FIFO.
                 * @retval TRUE TX FIFO full.
                 */
                bool TX_FULL  :1;

                /**
                 * Reuse TX payload.
                 *
                 * Used for a PTX device. Pulse the CE signal high for at least 10μs to Reuse last transmitted payload.
                 * TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX_REUSE is set by the SPI
                 * command REUSE_TX_PL, and is reset by the SPI commands W_TX_PAYLOAD or FLUSH TX.
                 */
                bool TX_REUSE :1;

                /**
                 * Reserved
                 */
                bool          :1; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Enable Dynamic Payload Length register
         */
        union DYNPD
        {
            struct
            {
                /**
                 * Enable dynamic payload length data pipe 0.
                 *
                 * @note Requires EN_DPL and ENAA_P0
                 */
                bool DPL_P0 :1;

                /**
                 * Enable dynamic payload length data pipe 1.
                 *
                 * @note Requires EN_DPL and ENAA_P1
                 */
                bool DPL_P1 :1;

                /**
                 * Enable dynamic payload length data pipe 2.
                 *
                 * @note Requires EN_DPL and ENAA_P2
                 */
                bool DPL_P2 :1;

                /**
                 * Enable dynamic payload length data pipe 3.
                 *
                 * @note Requires EN_DPL and ENAA_P3
                 */
                bool DPL_P3 :1;

                /**
                 * Enable dynamic payload length data pipe 4.
                 *
                 * @note Requires EN_DPL and ENAA_P4
                 */
                bool DPL_P4 :1;

                /**
                 * Enable dynamic payload length data pipe 5.
                 *
                 * @note Requires EN_DPL and ENAA_P5
                 */
                bool DPL_P5 :1;

                /**
                 * Reserved
                 */
                bool        :2; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };

        /**
         * Feature register
         */
        union FEATURE
        {
            struct
            {
                /**
                 * Enables the W_TX_PAYLOAD_NOACK command
                 */
                bool EN_DYN_ACK :1;

                /**
                 * Enables Payload with ACK
                 */
                bool EN_ACK_PAY :1;

                /**
                 * Enables Dynamic Payload Length
                 */
                bool EN_DPL     :1;

                /**
                 * Reserved
                 */
                bool            :5; /* reserved */
            };

            /**
             * Raw register content
             */
            uint8_t raw;
        };
    }
}

#endif //NRF24_REGISTERS_H
