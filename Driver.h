/**
 * @file Driver.h
 *
 * Copyright (C) 2016 Lorenzo Delgado <lorenzo.delgado@lnsd.es>
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NRF24_DRIVER_H
#define NRF24_DRIVER_H

#include "Arduino.h"
#include "Registers.h"

namespace NRF24
{
    /**
     * Low level nRF24L01(+) SPI device driver
     */
    class Driver
    {
    private:
        uint8_t _sck, _mosi, _miso, _csn;
        uint8_t _ce;

        /**
         * @name Low level signals control
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
         * @name SPI command
         */

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
        inline void spiCmdTransfer(uint8_t cmd, void* buf, size_t len);


        /**
         * @name SPI command-status
         */

        /**
         * Low-level SPI command wrapper
         * @param cmd SPI command
         * @return STATUS register content
         */
        inline Register::STATUS spiCmdStatusTransfer(uint8_t cmd);

        /**
         * Low-level SPI command wrapper
         * @param cmd Preceding SPI command
         * @param buf Array of data to be transferred
         * @param len Array data length
         * @return STATUS register content
         */
        inline Register::STATUS spiCmdStatusTransfer(uint8_t cmd, void* buf, size_t len);

        /**
         * @name Register manipulation
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

    public:

        /**
         * @name Constructors
         */

        /**
        * Arduino Constructor
        *
        * Creates a new instance of this driver. Before using, create an instance and pass
        * the unique pins that this chip is attached to.
        *
        * Instantiates a new {@link DeviceControllerDriver} class using Arduino UNO's hardware SPI.
        *
        * @param csn SPI chip select pin (CS/SSEL)
        * @param ce SPI chip enable pin (CE)
        */
        Driver(uint8_t csn, uint8_t ce);

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this driver. Before using, create an instance and pass
         * the unique pins that this chip is attached to.
         *
         * Instantiates a new {@link DeviceControllerDriver} class using hardware SPI.
         *
         * @param sck SPI clock signal
         * @param mosi SPI Master Output - Slave Input
         * @param miso SPI Master Input - Slave Output
         * @param csn SPI chip select pin (CS/SSEL)
         */
        Driver(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t csn, uint8_t ce);


        /**
         * @name Configuration
         */

        /**
         * Setup hardware
         */
        void configure();

        /**
         * Setup hardware
         * @param spi SPI clock frequency
         */
        void configure(uint32_t spi);


        /**
         * @name Chip control signals
         */

        /**
         * Set CE signal high
         */
        void chipEnable();

        /**
         * Set CE signal low
         */
        void chipDisable();


        /**
         * @name Register manipulation
         */

        /**
         * Read CONFIG register content
         * @return Register content
         */
        Register::CONFIG readConfigRegister();

        /**
         * Write CONFIG register content
         * @param reg Register content
         */
        void writeConfigRegister(Register::CONFIG reg);

        /**
         * Read EN_AA register content
         * @return Register content
         */
        Register::EN_AA readEnAARegister();

        /**
         * Write EN_AA register content
         * @param reg Register content
         */
        void writeEnAARegister(Register::EN_AA reg);

        /**
         * Read EN_RXADDR register content
         * @return Register content
         */
        Register::EN_RXADDR readEnRxAddrRegister();

        /**
         * Write EN_RXADDR register content
         * @param reg Register content
         */
        void writeEnRxAddrRegister(Register::EN_RXADDR reg);

        /**
         * Read SETUP_AW register content
         * @return Register content
         */
        Register::SETUP_AW readSetupAWRegister();

        /**
         * Write SETUP_AW register content
         * @param reg Register content
         */
        void writeSetupAWRegister(Register::SETUP_AW reg);

        /**
         * Read SETUP_RETR register content
         * @return Register content
         */
        Register::SETUP_RETR readSetupRetrRegister();

        /**
         * Write SETUP_RETR register content
         * @param reg Register content
         */
        void writeSetupRetrRegister(Register::SETUP_RETR reg);

        /**
         * Read RF_CH register content
         * @return Register content
         */
        Register::RF_CH readRFChannelRegister();

        /**
         * Write RF_CH register content
         * @param reg Register content
         */
        void writeRFChannelRegister(Register::RF_CH reg);

        /**
         * Read RF_SETUP register content
         * @return Register content
         */
        Register::RF_SETUP readRFSetupRegister();

        /**
         * Write RF_SETUP register content
         * @param reg Register content
         */
        void writeRFSetupRegister(Register::RF_SETUP reg);

        /**
         * Read STATUS register content
         * @return Register content
         */
        Register::STATUS readStatusRegister();

        /**
         * Write STATUS register content
         * @param reg Register content
         */
        void writeStatusRegister(Register::STATUS reg);

        /**
         * Read OBSERVE_TX register content
         * @return Register content
         */
        Register::OBSERVE_TX readObserveTxRegister();

        /**
         * Write OBSERVE_TX register content
         * @param reg Register content
         */
        void writeObserveTxRegister(Register::OBSERVE_TX reg);

        /**
         * Read RPD register content
         * @return Register content
         */
        Register::RPD readRPDRegister();

        /**
         * Write RPD register content
         * @param reg Register content
         */
        void writeRPDRegister(Register::RPD reg);

        /**
         * Read RX_ADDR_Pn register
         * @param pipe Rx pipe number
         * @param addr Pipe address buffer
         * @param len Address length
         */
        void readRxPipeAddrRegister(uint8_t pipe, uint8_t* addr, uint8_t len);

        /**
         * Write RX_ADDR_Pn register
         * @param pipe Rx pipe number
         * @param addr Pipe address
         * @param len Address length
         */
        void writeRxPipeAddrRegister(uint8_t pipe, uint8_t* addr, uint8_t len);

        /**
         * Read TX_ADDR register
         * @param addr Current address
         * @param len Adress length
         */
        void readTxAddrRegister(uint8_t* addr, uint8_t len);

        /**
         * Write TX_ADDR register
         * @param addr Address
         * @param len Address length
         */
        void writeTxAddrRegister(uint8_t* addr, uint8_t len);

        /**
         * Read RX_PW_Pn register content
         * @param pipe Rx pipe number
         * @return Register content
         */
        Register::RX_PW_PN readRxPWRegister(uint8_t pipe);

        /**
         * Write RX_PW_Pn register content
         * @param pipe Rx pipe
         * @param reg Register content
         */
        void writeRxPWRegister(uint8_t pipe, Register::RX_PW_PN reg);

        /**
         * Read FIFO_STATUS register content
         * @return Register content
         */
        Register::FIFO_STATUS readFifoStatusRegister();

        /**
         * Write FIFO_STATUS register content
         * @param reg Register content
         */
        void writeFifoStatusRegister(Register::FIFO_STATUS reg);

        /**
         * Read DYNPD register content
         * @return Register content
         */
        Register::DYNPD readDYNPDRegister();

        /**
         * Write DYNPD register content
         * @param reg Register content
         */
        void writeDYNPDRegister(Register::DYNPD reg);

        /**
         * Read FEATURE register content
         * @return Register content
         */
        Register::FEATURE readFeatureRegister();

        /**
         * Write FEATURE register content
         * @param reg Register content
         */
        void writeFeatureRegister(Register::FEATURE reg);


        /**
         * @name Commands
         */

        /**
         * Read RX-payload (R_RX_PAYLOAD command)
         * @param data Payload buffer
         * @param len Payload length
         */
        void readRxPayload(uint8_t* data, uint8_t len);

        /**
         * Write TX-payload (W_TX_PAYLOAD command)
         * @param data Payload buffer
         * @param len Payload length
         */
        void writeTxPayload(uint8_t* data, uint8_t len);

        /**
         * Flush Tx FIFO (FLUSH_TX command)
         */
        void flushTxFifo();

        /**
         * Flush Rx FIFO (FLUSH_RX command)
         * @note Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
         */
        void flushRxFifo();

        /**
         * Reuse last transmitted payload (REUSE_TX_PAYLOAD command)
         * @note Tx payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. Tx payload reuse must not be activated or deacti- vated during package transmission.
         */
        void reuseTxPayload();

        /**
         * Read RX payload width for the top R_RX_PAYLOAD in the Rx FIFO (R_RX_PL_WID command)
         * @Note Flush RX FIFO if the read value is larger than 32 bytes
         * @return RX payload width
         */
        uint8_t readRxPayloadLength();

        /**
         * Write Payload to be transmitted together with ACK packet on chosen pipe (W_ACK_PAYLOAD command)
         * @note Maximum three ACK packet payloads can be pending. Payloads with same # are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0.
         * @param pipe Pipe where payload is written
         * @param data Payload buffer
         * @param len Payload length
         */
        void writeACKPayload(uint8_t pipe, uint8_t* data, uint8_t len);

        /**
         * Write Payload to be transmitted and disable AUTOACK on this specific packet (W_TX_PAYLOAD_NOACK command)
         * @param data Payload buffer
         * @param len Payload length
         */
        void writeTxPayloadNOACK(uint8_t *data, uint8_t len);
    };
}

#endif //NRF24_DRIVER_H
