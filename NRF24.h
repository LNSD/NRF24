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
#include "Definitions.h"
#include "Configuration.h"

namespace NRF24
{
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
