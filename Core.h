/**
 * @file Core.h
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

#include "Driver.h"

namespace NRF24
{
    /**
     * @name Constants
     */

    /**
     * Max Payload Size: 32 bytes wide
     */
    const static uint8_t MAX_PAYLOAD_LENGTH = 32;

    /**
     * Max RF Cannel (128 channels available)
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
     * Automatic Retransmission diabled
     */
    const static uint8_t AUTO_RT_DISABLED = 0;

    /**
     * Number of bytes in RX payload in data pipe: 0 - Pipe not used
     */
    const static uint8_t PIPE_NOT_USED = 0;


    /**
     * @name Enum definitions
     */

    /**
     * Transceiver mode.
     */
    typedef enum TransceiverMode
    {
        /**
         * TX transceiver mode
         */
                TX_Mode = 0,

        /**
         * RX transceiver mode
         */
                RX_Mode = 1
    };

    /**
     * Power amplifier output level.
     */
    typedef enum OutputPower
    {
        /**
         * MIN output power level (-18dBm)
         */
                OutputPower_MIN = 0,

        /**
         * LOW output power level (-12dBm)
         */
                OutputPower_LOW = 1,

        /**
         * HIGH output power level (-6dBm)
         */
                OutputPower_HIGH  = 2,

        /**
         * MAX output power level (0dBm)
         */
                OutputPower_MAX   = 3
    } ;

    /**
     * Datarate.
     */
    typedef enum DataRate
    {
        /**
         * 1Mbps datarate
         */
                DataRate_1Mbps = 0,

        /**
         * 2Mbps datarate
         */
                DataRate_2Mbps,

        /**
         * 250kbps datarate
         */
                DataRate_250kbps
    };

    /**
     * CRC Length.
     */
    typedef enum CRCLength
    {
        /**
         * 8 bits CRC length
         */
                CRC_8 = 0,

        /**
         * 16 bits CRC length
         */
                CRC_16,

        /**
         * CRC disabled
         */
                CRC_DISABLED
    };

    /**
     * Fifo Status
     */
    typedef enum FifoStatus
    {
        /**
         * FIFO status empty
         */
                FIFO_STATUS_EMPTY = 0,

        /**
         * FIFO status ok
         */
                FIFO_STATUS_OK,

        /**
         * FIFO status full
         */
                FIFO_STATUS_FULL
    };

    /**
     * Higher level Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
     */
    class Radio
    {
    private:
        Driver _driver;
        int8_t _irq;

        friend class Debugger;

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
         * Instantiates a new {@link Radio} class using the Arduino UNO's hardware SPI.
         *
         * @param csn SPI chip select pin (CSN)
         * @param ce SPI chip enable pin (CE)
         */
        Radio(uint8_t csn, uint8_t ce);

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this driver. Before using, create an instance and pass
         * the unique pins that this chip is attached to.
         *
         * Instantiates a new {@link Radio} class using the Arduino UNO's hardware SPI and
         * Arduino interrupts system.
         *
         * @param csn SPI chip select pin (CSN)
         * @param ce SPI chip enable pin (CE)
         * @param irq Maskable interrupt pin (IRQ)
         */
        Radio(uint8_t csn, uint8_t ce, uint8_t irq);

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this driver. Before using, create an instance and pas
         * the unique pins that this chip is attached to.
         *
         * Instantiates a new {@link Radio} class using hardware SPI.
         *
         * @param sck SPI clock pin (SCK)
         * @param mosi SPI slave data input pin (MOSI)
         * @param miso SPI slave data output pin (MISO)
         * @param csn SPI chip select pin (CSN)
         * @param ce SPI chip enable pin (CE)
         */
        Radio(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t csn, uint8_t ce);

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this driver. Before using, create an instance and pas
         * the unique pins that this chip is attached to.
         *
         * Instantiates a new {@link Radio} class using hardware SPI and Arduino interrupts
         * system.
         *
         * @param sck SPI clock pin (SCK)
         * @param mosi SPI slave data input pin (MOSI)
         * @param miso SPI slave data output pin (MISO)
         * @param csn SPI chip select pin (CSN)
         * @param ce SPI chip enable pin (CE)
         * @param irq Maskable interrupt pin (IRQ)
        */
        Radio(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t csn, uint8_t ce, uint8_t irq);


        /**
         * @name Hardware setup
         */

        /**
         * Setup the hardware
         */
        void configure();


        /**
         * @name Configuration getters and setters
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
         * Enable input pipe Auto ACK
         * @param pipe RX pipe
         */
        void enableRxPipeAutoAck(uint8_t pipe);

        /**
         * Disable input pipe Auto ACK
         * @param pipe RX pipe
         */
        void disableRxPipeAutoAck(uint8_t pipe);

        /**
         * Disable all pipes Auto ACK
         */
        void disableAutoAck();


        /**
         * Enable RX pipe
         * @param pipe RX pipe
         */
        void enableRxPipeAddress(uint8_t pipe);

        /**
         * Disable RX pipe
         * @param pipe RX pipe
         */
        void disableRxPipeAddress(uint8_t pipe);


        /**
         * Set address width.
         * Address width: 3 - 5 bytes.
         * @param width Address width
         */
        void setAddressWidth(uint8_t width);

        /**
         * Get current address width
         * @return Current address width
         */
        uint8_t getAddressWidth();


        /**
         * Set autoretransmission delay
         * @param delay Autoretransmission delay
         */
        void setAutoRetransmitDelay(uint16_t delay);

        /**
         * Get current autoretransmission delay
         * @return Current autoretranmission delay (ms)
         */
        uint16_t getAutoRetransmitDelay();

        /**
         * Set max autoretransmission retries
         * @param count Max autoretransmission retries
         */
        void setAutoRetransmitCount(uint8_t count);

        /**
         * Get current max autoretransmission retries
         * @return Current max autoretransmission retries
         */
        uint8_t getAutoRetransmitCount();


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
         * Enable constant carrier
         */
        void enableConstantCarrier();

        /**
         * Disable constant carrier
         */
        void disableConstantCarrier();

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
         * Set transceiver's datarate
         * @param dataRate Datarate
         */
        void setDataRate(DataRate dataRate);

        /**
         * Get current transceiver's datarate
         * @return Current datarate
         */
        DataRate getDataRate();

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
         * Set input pipe address (RX pipe address)
         * @param pipe RX pipe
         * @param addr Pipe address
         * @param len Address length
         */
        void setRxPipeAddress(uint8_t pipe, uint8_t* addr, size_t len);

        /**
         * Get current input pipe address (RX pipe address)
         * @param pipe RX pipe
         * @param addr Pipe address
         * @param len Address length
         */
        void getRxPipeAddress(uint8_t pipe, uint8_t* addr, size_t len);


        /**
         * Set destination address (TX address)
         * @param addr Address
         * @param len Address length
         */
        void setTxAddress(uint8_t* addr, size_t len);

        /**
         * Get current destination address (TX address)
         * @param addr Current address
         * @param len Adress length
         */
        void getTxAddress(uint8_t* addr, size_t len);


        /**
         * Set input pipe payload length.
         * Max payload length: 32 bytes
         * @param pipe RX pipe
         * @param len Payload length
         */
        void setRxPipePayloadLength(uint8_t pipe, size_t len);

        /**
         * Get current input pipe payload length
         * @param pipe pipe
         * @return Payload length
         */
        uint8_t getRxPipePayloadLength(uint8_t pipe);


        /**
         * Enable input pipe dynamic payloads
         * @param pipe RX pipe
         */
        void enableRxPipeDynamicPayloads(uint8_t pipe);

        /**
         * Disable input pipe dynamic payloads
         * @param pipe RX pipe
         */
        void disableRxPipeDynamicPayloads(uint8_t pipe);

        /**
         * Disable all pipes dynamic payloads
         */
        void disableDynamicPayloads();


        /**
         * Enable ACK payload
         */
        void enableAckPayload();

        /**
         * Disable ACK payload
         */
        void disableAckPayload();

        /**
         * Enable dynamic ACK
         */
        void enableDynamicAck();

        /**
         * Disable dynamic ACK
         */
        void disableDynamicAck();


        /**
         * @name Command
         */

        /**
         * Read RX payload width for the top Payload in the RX FIFO
         * @note Flush RX FIFO if the read value is larger than 32 bytes
         *
         * @return Payload length
         */
        uint8_t getPayloadLength();

        /**
         * Read RX payload
         * Used in RX mode. Payload is deleted from FIFO after it is read.
         * A read operation always starts at byte 0. Max payload size: 32 bytes.
         *
         * @param data Payload buffer
         * @param len Payload length
         */
        void readPayload(uint8_t* data, size_t len);

        /**
         * Write TX payload
         * A read operation always starts at byte 0 used in TX-payload. Max payload size: 32 bytes.
         *
         * @param data Payload buffer
         * @param len Payload length
         */
        void writePayload(uint8_t* data, size_t len);

        /**
         * Write TX payload to be transmitted together with ACK packet on chosen pipe.
         * Used in RX mode. A write operation always starts at byte 0. Max payload size: 32 bytes.
         *
         * @note Maximum three ACK packet payloads can be pending. Payloads with same # are handled
         *       using first in - first out principle.
         *
         * @param pipe Pipe where payload is written
         * @param data Payload buffer
         * @param len Payload length
         */
        void writePipeAckPayload(uint8_t pipe, uint8_t* data, size_t len);

        /**
         * Write TX payload and disable AUTOACK on this specific packet.
         * Used in TX mode.
         *
         * @param data Payload buffer
         * @param len Payload length
         */
        void writePayloadNoAckPacket(uint8_t* data, size_t len);

        /**
         * Reuse last transmitted payload.
         * Used in TX mode.
         *
         * @note TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed.
         *       TX payload reuse must not be activated or deactivated during package transmission.
         */
        void reuseTxPayload();

        /**
         * Flush TX FIFO.
         * Used in TX mode.
         */
        void flushTxFifo();

        /**
         * Flush RX FIFO.
         * Used in RX mode.
         * @note Should not be executed during transmission of acknowledge, that is,
         *       acknowledge package will not be completed.
         */
        void flushRxFifo();


        /**
         * @name Status
         */

        /**
         * Get count of lost packets.
         * The counter is overflow protected to 15, and discontinues at max until reset.
         * @note The counter is reset by writing to RF_CH.
         *
         * @return Number of packets
         */
        uint8_t getLostPacketsCount();

        /**
         * Get count of retransmitted packets.
         * The counter is reset when transmission of a new packet starts.
         *
         * @return Number of packets
         */
        uint8_t getRetransmittedPacketsCount();

        /**
         * Check if TX payload reuse is active.
         * @note It should be active until W_TX_PAYLOAD or FLUSH TX is executed.
         */
        bool isReuseTxPayloadActive();

        /**
         * Get current TX FIFO status.
         * @return FIFO status
         */
        FifoStatus getTxFifoStatus();

        /**
         * Get current RX FIFO status.
         * @return FIFO status
         */
        FifoStatus getRxFifoStatus();

        /**
         * Get status
         * @param dataReady Data ready status
         * @param dataSent Data sent status
         * @param maxRt Max retansmissions reached status
         */
        void getStatus(bool* dataReady, bool* dataSent, bool* maxRt);

        /**
         * Clear RX_DR, TX_DS and MAX_RT status bits.
         */
        void clearStatus();


        /**
         * @name Driver
         */

        /**
         * Set the PWR_UP bit in the CONFIG register high to enter "Standby-I" mode.
         */
        void powerUp();

        /**
         * Set the PWR_UP bit in the CONFIG register low to enter "Power down" mode.
         */
        void powerDown();

        /**
         * Wake up transmitter.
         * Move to "Standby-I" state
         */
        void begin();

        /**
         * Begin transmission.
         * Move to "RX/TX Mode" state
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
         * @name Interrupt
         */


        /**
         * @name Util
         */

        /**
         * Check if carrier is detected
         * @note nRF24L01(+) must be in RX mode.
         */
        bool isCarrierDetected();

        /**
         * Check if nRF24L01 chip is P variant or not
         * @return Whether is P variant ot not
         */
        bool isPVariant();
    };
}

#endif //NRF24_H
