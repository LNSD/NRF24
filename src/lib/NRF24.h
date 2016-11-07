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
     * @name Driver Constants
     */

    const static uint8_t MAX_PAYLOAD_SIZE = 32;
    const static uint8_t MAX_RF_CHANNEL = 127;

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
        OutputPower_M12dBm,		// -12dBm LOW
        OutputPower_M6dBm,		// -6dBm HIGH
        OutputPower_0dBm,		// 	0dBm MAX
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
     * NRF24 configuration holder class
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
        Configuration() {};

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this configuration holder.
         *
         * @param mode Transceiver mode
         */
        Configuration(TransceiverMode mode):
                _mode(mode)
        {};

        /**
         * Arduino Constructor
         *
         * Creates a new instance of this configuration holder.
         *
         * @param mode Transceiver mode
         * @param level Output power level
         * @param rate Communication data rate
         */
        Configuration(TransceiverMode mode, OutputPower level, DataRate rate):
                _mode(mode),
                _outputPower(level),
                _dataRate(rate)
        {};

        /**
         * @name Configuration setters
         */

        /**
         * Set transceiver mode
         * @param mode Transceiver mode
         */
        void setTransceiverMode(TransceiverMode mode) {
            _mode = mode;
        }

        /**
         * Set output power
         * @param power Output power
         */
        void setOutputPower(OutputPower power) {
            _outputPower = power;
        }

        /**
         * Set communication data rate
         * @param dataRate Communication data rate
         */
        void setDataRate(DataRate dataRate) {
            _dataRate = dataRate;
        }

        /**
         * Set RF channel
         * @param channel RF channel
         */
        void setRFChannel(uint8_t channel) {
            _rfCh = channel;
        }

        /**
         * Enable constant carrier
         */
        void enableConstantCarrier() {
            _constCarrier = true;
        }

        /**
         * Disable constant carrier
         */
        void disableConstantCarrier() {
            _constCarrier = false;
        }

        /**
         * Force PLL lock
         */
        void forcePllLock() {
            _pllLock = true;
        }

        /**
         * Disable PLL lock
         */
        void disablePllLock() {
            _pllLock = false;
        }

        /**
         * Set CRC length
         * @param crc Length
         */
        void setCRC(CRCLength crc) {
            _crc = crc;
        }

        /**
         * Set address width
         * @param width Address width
         */
        void setAddressWidth(uint8_t width) {
            _addressWidth = width;
        }

        /**
         * Enable Rx Pipe
         * @param pipe Rx Pipe
         */
        void enableRxPipeAddress(RxPipe pipe) {
            _rxPipeAddressStatus[pipe] = true;
        }

        /**
         * Disable Rx Pipe
         * @param pipe Rx Pipe
         */
        void disableRxPipeAddress(RxPipe pipe) {
            _rxPipeAddressStatus[pipe] = false;
            _rxPipePayloadSize[pipe] = 0;
        }

        /**
         * Disable all Rx Pipes
         */
        void disableAllRxPipeAddresses() {
            memset(_rxPipeAddressStatus, false, sizeof(_rxPipeAddressStatus));
            memset(_rxPipePayloadSize, 0, sizeof(_rxPipePayloadSize));
        }

        /**
         * Set Rx Pipe payload size
         * @param pipe Rx Pipe
         * @param size Payload size
         */
        void setRxPipePayloadSize(NRF24::RxPipe pipe, uint8_t size) {
            _rxPipePayloadSize[pipe] = min(size, NRF24::MAX_PAYLOAD_SIZE);
        }

        /**
         * Set Rx Pipe address
         * @param pipe Rx Pipe
         * @param address Address bytes
         */
        void setRxPipeAddress(NRF24::RxPipe pipe, uint8_t* address) {
            if (pipe < 2) {
                memcpy(_rxPipeAddrLong[pipe], address, _addressWidth);
            } else {
                _rxPipeAddrShort[pipe] = address[0];
            }
        }

        /**
         * Set Tx address
         * @param address Address bytes
         */
        void setTxAddress(uint8_t *address) {
            memcpy(_txAddr, address, _addressWidth);
        }

        /**
         * Enable auto ACK
         */
        void enableAutoAck() {
            _autoAck = true;
        }

        /**
         * Disable auto ACK
         */
        void disableAutoAck() {
            _autoAck = false;
        }

        /**
         * Set auto retransmissions delay
         * @param delay Auto retransmission delay
         */
        void setAutoRtDelay(uint16_t delay) {
            _autoRtDelay = delay;
        }

        /**
         * Set MAX retransmissions count
         * @param count Retransmission count
         */
        void setAutoRtCount(uint8_t count) {
            _autoRtCount = count;
        }

        /**
         * Enable Rx Pipe dynamic payloads
         * @param pipe Rx Pipe
         */
        void enableRxPipeDynamicPayload(NRF24::RxPipe pipe) {
            _dynamicPayload[pipe] = true;
        }

        /**
         * Disable Rx Pipe dynamic payloads
         * @param pipe
         */
        void disableRxPipeDynamicPayload(NRF24::RxPipe pipe) {
            _dynamicPayload[pipe] = false;
        }

        /**
         * Disable all Rx Pipes dynamic payloads
         */
        void disableDynamicPayloads() {
            memset(_dynamicPayload, false, sizeof(_dynamicPayload));
        }

        /**
         * Enable ACK payload
         */
        void enableAckPayload() {
            _ackPayload = true;
        }

        /**
         * Disable ACK payloads
         */
        void disableAckPayload() {
            _ackPayload = false;
        }

        /**
         * Enable dynamic ACKs
         */
        void enableDynamicAck() {
            _dynamicAck = true;
        }

        /**
         * Disable dynamic ACKs
         */
        void disableDynamicAck() {
            _dynamicAck = false;
        }

    private:
        NRF24::TransceiverMode _mode = NRF24::Mode_PTX;
        NRF24::OutputPower _outputPower = NRF24::OutputPower_0dBm;
        NRF24::DataRate _dataRate = NRF24::DataRate_1Mbps;

        uint8_t _rfCh = 2;
        bool _constCarrier = false;
        bool _pllLock = false;
        NRF24::CRCLength _crc = NRF24::CRC_16;
        uint8_t _addressWidth = 5;
        bool _rxPipeAddressStatus[6] = {false, false, false, false, false, false};
        uint8_t _rxPipePayloadSize[6] = { 0, 0, 0, 0, 0, 0 };
        bool _autoAck = true;
        uint16_t _autoRtDelay = 1500;
        uint8_t _autoRtCount = 15;
        bool _dynamicPayload[6] = {false, false, false, false, false, false};
        bool _ackPayload = false;
        bool _dynamicAck = false;

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
        void setDataRate(DataRate speed);

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
        void setAddressWidth(uint8_t width);

        /**
         * Get current address width
         * @return Current address width
         */
        uint8_t getAddressWidth();

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
        void whichRxPipeAddrAreEnabled(bool *enabledAddr);

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
         * @param dynamicPayloads RX pipe dynamic payloads status
         */
        void whichRxPipeDynamicPayloadsAreEnabled(bool *dynamicPayloads);

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
         * @param autoAck Boolean array holding status of RX pipes auto ACK
         */
        void whichRxPipeAutoAckAreEnabled(bool *autoAck);

        /**
         * Set autoretransmission delay
         * @param delay Autoretransmission delay
         */
        void setAutoRtDelay(uint16_t delay);

        /**
         * Get current autoretransmission delay
         * @return Current autoretranmission delay
         */
        uint8_t getAutoRtDelay();

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
         * Get current communication status
         * @param status
         */
        void getCommStatus(bool* status);

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
}

#endif //NRF24_H
