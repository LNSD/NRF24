#ifndef NRF24_H
#define NRF24_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "NRF24L01.h"

/**
 * Interrupt configuration.
 */
typedef enum {
    NRF24_IRQ_RX_DR = 0,
    NRF24_IRQ_TX_DS,
    NRF24_IRQ_MAX_RT
} NRF24_Interrupt_t;

/**
 * Main NRF24 class
 */
class NRF24
{
public:

    const static uint8_t MAX_PAYLOAD_SIZE = 32;

    //region Enum typedefs
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
    } FIFOStatus;
    //endregion

    /**
     * NRF24 initial configuration holder class
     */
    class Config
    {
    public:
        Config(NRF24::TransceiverMode mode):
                _mode(mode)
        {};

        Config(NRF24::TransceiverMode mode, NRF24::OutputPower level, NRF24::DataRate rate):
                _mode(mode),
                _power(level),
                _dataRate(rate)
        {};

        void setTransceiverMode(NRF24::TransceiverMode mode) {
            _mode = mode;
        }

        void setPower(NRF24::OutputPower power) {
            _power = power;
        }

        void setDataRate(NRF24::DataRate dataRate) {
            _dataRate = dataRate;
        }

        void setRfChannel(uint8_t channel) {
            _rfCh = channel;
        }

        void enableConstCarrier() {
            _constCarrier = true;
        }

        void disableConstCarrier() {
            _constCarrier = false;
        }

        void forcePllLock() {
            _pllLock = true;
        }

        void disablePllLock() {
            _pllLock = false;
        }

        void setCRC(NRF24::CRCLength crc) {
            _crc = crc;
        }

        void setAddrWidth(uint8_t width) {
            _addrWidth = width;
        }

        void enableRxPipeAddress(NRF24::RxPipe pipe) {
            _rxPipeAddrStatus[pipe] = true;
        }

        void disableRxPipeAddress(NRF24::RxPipe pipe) {
            _rxPipeAddrStatus[pipe] = false;
            _rxPipePayloadSize[pipe] = 0;
        }

        void disableAllRxPipeAddress() {
            memset(_rxPipeAddrStatus, false, sizeof(_rxPipeAddrStatus));
            memset(_rxPipePayloadSize, 0, sizeof(_rxPipePayloadSize));
        }

        void setRxPipePayloadSize(NRF24::RxPipe pipe, uint8_t size) {
            _rxPipePayloadSize[pipe] = min(size, NRF24::MAX_PAYLOAD_SIZE);
        }

        void setRxPipeAddress(NRF24::RxPipe pipe, uint8_t* address) {
            if (pipe < 2) {
                memcpy(_rxPipeAddrLong[pipe], address, _addrWidth);
            } else {
                _rxPipeAddrShort[pipe] = address[0];
            }
        }

        void setTxAddress(uint8_t *address) {
            memcpy(_txAddr, address, _addrWidth);
        }

        void enableAutoAck() {
            _autoAck = true;
        }

        void disableAutoAck() {
            _autoAck = false;
        }

        void setAutoRtDelay(uint16_t autoRtDelay) {
            _autoRtDelay = autoRtDelay;
        }

        void setAutoRtCount(uint8_t autoRtCount) {
            _autoRtCount = autoRtCount;
        }

        void enableRxPipeDynamicPayload(NRF24::RxPipe pipe) {
            _dynamicPayload[pipe] = true;
        }

        void disableRxPipeDynamicPayload(NRF24::RxPipe pipe) {
            _dynamicPayload[pipe] = false;
        }

        void disableDynamicPayloads() {
            memset(_dynamicPayload, false, sizeof(_dynamicPayload));
        }

        void enableAckPayload() {
            _ackPayload = true;
        }

        void disableAckPayload() {
            _ackPayload = false;
        }

        void enableDynamicAck() {
            _dynamicAck = true;
        }

        void disableDynamicAck() {
            _dynamicAck = false;
        }

    private:
        NRF24::TransceiverMode _mode = NRF24::Mode_PTX;
        NRF24::OutputPower _power = NRF24::OutputPower_0dBm;
        NRF24::DataRate _dataRate = NRF24::DataRate_2Mbps;

        uint8_t _rfCh = 2;
        bool _constCarrier = false;
        bool _pllLock = false;
        NRF24::CRCLength _crc = NRF24::CRC_8;

        uint8_t _addrWidth = 5;

        bool _rxPipeAddrStatus[6] = {false, false, false, false, false, false};

        uint8_t _rxPipePayloadSize[6] = { 0, 0, 0, 0, 0, 0 };

        uint8_t _rxPipeAddrLong[2][5] = {{ 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 },
                                         { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 }};

        uint8_t _rxPipeAddrShort[4] = { 0xC3, 0xC4, 0xC5, 0xC6 };

        uint8_t _txAddr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

        bool _autoAck = true;
        uint16_t _autoRtDelay = 250;
        uint8_t _autoRtCount = 3;


        bool _dynamicPayload[6] = {false, false, false, false, false, false};

        bool _ackPayload = false;
        bool _dynamicAck = false;

        friend class NRF24;
    };

    // Constructor and configure subroutines
    NRF24(uint8_t csn, uint8_t ce);              // Hardware SPI
    NRF24(uint8_t csn, uint8_t c, uint8_t irq);  // Hardware SPI + IRQ
    void configure();
    void configure(Config configuration);

    // Register read and write functions
    uint8_t readRegister(uint8_t reg);
    void readRegister(uint8_t reg, uint8_t *buf, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t value);
    void writeRegister(uint8_t reg, uint8_t *buf, uint8_t len);

    //region Configuration functions. Getters and setters

    void setTransceiverMode(TransceiverMode mode);
    TransceiverMode getTransceiverMode();
    void enableConstantCarrier();
    void disableConstantCarrier();
    bool isConstantCarrierEnabled();
    void forcePllLock();
    void disablePllLock();
    bool isPllLockForced();
    void setOutputRfPower(OutputPower level);
    OutputPower getOutputRfPower();
    void setDataRate(DataRate speed);
    DataRate getDataRate();
    void setRfChannel(uint8_t channel);
    uint8_t getRfChannel();
    void setAddrLength(uint8_t length);
    uint8_t getAddrLength();
    void enablePipeRxAddr(RxPipe pipe);
    void disablePipeRxAddr(RxPipe pipe);
    void whichRxAddrAreEnabled(bool *addr_enabled);
    void setTxAddr(uint8_t* addr, uint8_t len);
    void getTxAddr(uint8_t* addr, uint8_t len);
    void setPipeRxAddr(RxPipe pipe, uint8_t* addr, uint8_t len);
    void getPipeRxAddr(RxPipe pipe, uint8_t* addr, uint8_t len);
    void setPipePayloadSize(RxPipe pipe, uint8_t size);
    uint8_t getPipePayloadSize(RxPipe pipe);
    void enableRxPipeDynamicPayloads(RxPipe pipe);
    void disableRxPipeDynamicPayloads(RxPipe pipe);
    void whichPipeDynamicPayloadsAreEnabled(bool *dynamicPayloads);
    void enableCRC(CRCLength length);
    void disableCRC();
    CRCLength getCRCConfig();
    void enableRxPipeAutoAck(RxPipe pipe);
    void disableRxPipeAutoAck(RxPipe pipe);
    void whichPipeAutoAckAreEnabled(bool *autoAck);
    void setAutoRtDelay(uint16_t delay);
    uint8_t getAutoRtDelay();
    void setAutoRtCount(uint8_t count);
    uint8_t getAutoRtCount();
    void enableAckPayload();
    void disableAckPayload();
    bool isAckPayloadEnabled();
    void enableDynamicAck();
    void disableDynamicAck();
    bool isDynamicAckEnabled();
    //endregion

    // Command functions
    uint8_t getRxPayloadLength();
    void readRxPayload(uint8_t* data, uint8_t len);
    void writeTxPayload(uint8_t* data, uint8_t len);
    void writePipeACKPayload(RxPipe pipe, uint8_t* data, uint8_t len);
    void disableAAforPayload();
    void reuseTxPayload();
    void flushTXFIFO();
    void flushRXFIFO();

    // Get status functions
    uint8_t getLostPacketsCount();
    uint8_t getRtCount();
    bool isCarrierDetected();
    bool isReuseTxPayloadActive();
    FIFOStatus getTxFifoStatus();
    FIFOStatus getRxFifoStatus();

    // Interrupt related functions
    void clearCommStatus();
    void getCommStatus(bool* status);

    //Util functions
    bool isPVariant();

private:
    uint8_t _sck, _mosi, _miso, _csn;
    uint8_t _ce, _irq;

    //Low-level signal & SPI-specific functions
    void csn(uint8_t val);
    void ce(uint8_t val);
    void spiCmdTransfer(uint8_t cmd);
    void spiCmdTransfer(uint8_t cmd, void *buf, size_t len);
};

#endif
