#ifndef NRF24_H
#define NRF24_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "NRF24L01.h"

/**
 * Data rate. How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum {
    NRF24_DataRate_1Mbps = 0,	// 1Mbps
    NRF24_DataRate_2Mbps,		// 2Mbps
    NRF24_DataRate_250kbps		// 250kbps
} NRF24_DataRate_t;

/**
 * Power Amplifier level.
 *
 * For use with +()
 */
typedef enum {
    NRF24_OutputPower_M18dBm = 0,	// -18dBm MIN
    NRF24_OutputPower_M12dBm,		// -12dBm LOW
    NRF24_OutputPower_M6dBm,		// -6dBm HIGH
    NRF24_OutputPower_0dBm,			// 	0dBm MAX
} NRF24_OutputPower_t;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum {
    NRF24_CRC_8 = 0,
    NRF24_CRC_16,
    NRF24_CRC_DISABLED
} NRF24_CRCLength_t;


/**
 * Interrupt configuration.
 *
 */
typedef enum {
    NRF24_IRQ_RX_DR = 0,
    NRF24_IRQ_TX_DS,
    NRF24_IRQ_MAX_RT
} NRF24_Interrupt_t;

/**
 * Pipes definition.
 *
 */
typedef enum {
    NRF24_RX_P0 = 0,
    NRF24_RX_P1 = 1,
    NRF24_RX_P2 = 2,
    NRF24_RX_P3 = 3,
    NRF24_RX_P4 = 4,
    NRF24_RX_P5 = 5
} NRF24_RxPipe_t;

/**
 * Transmision mode definition.
 *
 */
typedef enum {
    NRF24_Mode_PTX = 0,
    NRF24_Mode_PRX = 1
} NRF24_TransceiverMode_t;

/**
 * Fifo status.
 *
 */
typedef enum {
    NRF24_FIFO_EMPTY = 0,
    NRF24_FIFO_OK,
    NRF24_FIFO_FULL
} NRF24_FIFOStatus_t;


class NRF24
{
public:
    // Constructor and configure subroutines
    NRF24(uint8_t csn, uint8_t ce);              // Hardware SPI
	NRF24(uint8_t csn, uint8_t c, uint8_t irq);  // Hardware SPI + IRQ
	void configure();

    // Register read and write functions
    uint8_t readRegister(uint8_t reg);
    void readRegister(uint8_t reg, uint8_t *buf, uint8_t len);
    void writeRegister(uint8_t reg, uint8_t value);
    void writeRegister(uint8_t reg, uint8_t *buf, uint8_t len);

    // Command functions
    uint8_t getRxPayloadLength();
    void readRxPayload(uint8_t* data, uint8_t len);
    void writeTxPayload(uint8_t* data, uint8_t len);
    void writePipeACKPayload(NRF24_RxPipe_t pipe, uint8_t* data, uint8_t len);
    void disableAAforPayload();
    void reuseTxPayload();
    void flushTXFIFO();
    void flushRXFIFO();

    // Configuration functions. Getters and setters
    void setTransceiverMode(NRF24_TransceiverMode_t mode);
    NRF24_TransceiverMode_t getTransceiverMode();
    void enableConstantCarrier();
    void disableConstantCarrier();
    bool isConstantCarrierEnabled();
    void setOutputRfPower(NRF24_OutputPower_t level);
    NRF24_OutputPower_t getOutputRfPower();
    void setDataRate(NRF24_DataRate_t speed);
    NRF24_DataRate_t getDataRate();
    void setRFChannel(uint8_t channel);
    uint8_t getRFChannel();
    void setAddrLength(uint8_t length);
    uint8_t getAddrLength();
    void enablePipeRxAddr(NRF24_RxPipe_t pipe);
    void disablePipeRxAddr(NRF24_RxPipe_t pipe);
    void whichRxAddrAreEnabled(bool *addr_enabled);
    void setTxAddr(uint8_t* addr, uint8_t len);
    void getTxAddr(uint8_t* addr, uint8_t len);
    void setPipeRxAddr(NRF24_RxPipe_t pipe, uint8_t* addr, uint8_t len);
    void getPipeRxAddr(NRF24_RxPipe_t pipe, uint8_t* addr, uint8_t len);
    void setPipePayloadSize(NRF24_RxPipe_t pipe, uint8_t size);
    uint8_t getPipePayloadSize(NRF24_RxPipe_t pipe);
    void enablePipeDynamicPayloads(NRF24_RxPipe_t pipe);
    void disablePipeDynamicPayloads(NRF24_RxPipe_t pipe);
    void whichPipeDynamicPayloadsAreEnabled(bool *dynamicPayloads);
    void enableCRC(NRF24_CRCLength_t length);
    void disableCRC();
    NRF24_CRCLength_t getCRCConfig();
    void enablePipeAutoAck(NRF24_RxPipe_t pipe);
    void disablePipeAutoAck(NRF24_RxPipe_t pipe);
    void whichPipeAutoAckAreEnabled(bool *autoAck);
    void setAutoRtDelay(uint8_t delay);
    uint8_t getAutoRtDelay();
    void setAutoRtCount(uint8_t count);
    uint8_t getAutoRtCount();
    void enableAckPayload();
    void disableAckPayload();
    bool getAckPayloadConfig();
    void enableDynamicAck();
    void disableDynamicAck();
    bool getDynamicAckConfig();

private:
	uint8_t _sck, _mosi, _miso, _csn;
	uint8_t _ce, _irq;

	// Low-level signal & SPI-specific functions
	void csn(uint8_t val);
	void ce(uint8_t val);
    void spiCmdTransfer(uint8_t cmd);
    void spiCmdTransfer(uint8_t cmd, void *buf, size_t len);
};

#endif
