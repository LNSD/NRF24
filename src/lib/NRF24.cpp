#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>

#include "NRF24.h"

/**
 * MACROS
 */

#ifndef _BV
#define _BV(bit) (1<<(bit))
#endif

/**
 * Constructor and configure subroutines
 */

//region Constructor and configure subroutines

/**
 * Instantiates a new NRF24 class using hardware SPI.
 * @param csn SPI chip select pin (CS/SSEL)
 * @param ce SPI chip enable pin (CE)
 */
NRF24::NRF24(uint8_t csn, uint8_t ce):
        _sck(13),
        _miso(12),
        _mosi(11),
        _csn(csn),
        _ce(ce),
        _irq(0)
{
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);
}

/**
 * Instantiates a new NRF24 class using hardware SPI and IRQ.
 * @param csn SPI chip select pin (CS/SSEL)
 * @param ce SPI chip enable pin (CE)
 * @param irq SPI chip interrupt pin (IRQ)
*/
NRF24::NRF24(uint8_t csn, uint8_t ce, uint8_t irq):
        _sck(13),
        _miso(12),
        _mosi(11),
        _csn(csn),
        _ce(ce),
        _irq(irq)
{
    pinMode(_csn, OUTPUT);
    pinMode(_ce, OUTPUT);

    //TODO Configure interrupt pin to IRQ
}

/**
 * Setups the hardware
 */
void NRF24::configure()
{
    // SPI configure
    SPI.begin();
    SPI.beginTransaction(SPISettings((uint32_t) 1000000, MSBFIRST, SPI_MODE0));

    // Interface configure
    ce(LOW);
    csn(HIGH);
}

//endregion

/**
 *  Low-level signal & SPI-specific functions
 */

//region Low-level signal & SPI-specific functions

/**
 * Low-level CSN signal enable/disable
 * @param level Output signal level
 */
inline void NRF24::csn(uint8_t val)
{
    digitalWrite(_csn, val);
}

/**
 * Low-level CE signal enable/disable
 * @param level Output signal level
 */
inline void NRF24::ce(uint8_t val)
{
    digitalWrite(_ce, val);
}

/**
 * Low-level SPI command wrapper
 * @param cmd SPI command
 */
inline void NRF24::spiCmdTransfer(uint8_t cmd)
{
    csn(LOW);
    SPI.transfer(cmd);
    csn(HIGH);
}

/**
 * Low-level SPI command wrapper
 * @param cmd Preceding SPI command
 * @param buf Array of data to be transferred
 * @param len Array data length
 */
inline void NRF24::spiCmdTransfer(uint8_t cmd, void *buf, size_t len)
{
    csn(LOW);
    SPI.transfer(cmd);
    SPI.transfer(buf, len);
    csn(HIGH);
}

//endregion

/**
 * Register read and write functions
 */

//region Register read and write functions

/**
 * Read nRF24 register
 * @param reg Register to read
 * @return Register content
 */
uint8_t NRF24::readRegister(uint8_t reg)
{
    uint8_t data;

    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), &data, 1);

    return data;
}

/**
 * Read nRF24 multi-byte register
 * @param reg Register to read
 * @param buf Array of data received
 * @param len Array length
*/
void NRF24::readRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiCmdTransfer(R_REGISTER | (REGISTER_MASK & reg), buf, len);
}

/**
 * Write nRF24 register
 * @param reg Register to write
 * @param value Data to write
 */
void NRF24::writeRegister(uint8_t reg, uint8_t value)
{
    //uint8_t data = value; TODO check if this is necessary

    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), &value, 1);
}

/**
 * Write nRF24 multi-byte register
 * @param reg Register to write
 * @param buf Array of data to write
 * @param len Array length
 */
void NRF24::writeRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spiCmdTransfer(W_REGISTER | (REGISTER_MASK & reg), buf, len);
}

//endregion

/**
 * Configuration functions. Getters and setters
 */

//region Configuration functions. Getters and setters

/**
 * Set transceiver mode
 * @param mode Transceiver mode
 */
void NRF24::setTransceiverMode(TransceiverMode_t mode)
{
    uint8_t config = readRegister(CONFIG);

    if(mode == Mode_PTX)
    {
        config &= ~_BV(PRIM_RX);
    }
    else
    {
        config |= _BV(PRIM_RX);
    }

    writeRegister(CONFIG, config);
}

/**
 * Get current transceiver mode
 * @return Transceiver mode
 */
NRF24::TransceiverMode_t NRF24::getTransceiverMode()
{
    uint8_t result = readRegister(CONFIG) & _BV(PRIM_RX);

    if(result == Mode_PTX)
    {
        return Mode_PTX;
    }
    else
    {
        return Mode_PRX;
    }
}

/**
 * Enable constant carrier
 */
void NRF24::enableConstantCarrier()
{
    uint8_t rfsetup= readRegister(RF_SETUP);
    rfsetup |= _BV(CONT_WAVE);

    writeRegister(RF_SETUP, rfsetup);
}

/**
 * Disable constant carrier
 */
void NRF24::disableConstantCarrier()
{
    uint8_t rfsetup = readRegister(RF_SETUP);
    rfsetup &= ~_BV(CONT_WAVE);

    writeRegister(RF_SETUP, rfsetup);
}

/**
 * Check if constant carrier is enabled
 */
bool NRF24::isConstantCarrierEnabled()
{
    return (readRegister(RF_SETUP) & _BV(CONT_WAVE)) > 0;
}

/**
 * Set transceiver's output power level
 * @param level Output power level
 */
void NRF24::setOutputRfPower(OutputPower_t level)
{
    uint8_t setup = readRegister(RF_SETUP) ;
    setup &= ~(_BV(RF_PWR) | _BV(RF_PWR+1));

    switch(level)
    {
        case OutputPower_0dBm:
            setup |= (_BV(RF_PWR) | _BV(RF_PWR+1));
            break;
        case OutputPower_M6dBm:
            setup |= _BV(RF_PWR+1);
            break;
        case OutputPower_M12dBm:
            setup |= _BV(RF_PWR);
            break;
        case OutputPower_M18dBm:
            break;
    }

    writeRegister(RF_SETUP, setup);
}

/**
 * Get current transceiver's output power level
 * @return Current output power
 */
NRF24::OutputPower_t NRF24::getOutputRfPower()
{
    OutputPower_t result = OutputPower_0dBm;
    uint8_t power = readRegister(RF_SETUP) & (_BV(RF_PWR) | _BV(RF_PWR+1));

    if (power == (_BV(RF_PWR) | _BV(RF_PWR+1)))
    {
        result = OutputPower_0dBm ;
    }
    else if(power == _BV(RF_PWR+1))
    {
        result = OutputPower_M6dBm ;
    }
    else if(power == _BV(RF_PWR))
    {
        result = OutputPower_M12dBm ;
    }
    else
    {
        result = OutputPower_M18dBm ;
    }

    return result;
}

/**
 * Set transceiver's datarate
 * @param rate Datarate
 */
void NRF24::setDataRate(DataRate_t rate)
{
    uint8_t setup = readRegister(RF_SETUP);
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    switch(rate)
    {
        case DataRate_250kbps:
            setup |= _BV(RF_DR_LOW);
            break;
        case DataRate_2Mbps:
            setup |= _BV(RF_DR_HIGH);
            break;
        case DataRate_1Mbps:
            break;
    }

    writeRegister(RF_SETUP, setup);
}

/**
 * Get current transceiver's datarate
 * @return Current datarate
 */
NRF24::DataRate_t NRF24::getDataRate()
{
    uint8_t dr = readRegister(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    if(dr == _BV(RF_DR_LOW))
    {
        // '10' = 250Kbps
        return DataRate_250kbps;
    }
    else if(dr == _BV(RF_DR_HIGH))
    {
        // '01' = 2Mbps
        return DataRate_2Mbps;
    }
    else if (dr == 0)
    {
        // '00' = 1Mbps
        return DataRate_1Mbps;
    }

    return DataRate_2Mbps;
}

/**
 * Set transceiver's RF channel
 * @param channel RF channel
 */
void NRF24::setRfChannel(uint8_t channel)
{
    const uint8_t max_channel = 127;
    writeRegister(RF_CH, min(channel, max_channel));
}

/**
 * Get transceiver's current RF channel
 * @return Current RF channel
 */
uint8_t NRF24::getRfChannel()
{
    return readRegister(RF_CH);
}
/**
 * Set address length. Address length: 3 - 5 bytes.
 * @param length Address length
 */
void NRF24::setAddrLength(uint8_t length)
{
    if(length >= 3  && length <= 5)
    {
        writeRegister(SETUP_AW, length-2);
    }
}

/**
 * Get current address length
 * @return Current address length
 */
uint8_t NRF24::getAddrLength()
{
    return readRegister(SETUP_AW) + 2;
}

/**
 * Enable RX pipe
 * @param pipe RX pipe
 */
void NRF24::enablePipeRxAddr(RxPipe_t pipe)
{
    writeRegister(EN_RXADDR, readRegister(EN_RXADDR) | _BV(pipe));
}

/**
 * Disable RX pipe
 * @param pipe RX pipe
 */
void NRF24::disablePipeRxAddr(RxPipe_t pipe)
{
    writeRegister(EN_RXADDR, readRegister(EN_RXADDR) & ~_BV(pipe));
}

/**
 * Get list of RX pipes status
 * @param pipes Boolean array holding status of all the RX pipes
 */
void NRF24::whichRxAddrAreEnabled(bool *pipes)
{
    uint8_t enRxAddr = readRegister(EN_RXADDR);
    for (int p = 0; p < 6; ++p)
    {
        pipes[p] = (bool) (enRxAddr & _BV(p));
    }
}

/**
 * Set destination address (TX address)
 * @param addr Address
 * @param len Address length
 */
void NRF24::setTxAddr(uint8_t *addr, uint8_t len)
{
    writeRegister(TX_ADDR, addr, len);
}

/**
 * Get current destination address (TX address)
 * @param addr Current address
 * @param len Adress length
 */
void NRF24::getTxAddr(uint8_t *addr, uint8_t len)
{
    readRegister(TX_ADDR, addr, len);
}

const uint8_t pipeRx[6] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

/**
 * Set input pipe address (RX pipe address)
 * @param pipe RX pipe
 * @param addr Pipe address
 * @param len Address length
 */
void NRF24::setPipeRxAddr(RxPipe_t pipe, uint8_t *addr, uint8_t len)
{
    if(pipe < 2)
    {
        writeRegister(pipeRx[pipe], addr, len);
    }
    else
    {
        writeRegister(pipeRx[pipe], addr[0]);
    }
}

/**
 * Get current input pipe address (RX pipe address)
 * @param pipe RX pipe
 * @param addr Pipe address
 * @param len Address length
 */
void NRF24::getPipeRxAddr(RxPipe_t pipe, uint8_t *addr, uint8_t len)
{
    if(pipe < 2)
    {
        readRegister(pipeRx[pipe], addr, len);
    }
    else
    {
        addr[0] = readRegister(pipeRx[pipe]);
    }
}

const uint8_t pipe_payload[6] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};

/**
 * Set input pipe payload size. Max payload size: 32 bytes
 * @param pipe RX pipe
 * @param size Payload size
 */
void NRF24::setPipePayloadSize(RxPipe_t pipe, uint8_t size)
{
    const uint8_t max_size = 32;
    writeRegister(pipe_payload[pipe], min(size, max_size));
}

/**
 * Get current input pipe payload size
 * @param pipe pipe
 * @return Payload size
 */
uint8_t NRF24::getPipePayloadSize(RxPipe_t pipe)
{
    return readRegister(pipe_payload[pipe]);
}

/**
 * Enable input pipe dynamic payloads
 * @param pipe RX pipe
 */
void NRF24::enablePipeDynamicPayloads(RxPipe_t pipe)
{
    uint8_t dynpd = readRegister(DYNPD);
    dynpd |= _BV(pipe);

    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_DPL));
    writeRegister(DYNPD, dynpd);
}

/**
 * Disable input pipe dynamic payloads
 * @param pipe RX pipe
 */
void NRF24::disablePipeDynamicPayloads(RxPipe_t pipe)
{
    uint8_t dynpd = readRegister(DYNPD);

    if(!(dynpd & ~_BV(pipe))) {
        writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_DPL));
    }

    writeRegister(DYNPD, dynpd & ~_BV(pipe));
}

/**
 * Check which inout pipe dynamic payloads are enabled
 * @param dynamicPayloads RX pipe dynamic payloads status
 */
void NRF24::whichPipeDynamicPayloadsAreEnabled(bool *dynamicPayloads)
{
    if(readRegister(FEATURE) & _BV(EN_DPL))
    {
        uint8_t dPayloads = readRegister(DYNPD);
        for (int p = 0; p < 6; ++p)
        {
            bool dynp = (bool) (dPayloads & _BV(p));
            dynamicPayloads[p] = dynp;
        }
    }
    else
    {
        const bool defaultDynPayloads[6] = {false,false,false,false,false,false};
        memcpy(dynamicPayloads, defaultDynPayloads, sizeof(defaultDynPayloads));
    }
}

/**
 * Enable CRC and set length
 * @param length CRC length
 */
void NRF24::enableCRC(CRCLength_t length)
{
    uint8_t config = readRegister(CONFIG);
    config |= _BV(EN_CRC);

    if(length == CRC_16)
    {
        config |= _BV(CRCO);
    }
    else
    {
        config &= ~_BV(CRCO);
    }

    writeRegister(CONFIG, config);
}

/**
 * Disable CRC
 */
void NRF24::disableCRC()
{
    uint8_t config = readRegister(CONFIG);
    config &= ~_BV(EN_CRC);
    writeRegister(CONFIG, config);
}

/**
 * Get current CRC configuration
 * @return Current CRC configuration
 */
NRF24::CRCLength_t NRF24::getCRCConfig()
{
    uint8_t config = readRegister(CONFIG);
    if(config & _BV(EN_CRC))
    {
        if(config & _BV(CRCO))
        {
            return CRC_16;
        }
        else
        {
            return CRC_8;
        }
    }
    else
    {
        return CRC_DISABLED;
    }
}

/**
 * Enable input pipe auto ACK
 * @param pipe RX pipe
 */
void NRF24::enablePipeAutoAck(RxPipe_t pipe)
{
    writeRegister(EN_AA, readRegister(EN_AA) | _BV(pipe));
}

/**
 * Disable input pipe auto ACK
 * @param pipe RX pipe
 */
void NRF24::disablePipeAutoAck(RxPipe_t pipe)
{
    writeRegister(EN_AA, readRegister(EN_AA) & ~_BV(pipe));
}

/**
 * Get which input pipes have auto ACK enabled
 * @param autoAck Boolean array holding status of RX pipes auto ACK
 */
void NRF24::whichPipeAutoAckAreEnabled(bool *autoAck)
{
    uint8_t autoack = readRegister(EN_AA);
    for (int p = 0; p < 6; ++p)
    {
        autoAck[p] = (bool) (autoack & _BV(p));
    }
}

/**
 * Set autoretransmission delay
 * @param delay Autoretransmission delay
 */
void NRF24::setAutoRtDelay(uint16_t delay)
{
    const uint8_t max_delay = 0xF;
    uint8_t setupRetr = readRegister(SETUP_RETR);
    setupRetr &= 0x0F;
    setupRetr |= (min(delay/250, max_delay) << 4);

    writeRegister(SETUP_RETR, setupRetr);
}

/**
 * Get current autoretransmission delay
 * @return Current autoretranmission delay
 */
uint8_t NRF24::getAutoRtDelay()
{
    return (readRegister(SETUP_RETR) >> 4);
}

/**
 * Set max autoretransmission retries
 * @param count Max autoretransmission retries
 */
void NRF24::setAutoRtCount(uint8_t count)
{
    const uint8_t max_count = 0xF;
    uint8_t setupRetr = readRegister(SETUP_RETR);
    setupRetr &= 0xF0;
    setupRetr |= min(count, max_count);

    writeRegister(SETUP_RETR, setupRetr);
}

/**
 * Get current max autoretransmission retries
 * @return Current max autoretransmission retries
 */
uint8_t NRF24::getAutoRtCount()
{
    return readRegister(SETUP_RETR) & 0x0F;
}

/**
 * Enable ACK payload
 */
void NRF24::enableAckPayload()
{
    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_ACK_PAY));
}

/**
 * Disable ACK payload
 */
void NRF24::disableAckPayload()
{
    writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_ACK_PAY));
}

/**
 * Get current ACK payload configuration
 */
bool NRF24::getAckPayloadConfig()
{
    return  (bool)(readRegister(FEATURE) & _BV(EN_ACK_PAY));
}

/**
 * Enable dynamic ACK
 */
void NRF24::enableDynamicAck()
{
    writeRegister(FEATURE, readRegister(FEATURE) | _BV(EN_DYN_ACK));
}

/**
 * Disabel dynamic ACK
 */
void NRF24::disableDynamicAck()
{
    writeRegister(FEATURE, readRegister(FEATURE) & ~_BV(EN_DYN_ACK));
}

/**
 * Get current dynamic ACK configuration
 */
bool NRF24::getDynamicAckConfig()
{
    return (bool)(readRegister(FEATURE) & _BV(EN_DYN_ACK));
}

//endregion

/**
 * Command functions
 */

//region Command functions

/**
 * Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
 * @Note Flush RX FIFO if the read value is larger than 32 bytes
 * @return RX payload width
 */
uint8_t NRF24::getRxPayloadLength()
{
    uint8_t width;

    spiCmdTransfer(R_RX_PL_WID, &width, 1);

    return width;
}

/**
 * Read RX-payload: 1 - 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
 * @param data Payload buffer
 * @param len Payload length
 */
void NRF24::readRxPayload(uint8_t* data, uint8_t len)
{
    spiCmdTransfer(R_RX_PAYLOAD, data, len);
}

/**
 * Write TX-payload: 1 - 32 bytes. A read operation always starts at byte 0 used in TX payload.
 * @param data Payload buffer
 * @param len Payload length
 */
void NRF24::writeTxPayload(uint8_t* data, uint8_t len)
{
    spiCmdTransfer(W_TX_PAYLOAD, data, len);
}

/**
 * Used in RX mode. Write Payload to be transmitted together with ACK packet on chosen pipe.
 * @note Maximum three ACK packet payloads can be pending. Payloads with same # are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0.
 * @param pipe Pipe where payload is written
 * @param data Payload buffer
 * @param len Payload length
 */
void NRF24::writePipeACKPayload(RxPipe_t pipe, uint8_t* data, uint8_t len)
{
    spiCmdTransfer((uint8_t) (W_ACK_PAYLOAD | (pipe & 0x07)), data, len);
}

/**
 * Used in TX mode. Disables AUTOACK on this specific packet.
 */
void NRF24::disableAAforPayload()
{
    spiCmdTransfer(W_TX_PAYLOAD_NOACK);
}

/**
 * Used for a PTX device. Reuse last transmitted payload.
 * @note TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deacti- vated during package transmission.
 */
void NRF24::reuseTxPayload()
{
    spiCmdTransfer(REUSE_TX_PL);
}

/**
 * Flush TX FIFO, used in TX mode.
 */
void NRF24::flushTXFIFO()
{
    spiCmdTransfer(FLUSH_TX);
}

/**
 * Flush RX FIFO, used in RX mode.
 * @note Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
 */
void NRF24::flushRXFIFO()
{
    spiCmdTransfer(FLUSH_RX);
}

//endregion

/**
 * Util functions
 */

//region Util functions

//endregion