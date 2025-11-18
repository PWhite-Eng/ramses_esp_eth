// Original files from evofw3:CC1101.h, CC1101.c, trace.h, config.h. cc1101_param.h, cc1101_param.c, cc1101_const.h, nv.h, nv.c

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include "ESP_CC1101_GPIO_Interface.hpp"

// Constants from cc1101_const.h
#define CC_READ  0x80
#define CC_BURST 0x40

// Configuration Registers
#define CC1100_IOCFG2    0x00
#define CC1100_IOCFG1    0x01
#define CC1100_IOCFG0    0x02
#define CC1100_FIFOTHR   0x03
#define CC1100_SYNC1     0x04
#define CC1100_SYNC0     0x05
#define CC1100_PKTLEN    0x06
#define CC1100_PKTCTRL1  0x07
#define CC1100_PKTCTRL0  0x08
#define CC1100_ADDR      0x09
#define CC1100_CHANNR    0x0A
#define CC1100_FSCTRL1   0x0B
#define CC1100_FSCTRL0   0x0C
#define CC1100_FREQ2     0x0D
#define CC1100_FREQ1     0x0E
#define CC1100_FREQ0     0x0F
#define CC1100_MDMCFG4   0x10
#define CC1100_MDMCFG3   0x11
#define CC1100_MDMCFG2   0x12
#define CC1100_MDMCFG1   0x13
#define CC1100_MDMCFG0   0x14
#define CC1100_DEVIATN   0x15
#define CC1100_MCSM2     0x16
#define CC1100_MCSM1     0x17
#define CC1100_MCSM0     0x18
#define CC1100_FOCCFG    0x19
#define CC1100_BSCFG     0x1A
#define CC1100_AGCCTRL2  0x1B
#define CC1100_AGCCTRL1  0x1C
#define CC1100_AGCCTRL0  0x1D
#define CC1100_WOREVT1   0x1E
#define CC1100_WOREVT0   0x1F
#define CC1100_WORCTRL   0x20
#define CC1100_FREND1    0x21
#define CC1100_FREND0    0x22
#define CC1100_FSCAL3    0x23
#define CC1100_FSCAL2    0x24
#define CC1100_FSCAL1    0x25
#define CC1100_FSCAL0    0x26
#define CC1100_RCCTRL1   0x27
#define CC1100_RCCTRL0   0x28
#define CC1100_FSTEST    0x29
#define CC1100_PTEST     0x2A
#define CC1100_AGCTEST   0x2B
#define CC1100_TEST2     0x2C
#define CC1100_TEST1     0x2D
#define CC1100_TEST0     0x2E
#define CC1100_PARAM_MAX 0x2F

// Strobe Commands
#define CC1100_SRES      0x30
#define CC1100_SFSTXON   0x31
#define CC1100_SXOFF     0x32
#define CC1100_SCAL      0x33
#define CC1100_SRX       0x34
#define CC1100_STX       0x35
#define CC1100_SIDLE     0x36
#define CC1100_SWOR      0x38
#define CC1100_SPWD      0x39
#define CC1100_SFRX      0x3A
#define CC1100_SFTX      0x3B
#define CC1100_SNOP      0x3D
#define CC1100_PATABLE   0x3E
#define CC1100_FIFO      0x3F

#define CC1100_PA_MAX      8

// Status Registers
#define CC1100_PARTNUM        ( 0x30 | CC_BURST )
#define CC1100_VERSION        ( 0x31 | CC_BURST )
#define CC1100_FREQEST        ( 0x32 | CC_BURST )
#define CC1100_LQI            ( 0x33 | CC_BURST )
#define CC1100_RSSI           ( 0x34 | CC_BURST )
#define CC1100_MARCSTATE      ( 0x35 | CC_BURST )
#define CC1100_WORTIME1       ( 0x36 | CC_BURST )
#define CC1100_WORTIME0       ( 0x37 | CC_BURST )
#define CC1100_PKTSTATUS      ( 0x38 | CC_BURST )
#define CC1100_VCO_VC_DAC     ( 0x39 | CC_BURST )
#define CC1100_TXBYTES        ( 0x3A | CC_BURST )
#define CC1100_RXBYTES        ( 0x3B | CC_BURST )

// Chip Status byte
#define CC_CHIP_RDY    0x80
#define CC_STATE_MASK  0x70
#define CC_STATE( status ) ( status & CC_STATE_MASK )
#define CC_STATE_IDLE         0x00
#define CC_STATE_RX           0x10
#define CC_STATE_TX           0x20
#define CC_STATE_FSTXON       0x30
#define CC_STATE_CALIBRATE    0x40
#define CC_STATE_SETTLING     0x50
#define CC_STATE_RX_OVERFLOW  0x60
#define CC_STATE_TX_UNDERFLOW 0x70


class CC1101_ESP32 {
public:
    /**
     * @brief Constructor for the CC1101 library.
     * @param spi   A reference to an initialized SPI_CC1101 object.
     * @param gdo0  The GPIO pin connected to the CC1101 GDO0 pin.
     * @param gdo2  The GPIO pin connected to the CC1101 GDO2 pin.
     */
    CC1101_ESP32(CC1101_HW_SPI &spi, int8_t gdo0 = -1, int8_t gdo2 = -1);

    /**
     * @brief Initializes the CC1101 chip.
     * @return true on success, false on failure.
     */
    bool begin();

    // --- Public API matching cc1101.h ---

    /**
     * @brief Writes a block of parameters to the CC1101 registers.
     * @param reg   The starting register address.
     * @param nReg  The number of registers to write.
     * @param param A pointer to the buffer containing the data.
     * @return 1 if the write was valid, 0 otherwise.
     */
    uint8_t setParam(uint8_t reg, uint8_t nReg, uint8_t *param);

    /**
     * @brief Reads a block of parameters from the CC1101 registers.
     * @param reg   The starting register address.
     * @param nReg  The number of registers to read.
     * @param param A pointer to the buffer to store the data.
     */
    void readParam(uint8_t reg, uint8_t nReg, uint8_t *param);

    /**
     * @brief Reads the current RSSI value.
     * @return The RSSI value (10 to 138, where 10 = -10dBm, 138 = -138dBm).
     */
    uint8_t readRSSI(void);

    /**
     * @brief Commands the CC1101 to enter IDLE mode.
     */
    void enterIdleMode(void);

    /**
     * @brief Commands the CC1101 to enter Receive (RX) mode.
     */
    void enterRxMode(void);

    /**
     * @brief Commands the CC1101 to enter Transmit (TX) mode.
     */
    void enterTxMode(void);

    /**
     * @brief Writes a single byte to the TX FIFO.
     * @param b The byte to write.
     * @return The number of bytes free in the TX FIFO.
     */
    uint8_t writeFifo(uint8_t b);

    /**
     * @brief Configures GDO0 to signal when the TX FIFO is empty.
     */
    void fifoEnd(void);


    // --- Public API matching cc1101_param.h & nv.h ---

    /**
     * @brief Loads the default configuration registers into Non-Volatile Storage (NVS).
     * @param param  The starting parameter index (0-based).
     * @param nParam The number of parameters to write to NVS.
     * @return The number of bytes written.
     */
    uint8_t loadDefaultConfig(uint8_t param = 0, uint8_t nParam = CC1100_PARAM_MAX);

    /**
     * @brief Gets configuration registers from NVS.
     * @param param  The starting parameter index to read.
     * @param buff   The buffer to store the read data.
     * @param nParam The number of parameters to read.
     * @return The number of bytes read.
     */
    uint8_t getConfig(uint8_t param, uint8_t *buff, uint8_t nParam);

    /**
     * @brief Sets (saves) configuration registers to NVS.
     * @param param  The starting parameter index to write.
     * @param buff   The buffer containing the data to write.
     * @param nParam The number of parameters to write.
     * @return The number of bytes written.
     */
    uint8_t setConfig(uint8_t param, uint8_t *buff, uint8_t nParam);

    /**
     * @brief Loads the default PA (Power Amplifier) table into NVS.
     * @return The number of bytes written.
     */
    uint8_t loadDefaultPaTable(void);

    /**
     * @brief Gets the PA table from NVS.
     * @param paTable A buffer (at least 8 bytes) to store the PA table.
     * @return The number of entries in the PA table.
     */
    uint8_t getPaTable(uint8_t *paTable);

    /**
     * @brief Erases all CC1101 settings from Non-Volatile Storage (NVS).
     */
    void resetNVS(void);

    // --- Public access to low-level functions ---

    /**
     * @brief Reads a single CC1101 register.
     * @param addr The register address (must include CC_READ or CC_BURST if needed).
     * @return The 8-bit value of the register.
     */
    uint8_t readRegister(uint8_t addr);

    /**
     * @brief Writes a single byte to a CC1101 register.
     * @param addr The register address.
     * @param b    The 8-bit value to write.
     * @return The chip status byte.
     */
    uint8_t writeRegister(uint8_t addr, uint8_t b);

    /**
     * @brief Sends a strobe command to the CC1101.
     * @param cmd The strobe command (e.g., CC1100_SRES).
     * @return The chip status byte.
     */
    uint8_t strobe(uint8_t cmd);


private:
    CC1101_HW_SPI& _spi;    // Reference to the SPI helper library
    Preferences _prefs;  // Handle for Non-Volatile Storage
    int8_t _gdo0;
    int8_t _gdo2;
};