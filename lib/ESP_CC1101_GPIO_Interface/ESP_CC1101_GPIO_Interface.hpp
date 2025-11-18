#pragma once

#include "Arduino.h"
#include "driver/spi_master.h" // ESP-IDF Hardware SPI Driver
#include "driver/gpio.h"       // ESP-IDF GPIO Driver
#include "esp_log.h"

// Define which SPI host to use (FSPI/SPI2 is a good default on S3)
#define CC1101_SPI_HOST  SPI2_HOST

class CC1101_HW_SPI {
public:
    CC1101_HW_SPI();
    /**
     * @brief NEW: Constructor to initialize pins and optional clock speed
     */
    CC1101_HW_SPI(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, uint32_t clock_speed = 1000000); // Constructor
    ~CC1101_HW_SPI(); // Destructor to free SPI resources

    /**
     * @brief Initializes the custom Hardware SPI interface.
     * @param sck SPI Clock pin
     * @param miso Master In Slave Out (SO) pin
     * @param mosi Master Out Slave In (SI) pin
     * @param cs Chip Select (CSn) pin
     * @param clock_speed_hz Desired SPI clock speed in Hz (e.g., 1000000 for 1MHz).
     * @return true if SPI bus and GPIO pins were configured successfully, false otherwise.
     */
    bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, uint32_t clock_speed = 1000000);

    /**
     * @brief NEW: Initializes the custom Hardware SPI interface using parameters 
     * stored in the constructor.
     * @return true if successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Checks for a valid connection to the CC1101.
     * @return true if the chip is found and responds correctly, false otherwise.
     */
    bool test_connection();

    /**
     * @brief Sends a command strobe/byte and returns the chip's status byte.
     * This function performs the manual CS assertion, waits for CHIP_RDYn,
     * and then uses the hardware SPI to transfer the byte.
     *
     * @param command The 8-bit command strobe or header byte to send.
     * @return The 8-bit Status Byte, or 0xFF on CHIP_RDYn timeout.
     */
    uint8_t cc1101_read_status(uint8_t command);

    /**
     * @brief Writes a single byte to a configuration register.
     * @param addr The 6-bit register address (e.g., 0x00 to 0x2E).
     * @param value The 8-bit value to write.
     * @return The 8-bit status byte returned by the chip.
     */
    uint8_t write_register(uint8_t addr, uint8_t value);

    /**
     * @brief Reads a single byte from a configuration or status register.
     * @param addr The 6-bit register address (e.g., 0x00 to 0x3D).
     * @return The 8-bit value read from the register.
     */
    uint8_t read_register(uint8_t addr);

    /**
     * @brief Writes multiple bytes using hardware SPI (e.g., TX FIFO or PATABLE).
     * @param addr The 6-bit register address. (Burst bit is added automatically).
     * @param data A pointer to the data buffer to write.
     * @param len The number of bytes to write.
     */
    void write_burst(uint8_t addr, const uint8_t* data, uint8_t len);

    /**
     * @brief Reads multiple bytes using hardware SPI (e.g., RX FIFO).
     * @param addr The 6-bit register address. (Read and Burst bits are added automatically).
     * @param data A pointer to the buffer to store the read data.
     * @param len The number of bytes to read.
     */
    void read_burst(uint8_t addr, uint8_t* data, uint8_t len);

private:
    /**
     * @brief Manually asserts CS and waits for CHIP_RDYn.
     * @return true if chip became ready, false if timed out.
     */
    bool cs_select_and_wait();

    /**
     * @brief Manually de-asserts CS.
     */
    void cs_deselect();

    /**
     * @brief Performs a single-byte hardware SPI transaction.
     * @param tx_data The 8-bit byte to send.
     * @return The 8-bit byte received from MISO.
     */
    uint8_t spi_transfer_byte(uint8_t tx_data);

    spi_device_handle_t _spi_device; // Handle for the SPI device
    // Updated private members to store all pins and clock speed as integers
    int8_t _sck_pin = -1;
    int8_t _miso_pin = -1; 
    int8_t _mosi_pin = -1;
    int8_t _cs_pin = -1;
    uint32_t _clock_speed = 0;
    bool _is_initialized = false;
    
    static const char* TAG;
};