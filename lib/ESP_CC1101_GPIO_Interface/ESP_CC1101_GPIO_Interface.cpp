#include "ESP_CC1101_GPIO_Interface.hpp"

// Timeout for waiting for CHIP_RDYn in microseconds.
// 50ms (50,000us) as requested.
#define CHIP_RDYN_TIMEOUT_US 50000

// Define the logging tag
const char* CC1101_HW_SPI::TAG = "CC1101_HW_SPI";

// --- Header Byte Definitions ---
#define CC1101_HEADER_WRITE_BURST 0x40
#define CC1101_HEADER_READ_SINGLE 0x80
#define CC1101_HEADER_READ_BURST  0xC0


CC1101_HW_SPI::CC1101_HW_SPI() : _spi_device(NULL) {
    // Default constructor
    }

// NEW Constructor: Stores pin and speed configuration
CC1101_HW_SPI::CC1101_HW_SPI(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, uint32_t clock_speed) 
    : _spi_device(NULL), _sck_pin(sck), _miso_pin(miso), _mosi_pin(mosi), _cs_pin(cs), _clock_speed(clock_speed), _is_initialized(false) {
    // Constructor
}

CC1101_HW_SPI::~CC1101_HW_SPI() {
    // Destructor: free up the SPI bus
    if (_is_initialized) {
        ESP_LOGD(TAG, "Releasing SPI device and bus.");
        spi_bus_remove_device(_spi_device);
        spi_bus_free(CC1101_SPI_HOST);
    }
}

// The begin function is modified to use stored pins if available.
bool CC1101_HW_SPI::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, uint32_t clock_speed) {
    // Determine the final configuration using stored values (from constructor) or passed values (from begin)
    int8_t final_sck  = (_sck_pin != -1) ? _sck_pin : sck;
    int8_t final_miso = (_miso_pin != -1) ? _miso_pin : miso;
    int8_t final_mosi = (_mosi_pin != -1) ? _mosi_pin : mosi;
    int8_t final_cs   = (_cs_pin != -1) ? _cs_pin : cs;
    uint32_t final_clock_speed = (_clock_speed != 0) ? _clock_speed : clock_speed;
    
    // Sanity check for necessary pins
    if (final_sck == -1 || final_miso == -1 || final_mosi == -1 || final_cs == -1 || final_clock_speed == 0) {
         ESP_LOGE(TAG, "Cannot initialize SPI: Pin/speed configuration missing or invalid. Did you pass all pins to the constructor or begin()?");
         return false;
    }
    
    // Store final configuration to the object's members
    _sck_pin = final_sck;
    _miso_pin = final_miso;
    _mosi_pin = final_mosi;
    _cs_pin = final_cs;
    _clock_speed = final_clock_speed;

    ESP_LOGI(TAG, "Initializing CC1101 Hardware SPI driver...");
    ESP_LOGD(TAG, "SCK=%d, MISO=%d, MOSI=%d, CS=%d, Speed=%lu Hz", _sck_pin, _miso_pin, _mosi_pin, _cs_pin, _clock_speed);

    esp_err_t ret;

    // --- Step 1: Configure the GPIO pins ---
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << _cs_pin), // Use the stored pin
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&cs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS GPIO: %s", esp_err_to_name(ret));
        return false;
    }
    gpio_set_level((gpio_num_t)_cs_pin, 1); // Cast to gpio_num_t

    gpio_config_t miso_conf = {
        .pin_bit_mask = (1ULL << _miso_pin), // Use the stored pin
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&miso_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MISO GPIO: %s", esp_err_to_name(ret));
        return false;
    }

    // --- Step 2: Configure the Hardware SPI Bus ---
    spi_bus_config_t buscfg = {
        .mosi_io_num = _mosi_pin, // Use stored pins
        .miso_io_num = _miso_pin, // Use stored pins
        .sclk_io_num = _sck_pin,  // Use stored pins
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64 // Allow larger burst transfers
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0, // SPI mode 0
        .clock_speed_hz = (int)_clock_speed, // Use stored speed
        .spics_io_num = -1, // Manual CS control
        .queue_size = 1
    };

    ret = spi_bus_initialize(CC1101_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    ret = spi_bus_add_device(CC1101_SPI_HOST, &devcfg, &_spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(CC1101_SPI_HOST);
        return false;
    }

    ESP_LOGI(TAG, "Hardware SPI driver initialized successfully.");
    _is_initialized = true;
    return true;
}

// Overloaded begin() that uses constructor values
bool CC1101_HW_SPI::begin() {
    return begin(-1, -1, -1, -1, 0);
}

bool CC1101_HW_SPI::cs_select_and_wait() {
    if (!_is_initialized) return false;

    gpio_set_level((gpio_num_t)_cs_pin, 0); // Cast to gpio_num_t
    ESP_LOGD(TAG, "CSn low. Waiting for CHIP_RDYn (MISO low)...");

    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)_miso_pin) == 1) { // Cast to gpio_num_t
        if ((esp_timer_get_time() - start_time) > CHIP_RDYN_TIMEOUT_US) {
            ESP_LOGE(TAG, "CHIP_RDYn timeout! MISO pin never went low.");
            gpio_set_level((gpio_num_t)_cs_pin, 1); // Cast to gpio_num_t
            return false;
        }
    }
    ESP_LOGD(TAG, "CHIP_RDYn asserted.");
    return true;
}

void CC1101_HW_SPI::cs_deselect() {
    if (!_is_initialized) return;
    gpio_set_level((gpio_num_t)_cs_pin, 1); // Cast to gpio_num_t
    ESP_LOGD(TAG, "CSn high.");
}

uint8_t CC1101_HW_SPI::spi_transfer_byte(uint8_t tx_data) {
    if (!_is_initialized) return 0xFF;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8; 
    t.tx_buffer = &tx_data;
    uint8_t rx_data = 0;
    t.rx_buffer = &rx_data;

    esp_err_t ret = spi_device_polling_transmit(_spi_device, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_transfer_byte failed: %s", esp_err_to_name(ret));
        return 0xFF;
    }
    
    ESP_LOGD(TAG, "HW SPI TX: 0x%02X, RX: 0x%02X", tx_data, rx_data);
    return rx_data;
}

// --- Public API Functions ---

uint8_t CC1101_HW_SPI::cc1101_read_status(uint8_t command) {
    ESP_LOGD(TAG, "Read Status (Strobe): 0x%02X", command);
    uint8_t status = 0xFF; 

    if (cs_select_and_wait()) {
        status = spi_transfer_byte(command);
    }
    cs_deselect();
    return status;
}

bool CC1101_HW_SPI::test_connection() {
    if (!_is_initialized) {
        ESP_LOGE(TAG, "Cannot test connection, driver not initialized.");
        return false;
    }
    ESP_LOGI(TAG, "Testing CC1101 connection...");
    
    // Status registers (0x30-0x3D) MUST be read with the BURST bit set.
    uint8_t partnum_addr = 0x30 | CC1101_HEADER_READ_BURST; // 0xF0
    uint8_t version_addr = 0x31 | CC1101_HEADER_READ_BURST; // 0xF1

    uint8_t partnum = 0, version = 0;

    if (cs_select_and_wait()) {
        spi_transfer_byte(partnum_addr); // Send PARTNUM read header
        partnum = spi_transfer_byte(0x00); // Read data
    }
    cs_deselect();

    if (cs_select_and_wait()) {
        spi_transfer_byte(version_addr); // Send VERSION read header
        version = spi_transfer_byte(0x00); // Read data
    }
    cs_deselect();
    // --- END OF FIXED LOGIC ---

    if (partnum == 0x00 && version == 0x14) {
        ESP_LOGI(TAG, "CC1101 connection successful. PARTNUM=0x%02X, VERSION=0x%02X", partnum, version);
        return true;
    } else {
        ESP_LOGE(TAG, "CC1101 connection FAILED. Expected 0x00/0x14, Got 0x%02X/0x%02X", partnum, version);
        return false;
    }
}

uint8_t CC1101_HW_SPI::write_register(uint8_t addr, uint8_t value) {
    ESP_LOGD(TAG, "Write Register: 0x%02X, Value: 0x%02X", addr, value);
    uint8_t status = 0xFF;
    addr &= 0x3F; // R/W=0, Burst=0

    if (cs_select_and_wait()) {
        status = spi_transfer_byte(addr);
        spi_transfer_byte(value); // Last status byte is ignored, that's fine
    }
    cs_deselect();
    return status; // Returns status from *header* byte
}

uint8_t CC1101_HW_SPI::read_register(uint8_t addr) {
    ESP_LOGD(TAG, "Read Register: 0x%02X", addr);
    uint8_t value = 0xFF;
    uint8_t header = 0;

    // --- THIS IS THE FIXED LOGIC ---
    if (addr >= 0x30 && addr <= 0x3D) {
        // Status Register: R/W=1, Burst=1
        header = (addr & 0x3F) | CC1101_HEADER_READ_BURST;
    } else {
        // Config Register: R/W=1, Burst=0
        header = (addr & 0x3F) | CC1101_HEADER_READ_SINGLE;
    }
    // --- END OF FIXED LOGIC ---

    if (cs_select_and_wait()) {
        spi_transfer_byte(header); // Send header, ignore status
        value = spi_transfer_byte(0x00); // Send dummy, get value
    }
    cs_deselect();
    return value;
}

void CC1101_HW_SPI::write_burst(uint8_t addr, const uint8_t* data, uint8_t len) {
    if (len == 0 || !_is_initialized) return;
    ESP_LOGD(TAG, "Write Burst: 0x%02X, Length: %d", addr, len);
    
    uint8_t header = (addr & 0x3F) | CC1101_HEADER_WRITE_BURST;
    
    if (cs_select_and_wait()) {
        spi_transfer_byte(header); // Send header

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = len * 8; 
        t.tx_buffer = data;
        
        esp_err_t ret = spi_device_polling_transmit(_spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "write_burst failed: %s", esp_err_to_name(ret));
        }
    }
    cs_deselect();
}

void CC1101_HW_SPI::read_burst(uint8_t addr, uint8_t* data, uint8_t len) {
    if (len == 0 || !_is_initialized) return;
    ESP_LOGD(TAG, "Read Burst: 0x%02X, Length: %d", addr, len);

    uint8_t header = (addr & 0x3F) | CC1101_HEADER_READ_BURST;

    if (cs_select_and_wait()) {
        spi_transfer_byte(header); // Send header

        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = len * 8; 
        t.rx_buffer = data; 
        
        esp_err_t ret = spi_device_polling_transmit(_spi_device, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "read_burst failed: %s", esp_err_to_name(ret));
        }
    }
    cs_deselect();
}