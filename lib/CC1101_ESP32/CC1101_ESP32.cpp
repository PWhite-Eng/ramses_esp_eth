#include "CC1101_ESP32.hpp"
#include "esp_log.h"

static constexpr unsigned long CC1101_STATE_TIMEOUT_MS = 100; // 100ms timeout

// --- Anonymous namespace for file-local constants ---
namespace {
    static constexpr const char* NVS_KEY_CONFIG = "CC_CFG";
    static constexpr const char* NVS_KEY_PA = "CC_PA";
}

// Default CC1101 configuration registers from cc1101_param.c
static const uint8_t CC_DEFAULT_CFG[CC1100_PARAM_MAX] = {
    0x0D,  //  CC1100_IOCFG2 	  GDO2- RX data
    0x2E,  //  CC1100_IOCFG1 	  GDO1- not used
    0x2E,  //  CC1100_IOCFG0	  GDO0- TX data
    0x07,  //  CC1100_FIFOTHR   default
    0xD3,  //  CC1100_SYNC1     default
    0x91,  //  CC1100_SYNC0     default
    0xFF,  //  CC1100_PKTLEN	  default
    0x04,  //  CC1100_PKTCTRL1  default
    0x31,  //  CC1100_PKTCTRL0  Asynchornous Serial, TX on GDO0, RX on GDOx
    0x00,  //  CC1100_ADDR      default
    0x00,  //  CC1100_CHANNR    default
    0x0f,  //  CC1100_FSCTRL1   default
    0x00,  //  CC1100_FSCTRL0   default
    0x21,  //  CC1100_FREQ2     /
    0x65,  //  CC1100_FREQ1     / 868.3 MHz
    0x6A,  //  CC1100_FREQ0     /
    0x6A,  //  CC1100_MDMCFG4   //
    0x83,  //  CC1100_MDMCFG3   // DRATE_M=131 data rate=38,383.4838867Hz
    0x10,  //  CC1100_MDMCFG2   GFSK, No Sync Word
    0x22,  //  CC1100_MDMCFG1   FEC_EN=0, NUM_PREAMBLE=4, CHANSPC_E=2
    0xF8,  //  CC1100_MDMCFG0   Channel spacing 199.951 KHz
    0x50,  //  CC1100_DEVIATN   
    0x07,  //  CC1100_MCSM2     default
    0x30,  //  CC1100_MCSM1     default
    0x18,  //  CC1100_MCSM0     Auto-calibrate on Idle to RX+TX
    0x16,  //  CC1100_FOCCFG    default
    0x6c,  //  CC1100_BSCFG     default
    0x43,  //  CC1100_AGCCTRL2  
    0x40,  //  CC1100_AGCCTRL1  default
    0x91,  //  CC1100_AGCCTRL0  default
    0x87,  //  CC1100_WOREVT1   default
    0x6B,  //  CC1100_WOREVT0   default
    0xF8,  //  CC1100_WORCTRL   default
    0x56,  //  CC1100_FREND1    default
    0x10,  //  CC1100_FREND0    default
    0xE9,  //  CC1100_FSCAL3
    0x21,  //  CC1100_FSCAL2
    0x00,  //  CC1100_FSCAL1
    0x1f,  //  CC1100_FSCAL0
    0x41,  //  CC1100_RCCTRL1   default
    0x00,  //  CC1100_RCCTRL0   default
    0x59,  //  CC1100_FSTEST    default
    0x7F,  //  CC1100_PTEST     default
    0x3F,  //  CC1100_AGCTEST   default
    0x81,  //  CC1100_TEST2
    0x35,  //  CC1101_TEST1
    0x09,  //  CC1101_TEST0
};

// Default Power Ramp settings
static const uint8_t CC_DEFAULT_PA[CC1100_PA_MAX] = {
    0xC3, 0, 0, 0, 0, 0, 0, 0
};

// --- Constructor ---
CC1101_ESP32::CC1101_ESP32(CC1101_HW_SPI &spi, int8_t gdo0, int8_t gdo2) : _spi(spi) {
    _gdo0 = gdo0;
    _gdo2 = gdo2;
}

// --- Initialization ---
bool CC1101_ESP32::begin() {
    // Init the SPI helper library using its stored constructor values
    if (!_spi.begin()) {
        ESP_LOGE("CC1101_ESP32", "Hardware SPI begin failed");
        return false;
    }

    // Init the GDO pins if specified
    if (_gdo0 >= 0) {
        pinMode(_gdo0, INPUT);
    }
    if (_gdo2 >= 0) {
        pinMode(_gdo2, INPUT);
    }

    // Send reset strobe
    strobe(CC1100_SRES);
    delay(1); // Wait for reset to complete

    // Test the connection using the new library function
    if (!_spi.test_connection()) {
        ESP_LOGE("CC1101_ESP32", "CC1101 test_connection failed!");
        return false;
    }

    // Open NVS
    if (!_prefs.begin("CC1101_ESP32", false)) {
        // Failed to open preferences
        return false; 
    }

    // Load and write config registers from NVS
    uint8_t param[CC1100_PARAM_MAX];
    uint8_t len = getConfig(0, param, sizeof(param));
    for (uint8_t i = 0; i < len; i++) {
        writeRegister(i, param[i]);
    }

    // Load and write PA table from NVS
    uint8_t paTable[CC1100_PA_MAX];
    len = getPaTable(paTable);
    for (uint8_t i = 0; i < len; i++) {
        writeRegister(CC1100_PATABLE, paTable[i]);
    }

    // Set TX FIFO threshold (from original cc_init)
    writeRegister(CC1100_FIFOTHR, (param[CC1100_FIFOTHR] & 0xF0) + 14);

    enterIdleMode();
    return true;
}

// --- Low-level SPI register access ---

uint8_t CC1101_ESP32::readRegister(uint8_t addr) {
    return _spi.read_register(addr);
}

uint8_t CC1101_ESP32::writeRegister(uint8_t addr, uint8_t b) {
    // Replaced with the new library function
    return _spi.write_register(addr, b);
}

uint8_t CC1101_ESP32::strobe(uint8_t cmd) {
    // Replaced with the new library function
    return _spi.cc1101_read_status(cmd);
}

// --- CC1101 Public API ---

uint8_t CC1101_ESP32::setParam(uint8_t reg, uint8_t nReg, uint8_t *param) {
    uint8_t valid = 0;
    if (reg < CC1100_PARAM_MAX && (reg + nReg) < CC1100_PARAM_MAX) {
        valid = 1;
        enterIdleMode();
        for (uint8_t i = 0; i < nReg; i++) {
            writeRegister(reg + i, param[i]);
        }
        enterRxMode(); // Return to RX mode
    }
    return valid;
}

void CC1101_ESP32::readParam(uint8_t reg, uint8_t nReg, uint8_t *param) {
    if (param && reg < CC1100_PARAM_MAX) {
        for (uint8_t i = 0; i < nReg; i++) {
            if (reg + i >= CC1100_PARAM_MAX) break;
            param[i] = readRegister(reg + i);
        }
    }
}

uint8_t CC1101_ESP32::readRSSI(void) {
    int8_t rssi_dec = (int8_t)readRegister(CC1100_RSSI);
    int8_t rssi_dBm = (rssi_dec / 2) - 74;
    return (uint8_t)(-rssi_dBm); // Return as a positive number
}

void CC1101_ESP32::enterIdleMode(void) {
    // Send IDLE strobe until chip reports IDLE state
    unsigned long start = millis();
    while (CC_STATE(strobe(CC1100_SIDLE)) != CC_STATE_IDLE){
        if (millis() - start > CC1101_STATE_TIMEOUT_MS) {
            ESP_LOGE("CC1101", "Timeout waiting for IDLE state!");
            break; // Exit loop on timeout
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Add a 1ms yield
    }
}

void CC1101_ESP32::enterRxMode(void) {
    enterIdleMode();
    // Config from original cc_enter_rx_mode
    writeRegister(CC1100_IOCFG0, 0x2E);
    writeRegister(CC1100_PKTCTRL0, 0x32);
    
    strobe(CC1100_SFRX); // Flush RX FIFO

    unsigned long start = millis();
    while (CC_STATE(strobe(CC1100_SRX)) != CC_STATE_RX){
        if (millis() - start > CC1101_STATE_TIMEOUT_MS) {
            ESP_LOGE("CC1101", "Timeout waiting for RX state!");
            break; // Exit loop on timeout
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Add a 1ms yield
    } // Enter RX
}

void CC1101_ESP32::enterTxMode(void) {
    enterIdleMode();
    // Config from original cc_enter_tx_mode
    writeRegister(CC1100_PKTCTRL0, 0x02);
    writeRegister(CC1100_IOCFG0, 0x02);
    
    strobe(CC1100_SFTX); // Flush TX FIFO

    unsigned long start = millis();
    while (CC_STATE(strobe(CC1100_STX)) != CC_STATE_TX){
        vTaskDelay(1 / portTICK_PERIOD_MS); // Add a 1ms yield
        if (millis() - start > CC1101_STATE_TIMEOUT_MS) {
            ESP_LOGE("CC1101", "Timeout waiting for TX state!");
            break; // Exit loop on timeout
        }
    } // Enter TX
}

uint8_t CC1101_ESP32::writeFifo(uint8_t b) {
    // Write to FIFO register (address 0x3F)
    return writeRegister(CC1100_FIFO, b) & 0x0F; // Return TX fifo space
}

void CC1101_ESP32::fifoEnd(void) {
    // Set GDO0 to assert on TX FIFO empty
    writeRegister(CC1100_IOCFG0, 0x06);
}

// --- NVS (Preferences) API ---

void CC1101_ESP32::resetNVS(void) {
    _prefs.clear();
}

uint8_t CC1101_ESP32::loadDefaultConfig(uint8_t param, uint8_t nParam) {
    if (param + nParam > CC1100_PARAM_MAX) {
        nParam = CC1100_PARAM_MAX - param;
    }
    // Write the default config blob to NVS
    return _prefs.putBytes(NVS_KEY_CONFIG, CC_DEFAULT_CFG, sizeof(CC_DEFAULT_CFG));
}

uint8_t CC1101_ESP32::getConfig(uint8_t param, uint8_t *buff, uint8_t nParam) {
    uint8_t len = 0;
    if (param >= CC1100_PARAM_MAX) return 0;

    // Check if the config exists in NVS
    if (!_prefs.isKey(NVS_KEY_CONFIG)) {
        // It doesn't exist, so create the default
        loadDefaultConfig(0, CC1100_PARAM_MAX);
    }

    // Read the full config blob
    uint8_t cfg_blob[CC1100_PARAM_MAX];
    size_t blob_len = _prefs.getBytes(NVS_KEY_CONFIG, cfg_blob, sizeof(cfg_blob));

    if (blob_len > 0) {
        // Calculate how much data we can actually copy
        if (param + nParam > blob_len) {
            len = blob_len - param;
        } else {
            len = nParam;
        }
        memcpy(buff, cfg_blob + param, len);
    }
    return len;
}

uint8_t CC1101_ESP32::setConfig(uint8_t param, uint8_t *buff, uint8_t nParam) {
    if (param + nParam > CC1100_PARAM_MAX) return 0;

    // Get the current config blob
    uint8_t cfg_blob[CC1100_PARAM_MAX];
    getConfig(0, cfg_blob, sizeof(cfg_blob)); // This will load default if it doesn't exist

    // Modify the blob in memory
    memcpy(cfg_blob + param, buff, nParam);

    // Write the modified blob back to NVS
    size_t written = _prefs.putBytes(NVS_KEY_CONFIG, cfg_blob, sizeof(cfg_blob));
    return (written > 0) ? nParam : 0;
}

uint8_t CC1101_ESP32::loadDefaultPaTable(void) {
    return _prefs.putBytes(NVS_KEY_PA, CC_DEFAULT_PA, sizeof(CC_DEFAULT_PA));
}

uint8_t CC1101_ESP32::getPaTable(uint8_t *paTable) {
    size_t len = 0;

    if (!_prefs.isKey(NVS_KEY_PA)) {
        // PA table doesn't exist, load default
        loadDefaultPaTable();
    }

    len = _prefs.getBytes(NVS_KEY_PA, paTable, CC1100_PA_MAX);

    // Replicate original logic to find end of table
    for (len = 0; len < CC1100_PA_MAX; len++) {
        if (paTable[len] == 0x00 || paTable[len] == 0xFF) {
            break;
        }
    }
    return len;
}