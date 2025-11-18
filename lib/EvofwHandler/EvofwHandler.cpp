#include "EvofwHandler.hpp"
#include "version.h"    // Includes BRANCH, MAJOR, MINOR, SUBVER
#include "esp_mac.h"    // For getting the ESP32 MAC address

// --- Constructor ---
EvofwHandler::EvofwHandler(EvofwProtocol &protocol, CC1101_ESP32 &cc1101)
    : _protocol(protocol), _cc1101(cc1101) {
}

// --- Public Methods ---

void EvofwHandler::begin(uint8_t device_class) {
    device_init(device_class);
    reset_command();
    _tuneState = TUNE_IDLE;
    _tuneEnabled = 0;
    _lastF = 0;
    _lastValidTuneTime = 0;
}

uint8_t EvofwHandler::loop(char *cmd_str_buffer, uint8_t buffer_len) {
    // This loop now only handles the tuning state machine

    if (cc_tuneEnabled()) {
        // Check for a received message to feed to the tuner
        struct message *rx_msg = _protocol.msg_rx_get();
        
        char tune_cmd_buffer[TXBUF];
        uint8_t tune_cmd_len = cc_tune_work(rx_msg, tune_cmd_buffer);

        if (rx_msg) {
            _protocol.msg_free(&rx_msg); // Free message after checking
        }

        if (tune_cmd_len > 0 && tune_cmd_len < buffer_len) {
            // The tuner generated a command. Return 1 so main.cpp can process it.
            return 1;
        }
    }
    return 0; // Nothing to do
}

void EvofwHandler::device_get_id(uint8_t *class_id, uint32_t *id) {
    if (class_id) *class_id = _devClass;
    if (id) *id = _devId;
}

// --- device.c implementation ---

void EvofwHandler::device_init(uint8_t class_id) {
    _devClass = class_id;
    
    // Get the ESP32's base MAC address as a unique ID
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    // Use the lower 22 bits (0x3FFFFF) to match the original AVR logic's mask
    _devId = ((uint32_t)mac[3] << 16) |
             ((uint32_t)mac[4] << 8)  |
             ((uint32_t)mac[5]);
    _devId &= 0x3FFFFF; // Original mask was 0x3FFFF, but signature bytes were 3
                        // Using 22 bits (0x3FFFFF) from 3 bytes seems more likely.
                        // If it was 18 bits (0x3FFFF), use that instead.
}

// --- cc1101_tune.c implementation ---

uint8_t EvofwHandler::cc_tuneEnabled(void) {
    return _tuneEnabled;
}

#define STEP0 0x10
#define TUNE_TIMEOUT (4UL * 60 * 1000) // 4 minutes

uint32_t EvofwHandler::cc_tune(uint8_t validMsg, uint8_t timeout) {
    switch (_tuneState) {
    case TUNE_ABORT:
        _f_current = _f0; // Reset to start frequency
        _tuneState = TUNE_STOP;
        break;

    case TUNE_STOP:
        _tuneState = TUNE_IDLE;
        _tuneEnabled = 0;
        break;

    case TUNE_IDLE:
        _tune_step = -STEP0; // Initial search is for low limit
        _f_current = _f0;
        _tuneState = TUNE_BEGIN;
        break;

    case TUNE_BEGIN: // Begin initial search
        _tune_y = 0;
        _tune_x = _tune_step;
        _f_current = _f0 + _tune_x;
        _tuneState = TUNE_WAIT;
        break;

    case TUNE_WAIT:
        if (validMsg) {
            _tune_x += _tune_step;
            _tune_y += _tune_step;
            _f_current += _tune_step;
        } else if (timeout) {
            _f_current -= _tune_step / 2;
            _tuneState = TUNE_CHOP;
        }
        break;

    case TUNE_CHOP:
        if (validMsg) {
            _tune_y = (_tune_x + _tune_y) / 2;
            _f_current += (_tune_x - _tune_y) / 2;
        } else if (timeout) {
            _tune_x = (_tune_x + _tune_y) / 2;
            _f_current -= (_tune_x - _tune_y) / 2;
        }

        if (abs(_tune_x - _tune_y) <= 1) {
            if (_tune_step < 0) { // Low limit found
                _tune_low = _tune_y;
                _f_current = _f0;
                _tune_step = -_tune_step; // Search in opposite direction
                _tuneState = TUNE_BEGIN;
            } else { // High limit found
                _tune_high = _tune_y;
                _f_current = _f0 + (_tune_low + _tune_high) / 2;
                _tuneState = TUNE_STOP;
            }
        }
    }
    return _f_current;
}

void EvofwHandler::cc_tune_enable(uint8_t enable) {
    if (_tuneEnabled != enable) {
        if (enable) { // Start tune
            uint8_t startFreq[3];
            _cc1101.readParam(CC1100_FREQ2, 3, startFreq);
            _f0 = ((uint32_t)startFreq[0] << 16) |
                  ((uint32_t)startFreq[1] << 8)  |
                  ((uint32_t)startFreq[2]);
            _f_current = _f0;
            _lastF = _f0;
            _tuneEnabled = 1;
            _tuneState = TUNE_IDLE; // Reset state machine
        } else { // Abort tune
            _tuneState = TUNE_ABORT;
        }
    }
}

uint8_t EvofwHandler::cc_tune_work(struct message *msg, char *cmdBuff) {
    uint8_t nCmd = 0;
    unsigned long now = millis();

    uint8_t timeout = 0;
    uint8_t isValid = _protocol.msg_isValid(msg);
    if (!isValid) {
        unsigned long interval = now - _lastValidTuneTime;
        if (interval > TUNE_TIMEOUT)
            timeout = 1;
    }

    if (isValid || timeout)
        _lastValidTuneTime = now;

    _f_current = cc_tune(isValid, timeout);
    if (_lastF != _f_current) {
        // Build !c command string
        nCmd  = sprintf(cmdBuff, "!c %02X %02X %02X %02X\r",
                        CC1100_FREQ2,
                        (uint8_t)((_f_current >> 16) & 0xFF),
                        (uint8_t)((_f_current >> 8) & 0xFF),
                        (uint8_t)((_f_current >> 0) & 0xFF));
        _lastF = _f_current;
        _lastValidTuneTime = now;
    }
    return nCmd;
}

// --- cmd.c implementation ---

void EvofwHandler::reset_command(void) {
    memset(&_command, 0, sizeof(_command));
}

uint8_t EvofwHandler::get_hex(uint8_t len, char *param) {
    uint8_t value = 0;
    if (len > 2) len = 2;
    while (len) {
        char p = *(param++);
        len--;
        value <<= 4;
        value += (p >= '0' && p <= '9') ? p - '0'
               : (p >= 'A' && p <= 'F') ? p - 'A' + 10
               : (p >= 'a' && p <= 'f') ? p - 'a' + 10
               : 0;
    }
    return value;
}

uint8_t EvofwHandler::cmd_trace(struct cmd_state *cmd) {
    if (cmd->n > 1)
        _protocol.setTraceLevel(get_hex(cmd->n - 1, cmd->buffer + 1));
    cmd->n = sprintf(cmd->buffer, "# !T=%02x\r\n", _protocol.getTraceLevel());
    return 1;
}

uint8_t EvofwHandler::cmd_version(struct cmd_state *cmd) {
    cmd->n = sprintf(cmd->buffer, "# %s %d.%d.%d\r\n", BRANCH, MAJOR, MINOR, SUBVER);
    return 1;
}

uint8_t EvofwHandler::cmd_id(struct cmd_state *cmd) {
    uint8_t class_id;
    uint32_t id;
    device_get_id(&class_id, &id);
    cmd->n = sprintf(cmd->buffer, "# %02hhu:%06lu\r\n", class_id, id);
    return 1;
}

uint8_t EvofwHandler::cmd_boot(struct cmd_state *cmd) {
    ESP.restart();
    return 0; // Will not be reached
}

uint8_t EvofwHandler::cmd_cc1101(struct cmd_state *cmd) {
    uint8_t validCmd = 0;
    if (cmd->n > 1) {
        switch (cmd->buffer[1]) {
        case 'R':
        case 'r':
            _cc1101.loadDefaultConfig();
            _cc1101.loadDefaultPaTable();
            cmd->n = sprintf(cmd->buffer, "# !%c Reset\r\n", cmd->buffer[0]);
            validCmd = 1;
            break;

        case 'S':
        case 's': {
            uint8_t param[CC1100_PARAM_MAX];
            _cc1101.readParam(0, CC1100_PARAM_MAX, param);
            _cc1101.setConfig(0, param, CC1100_PARAM_MAX);
            cmd->n = sprintf(cmd->buffer, "# !%c Saved\r\n", cmd->buffer[0]);
            validCmd = 1;
            break;
        }
        case ' ': {
            uint8_t nParam = 0;
            uint8_t param[5]; // CC_MAX_PARAM
            uint8_t start, end;
            uint8_t n = cmd->n;
            end = start = 2;
            while (n > end) {
                while (n > start && cmd->buffer[start] == ' ') start++;
                end = start;
                while (n > end && cmd->buffer[end] != ' ') end++;
                if (end > start)
                    param[nParam++] = get_hex(end - start, cmd->buffer + start);
                start = end;
                if (nParam == 5) break; // CC_MAX_PARAM
            }
            if (nParam > 1) { // Need at least reg + 1 data byte
                cmd->n = sprintf(cmd->buffer, "# !%c", cmd->buffer[0]);
                cmd->n += sprintf(cmd->buffer + cmd->n, " %02x", param[0]);
                for (n = 1; n < nParam; n++)
                    cmd->n += sprintf(cmd->buffer + cmd->n, " %02x", param[n]);
                cmd->n += sprintf(cmd->buffer + cmd->n, "\r\n");

                if (param[0] < CC1100_PARAM_MAX && (param[0] + nParam - 1) <= CC1100_PARAM_MAX) {
                    _cc1101.setParam(param[0], nParam - 1, param + 1);
                    validCmd = 1;
                }
            }
            break;
        }
        }
    }
    return validCmd;
}

uint8_t EvofwHandler::cmd_cc_tune(struct cmd_state *cmd) {
    uint8_t validCmd = 0;
    uint8_t param[3];
    if (cmd->n > 1) {
        switch (cmd->buffer[1] & ~('A' ^ 'a')) { // ToUpper
        case 'T':
            cc_tune_enable(1);
            cmd->n = sprintf(cmd->buffer, "# !%c Tune\r\n", cmd->buffer[0]);
            validCmd = 1;
            break;
        case 'A':
            cc_tune_enable(0);
            cmd->n = sprintf(cmd->buffer, "# !%c Abort\r\n", cmd->buffer[0]);
            validCmd = 1;
            break;
        case 'R':
            cc_tune_enable(0);
            _cc1101.loadDefaultConfig(CC1100_FREQ2, 3); // Reset Freq in NVS
            _cc1101.getConfig(CC1100_FREQ2, param, 3);  // Get it from NVS
            _cc1101.setParam(CC1100_FREQ2, 3, param);   // Write to chip
            cmd->n = sprintf(cmd->buffer, "# !%c Reset F=%02x%02x%02x\r\n", cmd->buffer[0], param[0], param[1], param[2]);
            validCmd = 1;
            break;
        case 'S':
            _cc1101.readParam(CC1100_FREQ2, 3, param);  // Read from chip
            _cc1101.setConfig(CC1100_FREQ2, param, 3);  // Save to NVS
            cmd->n = sprintf(cmd->buffer, "# !%c Saved F=%02x%02x%02x\r\n", cmd->buffer[0], param[0], param[1], param[2]);
            validCmd = 1;
            break;
        }
    } else {
        _cc1101.readParam(CC1100_FREQ2, 3, param);
        cmd->n = sprintf(cmd->buffer, "# !%c", cmd->buffer[0]);
        cmd->n += sprintf(cmd->buffer + cmd->n, " F=%02x%02x%02x", param[0], param[1], param[2]);
        if (cc_tuneEnabled())
            cmd->n += sprintf(cmd->buffer + cmd->n, " tuning");
        cmd->n += sprintf(cmd->buffer + cmd->n, "\r\n");
        validCmd = 1;
    }
    return validCmd;
}

uint8_t EvofwHandler::cmd_eeprom(struct cmd_state *cmd) {
    uint8_t validCmd = 0;
    if (cmd->n > 1) {
        switch (cmd->buffer[1] & ~('A' ^ 'a')) {
        case 'R':
            _cc1101.resetNVS(); // This is where the NV logic was moved
            cmd->n = sprintf(cmd->buffer, "# !%c reset\r\n", cmd->buffer[0]);
            validCmd = 1;
            break;
        }
    }
    return validCmd;
}

uint8_t EvofwHandler::check_command(struct cmd_state *cmd) {
    uint8_t validCmd = 0;
    if (cmd->n > 0) {
        switch (cmd->buffer[0] & ~('A' ^ 'a')) { // ToUpper
        case 'V': validCmd = cmd_version(cmd); break;
        case 'T': validCmd = cmd_trace(cmd);   break;
        case 'B': validCmd = cmd_boot(cmd);    break;
        case 'C': validCmd = cmd_cc1101(cmd);  break;
        case 'F': validCmd = cmd_cc_tune(cmd); break;
        case 'E': validCmd = cmd_eeprom(cmd);  break;
        case 'I': validCmd = cmd_id(cmd);      break;
        }
    }
    return validCmd;
}

uint8_t EvofwHandler::process_cmd_char(uint8_t byte, char **buffer, uint8_t *n, char* cmdStrBuf) {
    if (byte == CMD_CHAR) {
        reset_command();
        _command.inCmd = 1;
    } else if (_command.inCmd) {
        if (byte == '\r') {
            if (_command.n == 0) {
                _command.inCmd = 0; // Empty command, ignore
            } else {
                // Store the command string before processing
                if (cmdStrBuf) {
                    strncpy(cmdStrBuf, _command.buffer, _command.n);
                    cmdStrBuf[_command.n] = '\0';
                }

                _command.inCmd = check_command(&_command);
                if (_command.inCmd) {
                    if (buffer) (*buffer) = _command.buffer;
                    if (n) (*n) = _command.n;
                }
            }
        } else if (byte >= ' ') { // Printable character
            if (_command.n < TXBUF)
                _command.buffer[_command.n++] = byte;
            else
                _command.inCmd = 0; // Buffer overflow, reset
        }
    }
    return _command.inCmd;
}

uint8_t EvofwHandler::process_cmd_str(const char *str, char **buffer, uint8_t *n) {
    uint8_t inCmd = 0;
    if (str) {
        // Manually trigger the command start
        inCmd = process_cmd_char(CMD_CHAR, buffer, n, nullptr);
        for (int i = 0; inCmd && str[i] != '\0'; i++) {
            inCmd = process_cmd_char(str[i], buffer, n, nullptr);
        }
    }
    return inCmd;
}