#include "GatewayHandler.hpp"

static constexpr const char* JSON_KEY_MSG = "msg";
static constexpr const char* JSON_KEY_TS  = "ts";
static constexpr TickType_t QUEUE_SEND_WAIT_TICKS = 0; // If the queue is full, it is better to drop that single MQTT message than to stall the radio driver and lose sync with the RF stream

// --- Constructor ---
GatewayHandler::GatewayHandler(EvofwProtocol& proto, EvofwHandler& handler, Stream& host, 
                               QueueHandle_t ledQueue, QueueHandle_t txQueue, QueueHandle_t cmdQueue, 
                               QueueHandle_t pubQueue, QueueHandle_t traceQueue, 
                               const uint8_t gwayClass, const uint8_t maxMsgLen,
                               CommandResultCallback callback)

  : _protocol(proto), 
    _handler(handler), 
    _hostSerial(host), 
    _ledQueueHandle(ledQueue), 
    _txStringQueueHandle(txQueue), 
    _cmdStringQueueHandle(cmdQueue), 
    _publishQueueHandle(pubQueue),
    _traceQueueHandle(traceQueue),
    _GWAY_CLASS(gwayClass), 
    _MQTT_MSG_MAX_LEN(maxMsgLen),
    _cmdResultCallback(callback)
{
    // Initialize pointers to null
    _rxMsg = nullptr;
    _txMsg = nullptr;
    _txMsgMQTT = nullptr;
    _inCmd = 0;
    _cmdBuff = nullptr;
    _nCmd = 0;
}

// --- Public Methods ---

void GatewayHandler::begin() {
    ESP_LOGD(TAG_GWAY, "GatewayHandler: Application handler initialized.");
}

void GatewayHandler::run() {
    // Run the protocol's internal state machine
    _protocol.loop();

    // Process MQTT -> Radio TX queue
    handleMqttTx();

    // Process MQTT -> Command queue
    handleMqttCmd();

    // The logic is a state machine:
    // 1. If we are printing an RX message, finish that.
    // 2. Else, if we are printing a CMD response, finish that.
    // 3. Else, we are idle. Poll for new work.
    if (_rxMsg) {
        handleRadioRx();
    } else if (_nCmd) {
        handleCmdResponse();
    } else {
        handleIdleState();
    }

    // Always check for new commands from the host
    handleHostSerial();
}

// --- Private Methods ---

void GatewayHandler::setLedState(LedState newState) {
    xQueueSend(_ledQueueHandle, &newState, (TickType_t)0);
}

void GatewayHandler::publishRadioPacket(const char* message) {
    char publish_buf[_MQTT_MSG_MAX_LEN];
    strncpy(publish_buf, message, _MQTT_MSG_MAX_LEN - 1);
    publish_buf[_MQTT_MSG_MAX_LEN - 1] = '\0';

    if (xQueueSend(_publishQueueHandle, &publish_buf, QUEUE_SEND_WAIT_TICKS) != pdPASS) {
        ESP_LOGE("TAG_GWAY", "Gateway: CRITICAL: Publish queue full, dropping message. !!!");
    }
}

void GatewayHandler::handleHostSerial() {
    if(_hostSerial.available()) {
        uint8_t byte = _hostSerial.read();
        char cmdStr[TXBUF]; // Buffer to hold the original command
        bool cmd_finished = false;

        if (!_nCmd && (byte == CMD_CHAR || _inCmd)) {
            if (byte == CMD_CHAR && !_inCmd) {
                memset(cmdStr, 0, sizeof(cmdStr));
            }
            _inCmd = _handler.process_cmd_char(byte, &_cmdBuff, &_nCmd, cmdStr);
             if (byte == '\r' && _inCmd) {
                cmd_finished = true;
            }                
            byte = '\0';
        }

        if (byte) {
            if (!_txMsg) {
                _txMsg = _protocol.msg_alloc();
            }
            if (_txMsg) {
                if (_protocol.msg_scan(_txMsg, byte)) {
                    if (_protocol.msg_isValid(_txMsg)) {
                        uint8_t myClass;
                        uint32_t myId;
                        _handler.device_get_id(&myClass, &myId);
                        
                        _protocol.msg_change_addr(_txMsg, 0, _GWAY_CLASS, 730, myClass, myId);
                        _protocol.msg_tx_ready(&_txMsg);
                    } else if (_protocol.getTraceLevel() & TRC_TXERR) {
                        setLedState(LED_ERROR_FLASH);
                        _protocol.msg_rx_ready(&_txMsg);
                    } else {
                        _protocol.msg_free(&_txMsg);
                    }
                }
            }
        } 

        if (cmd_finished && _cmdResultCallback) {
            _cmdResultCallback(cmdStr, _cmdBuff);
        }
    }
}

void GatewayHandler::handleMqttTx() {
    char tx_string[_MQTT_MSG_MAX_LEN];

    // Use a while loop to drain the queue on every call
    while (xQueueReceive(_txStringQueueHandle, &tx_string, (TickType_t)0)) {
        ESP_LOGD(TAG_GWAY, "Gateway: Dequeued TX from MQTT: %s", tx_string);

        if (!_txMsgMQTT) {
            _txMsgMQTT = _protocol.msg_alloc();
        }

        if (_txMsgMQTT) {
            bool scan_complete = false;
            for (int i = 0; tx_string[i] != '\0'; i++) {
                if (_protocol.msg_scan(_txMsgMQTT, tx_string[i])) {
                    scan_complete = true;
                    break;
                }
            }
            if (!scan_complete) {
                scan_complete = _protocol.msg_scan(_txMsgMQTT, '\r');
            }

            if (scan_complete && _protocol.msg_isValid(_txMsgMQTT)) {
                ESP_LOGD(TAG_GWAY, "Gateway: TX message parsed OK, queuing for radio.");
                _protocol.msg_tx_ready(&_txMsgMQTT); // This sets _txMsgMQTT to NULL
            } else {
                ESP_LOGD(TAG_GWAY, "Gateway: TX message from MQTT is invalid.");
                _protocol.msg_free(&_txMsgMQTT); // This sets _txMsgMQTT to NULL
            }
        } else {
             ESP_LOGE(TAG_GWAY, "Gateway: Failed to allocate message object. TX dropped.");
            // We lost the message, but at least the queue is cleared.
        }
    }
}

void GatewayHandler::handleMqttCmd() {
    char cmd_string[_MQTT_MSG_MAX_LEN];

    if (xQueueReceive(_cmdStringQueueHandle, &cmd_string, (TickType_t)0)) {
        ESP_LOGD(TAG_GWAY, "Gateway: Received CMD from MQTT: %s", cmd_string);

        char* responseBuffer = nullptr;
        uint8_t responseLen = 0;
        char originalCmd[TXBUF];
        strncpy(originalCmd, cmd_string, TXBUF-1);
        originalCmd[TXBUF-1] = '\0';

        // We need 2 bytes: 1 for \r and 1 for the \0 null terminator.
        size_t cmd_len = strlen(cmd_string);
        if (cmd_len >= _MQTT_MSG_MAX_LEN - 1) {
            // Command is too long to append \r.
            ESP_LOGE(TAG_GWAY, "Error: Command too long to append CR.");
            // Call back with an error
            if (_cmdResultCallback) _cmdResultCallback(originalCmd, "Error: Command too long");
            return; // Exit the handler
        }

        // Append \r in-place to cmd_string
        cmd_string[cmd_len] = '\r';
        cmd_string[cmd_len + 1] = '\0'; // Add new null terminator

        uint8_t inCmd = _handler.process_cmd_str(cmd_string, &responseBuffer, &responseLen);

        if (inCmd && responseBuffer && responseLen > 0) {
            // Trim trailing newlines from the response
            for(int i = 0; i < responseLen; i++) {
                if (responseBuffer[i] == '\r' || responseBuffer[i] == '\n') {
                    responseBuffer[i] = '\0';
                    break;
                }
            }
            if (_cmdResultCallback) _cmdResultCallback(originalCmd, responseBuffer);
        } else {
            // Command failed or had no response
            if (_cmdResultCallback) _cmdResultCallback(originalCmd, "");
        }
    }
}

void GatewayHandler::publishTrace(const char* message) {
    char trace_buf[_MQTT_MSG_MAX_LEN];
    strncpy(trace_buf, message, _MQTT_MSG_MAX_LEN - 1);
    trace_buf[_MQTT_MSG_MAX_LEN - 1] = '\0';

    // Use a short wait time; if queue is full, drop the trace rather than stalling the radio
    if (xQueueSend(_traceQueueHandle, &trace_buf, (TickType_t)0) != pdPASS) {
        // Queue full, drop message
    }
}

void GatewayHandler::handleRadioRx() {
    if (!_rxMsg) return;

    char fullMsgBuf[300];
    uint16_t bufPos = 0;
    uint8_t nChunk;

    do {
        nChunk = _protocol.msg_print(_rxMsg, _serialTxBuf);
        if (nChunk > 0) {
            if (bufPos + nChunk < sizeof(fullMsgBuf)) {
                memcpy(fullMsgBuf + bufPos, _serialTxBuf, nChunk);
                bufPos += nChunk;
            } else {
                nChunk = 0; 
            }
        }
    } while (nChunk > 0);

    fullMsgBuf[bufPos] = '\0';

    bool isValid = _protocol.msg_isValid(_rxMsg);
    uint8_t tuning = _handler.cc_tuneEnabled();

    if (isValid) {
        if (bufPos > 2 && fullMsgBuf[bufPos-2] == '\r' && fullMsgBuf[bufPos-1] == '\n') {
            fullMsgBuf[bufPos-2] = '\0';
        }
        publishRadioPacket(fullMsgBuf);
        if (_protocol.msg_isTx(_rxMsg)) {
            setLedState(LED_TX_FLASH);
            ESP_LOGD(TAG_GWAY, "Gateway: Confirmed TX Sent: %s", fullMsgBuf);
        } else {
            setLedState(LED_RX_FLASH);
            ESP_LOGD(TAG_GWAY, "Gateway: Received RF Packet: %s", fullMsgBuf);
        }
    } else {
        // If packet is invalid, send it to the Trace Queue
        // This captures * ERR:06, * ERR:04, etc.
        publishTrace(fullMsgBuf);

        setLedState(LED_ERROR_FLASH);
        ESP_LOGE(TAG_GWAY, "Gateway: Invalid RX Packet: %s", fullMsgBuf);
    }

    if (!isValid || tuning != 0 || _protocol.getTraceLevel() != 0) {
        ESP_LOGE(TAG_GWAY, "Gateway: packet is 'NOT valid' or 'tuning active' or 'tracing enabled' - Packet: %s", fullMsgBuf);
    }
    
    _protocol.msg_free(&_rxMsg);
}

void GatewayHandler::handleCmdResponse() {
    if (!_nCmd) return;

    _nCmd -= _hostSerial.write((uint8_t *)_cmdBuff, _nCmd);
    if (!_nCmd) {
        _inCmd = 0;
    }
}

void GatewayHandler::handleIdleState() {
    uint8_t tuning = _handler.cc_tuneEnabled();

    _rxMsg = _protocol.msg_rx_get();

    if (tuning) {
        char cmdStr[TXBUF];
        if (_handler.loop(cmdStr, TXBUF)) {
            char* responseBuffer = nullptr;
            uint8_t responseLen = 0;
            _inCmd = _handler.process_cmd_str(cmdStr, &responseBuffer, &responseLen);

            if (_inCmd && responseBuffer && _cmdResultCallback) {
                 _cmdResultCallback(cmdStr, responseBuffer);
            }
        }
    } else {
        char dummyCmdStr[1];
        _handler.loop(dummyCmdStr, sizeof(dummyCmdStr));
    }

    if (_rxMsg) {
        if (!_protocol.msg_isValid(_rxMsg)) {
            if (_protocol.getTraceLevel() == 0 && !tuning) {
                setLedState(LED_ERROR_FLASH);
                _protocol.msg_free(&_rxMsg);
            }
        }
    }
}