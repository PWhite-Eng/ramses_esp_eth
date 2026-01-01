#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "freertos/queue.h"
#include "EvofwProtocol.hpp"
#include "EvofwHandler.hpp"
#include "StatusLED.hpp"
#include "log_config.h"
#include <functional>

// Define a type for our command result callback function
using CommandResultCallback = std::function<void(const char*, const char*)>;

class GatewayHandler {
private:
    // --- Local State Variables ---
    struct message *_rxMsg = nullptr;
    struct message *_txMsg = nullptr;
    uint8_t _inCmd = 0;
    char *_cmdBuff = nullptr;
    uint8_t _nCmd = 0;
    struct message *_txMsgMQTT = nullptr; 
    static constexpr int SERIAL_TX_BUF_SIZE = 64;
    char _serialTxBuf[SERIAL_TX_BUF_SIZE];

    // --- References to external libraries ---
    EvofwProtocol& _protocol;
    EvofwHandler& _handler;
    Stream& _hostSerial;
    
    // --- RTOS Handles & Config ---
    QueueHandle_t _ledQueueHandle;
    QueueHandle_t _txStringQueueHandle;
    QueueHandle_t _cmdStringQueueHandle;
    QueueHandle_t _publishQueueHandle;
    QueueHandle_t _traceQueueHandle;
    const uint8_t _GWAY_CLASS;
    const uint8_t _MQTT_MSG_MAX_LEN;

    // --- Callback Function ---
    CommandResultCallback _cmdResultCallback;

    // --- Private Method Declarations ---
    void setLedState(LedState newState);
    void publishRadioPacket(const char* message);
    void publishTrace(const char* message);
    void handleHostSerial();
    void handleMqttTx();
    void handleMqttCmd();
    void handleRadioRx();
    void handleCmdResponse();
    void handleIdleState();

public:
    /**
     * @brief Constructor for the Gateway Handler.
     * @param proto Reference to the EvofwProtocol object.
     * @param handler Reference to the EvofwHandler object.
     * @param host Reference to the host serial port (e.g., Serial).
     * @param ledQueue Handle to the status LED queue.
     * @param txQueue Handle to the MQTT->Radio TX queue.
     * @param cmdQueue Handle to the MQTT->Command queue.
     * @param pubQueue Handle to the Radio->MQTT publish queue.
     * @param traceQueue Handle to the trace/logging queue.
     * @param gwayClass The gateway's class ID (e.g., 18).
     * @param maxMsgLen The maximum length for an MQTT message.
     * @param callback A function pointer to call for publishing command results.
     */
    GatewayHandler(EvofwProtocol& proto, EvofwHandler& handler, Stream& host, 
                   QueueHandle_t ledQueue, QueueHandle_t txQueue, QueueHandle_t cmdQueue, 
                   QueueHandle_t pubQueue, QueueHandle_t traceQueue,const uint8_t gwayClass, 
                   const uint8_t maxMsgLen, CommandResultCallback callback);
    
    /**
     * @brief Initializes the handler.
     */
    void begin();
    
    /**
     * @brief The main execution loop for the gateway task.
     */
    void run();
};