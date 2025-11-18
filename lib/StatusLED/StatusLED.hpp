#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "freertos/queue.h"

// --- LED States and Colors ---
enum LedState {
  LED_OFF,
  LED_HEARTBEAT_ON,
  LED_HEARTBEAT_OFF,
  LED_RX_FLASH,
  LED_TX_FLASH,
  LED_ERROR_FLASH
};

class StatusLED {
public:
    /**
     * @brief Constructor for the Status LED manager.
     * @param pin The GPIO pin the NeoPixel is connected to.
     * @param count The number of LEDs in the strip (usually 1).
     */
    StatusLED(int pin, int count = 1);

    /**
     * @brief Initializes the NeoPixel and creates the RTOS queue.
     * Must be called from setup().
     */
    void begin();

    /**
     * @brief Starts the FreeRTOS task to manage the LED.
     * @param priority The priority to assign to the task.
     * @param core The core to pin the task to (0 or 1).
     */
    void startTask(UBaseType_t priority = 1, BaseType_t core = 1);

    /**
     * @brief Gets the queue handle for sending LED commands.
     * @return The QueueHandle_t for LedState commands.
     */
    QueueHandle_t getQueueHandle();

private:
    /**
     * @brief The static FreeRTOS task function.
     * @param pvParameters A void pointer to the StatusLED instance.
     */
    static void ledTask(void *pvParameters);

    /**
     * @brief The instance method containing the task's infinite loop.
     */
    void runTask();

    Adafruit_NeoPixel _pixels;
    QueueHandle_t _ledQueueHandle;

    // --- Configuration ---
    static constexpr int LED_QUEUE_LENGTH = 5;

    // --- Colors ---
    const uint32_t COLOR_HEARTBEAT;
    const uint32_t COLOR_RX;
    const uint32_t COLOR_TX;
    const uint32_t COLOR_ERROR;
    const uint32_t COLOR_OFF;

    // --- Timing ---
    const int FLASH_DURATION_MS = 100;
    const int HEARTBEAT_ON_MS   = 100;
    const int HEARTBEAT_OFF_MS  = 900;
};