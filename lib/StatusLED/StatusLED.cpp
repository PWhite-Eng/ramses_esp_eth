#include "StatusLED.hpp"
#include "log_config.h"         // Logging configuration

StatusLED::StatusLED(int pin, int count)
    : _pixels(count, pin, NEO_GRB + NEO_KHZ800),
      _ledQueueHandle(NULL),
      // --- Initialize Colors ---
      COLOR_HEARTBEAT(Adafruit_NeoPixel::Color(0, 0, 50)), // Blue
      COLOR_RX(Adafruit_NeoPixel::Color(0, 50, 0)),        // Green
      COLOR_TX(Adafruit_NeoPixel::Color(50, 50, 0)),      // Yellow
      COLOR_ERROR(Adafruit_NeoPixel::Color(50, 0, 0)),    // Red
      COLOR_OFF(Adafruit_NeoPixel::Color(0, 0, 0)) {
}

void StatusLED::begin() {
    _pixels.begin();
    _pixels.clear();
    _pixels.show();

    _ledQueueHandle = xQueueCreate(LED_QUEUE_LENGTH, sizeof(LedState));
    if (_ledQueueHandle == NULL) {
        ESP_LOGE(TAG_LED, "StatusLED: FAILED to create LED queue!");
    }
}

QueueHandle_t StatusLED::getQueueHandle() {
    return _ledQueueHandle;
}

void StatusLED::startTask(UBaseType_t priority, BaseType_t core) {
    if (_ledQueueHandle == NULL) {
        ESP_LOGE(TAG_LED, "StatusLED: Cannot start task, queue is NULL. Did you call begin()?");
        return;
    }
    TaskHandle_t ledTaskHandle = nullptr;
    BaseType_t ledtaskCreated = xTaskCreatePinnedToCore(
        ledTask,         // 1. Static task function
        "LED Task",      // 2. Name
        4096,            // 3. Stack size
        this,            // 4. Parameter (pointer to this instance)
        priority,        // 5. Priority
        &ledTaskHandle,  // 6. Task handle
        core             // 7. Core
    );
    if (ledtaskCreated != pdPASS || ledTaskHandle == nullptr) {
      ESP_LOGE(TAG_LED, "FATAL: Failed to create LED Task! Halting.");
      // Send error flash and halt
      LedState errorState = LED_ERROR_FLASH;
      xQueueSend(_ledQueueHandle, &errorState, (TickType_t)0);
      while(1) { vTaskDelay(1000); } // Halt
  } else {
    ESP_LOGI(TAG_LED, "StatusLED: Created and pinned LED Task to Core %d.", (int)core);
  }
}

void StatusLED::ledTask(void *pvParameters) {
    // The static task function just calls the instance's run method
    static_cast<StatusLED*>(pvParameters)->runTask();
}

void StatusLED::runTask() {
    ESP_LOGI(TAG_LED, "LED Task: Starting on Core %d", xPortGetCoreID());
    
    LedState currentState = LED_HEARTBEAT_OFF; // Start off
    LedState newState;
    
    TickType_t timeout = HEARTBEAT_OFF_MS / portTICK_PERIOD_MS;

    _pixels.setPixelColor(0, COLOR_OFF);
    _pixels.show();

    for (;;) {
        // Wait for a new command, with a timeout based on the current state
        if (xQueueReceive(_ledQueueHandle, &newState, timeout)) {
            // --- 1. A command was received ---
            
            if (newState == LED_RX_FLASH || newState == LED_TX_FLASH || newState == LED_ERROR_FLASH) {
                
                uint32_t flashColor = COLOR_OFF;
                if (newState == LED_RX_FLASH) flashColor = COLOR_RX;
                else if (newState == LED_TX_FLASH) flashColor = COLOR_TX;
                else if (newState == LED_ERROR_FLASH) flashColor = COLOR_ERROR;

                _pixels.setPixelColor(0, flashColor);
                _pixels.show();
                vTaskDelay(FLASH_DURATION_MS / portTICK_PERIOD_MS);
                
                _pixels.setPixelColor(0, COLOR_OFF);
                _pixels.show();
                currentState = LED_HEARTBEAT_OFF;
                timeout = HEARTBEAT_OFF_MS / portTICK_PERIOD_MS;
                
                continue; 
            }
            
            currentState = newState;

        } else {
            // --- 2. The queue timed out (heartbeat "tick") ---
            if (currentState == LED_HEARTBEAT_ON) {
                currentState = LED_HEARTBEAT_OFF;
            } else {
                currentState = LED_HEARTBEAT_ON;
            }
        }

        // --- 3. Set the new pixel color and next timeout ---
        if (currentState == LED_HEARTBEAT_ON) {
            _pixels.setPixelColor(0, COLOR_HEARTBEAT);
            timeout = HEARTBEAT_ON_MS / portTICK_PERIOD_MS;
        } else { // LED_HEARTBEAT_OFF or LED_OFF
            _pixels.setPixelColor(0, COLOR_OFF);
            timeout = HEARTBEAT_OFF_MS / portTICK_PERIOD_MS;
        }
        _pixels.show();
        
    } // end for(;;)
}