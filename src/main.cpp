/**
 * @file main.cpp
 * @author Phil White
 * @brief Main application entry point for the ESP32-S3 Evofw MQTT Gateway.
 * Initializes hardware (Ethernet, CC1101), starts FreeRTOS tasks,
 * and handles core application logic.
 * @version 1.0
 * @date 2025-11-01
 *
 * @copyright Copyright (c) 2025 - Phil White
 * Licensed under the MIT License.
 */
//-----------------------------------------------------------------------------
//  Header Files
//-----------------------------------------------------------------------------
// C/C++ System Libraries
#include <SPI.h>
#include <driver/gpio.h>
#include "freertos/queue.h"

// Library Headers
#include <Arduino.h>
#include <SPI.h>
#include <EthernetESP32.h>                // Ethernet library for ESP32 and W5500 on the SPI bus
#include <chrono>                         // For high-precision time
#include <ArduinoJson.h>
#include <MQTT.h>
#include <ArduinoHA.h>
#include <time.h>                         // For NTP time synchronization
#include "esp_sntp.h"                     // Add this for sntp_set_sync_interval

// Project-specific Headers
#include "config_secrets.h"               // User configuration
#include "config_pins.h"                  // Hardware pin definitions
#include "ESP_CC1101_GPIO_Interface.hpp"  // CC1101 Driver
#include "EvofwProtocol.hpp"              // Evofw Protocol Handler
#include "EvofwHandler.hpp"               // Evofw Device Handler
#include "StatusLED.hpp"                  // Status LED library
#include "GatewayHandler.hpp"             // Gateway Handler
#include "log_config.h"                   // Logging configuration
#include "WiFi.h"                         // Needed to access the hardware MAC address functions
#include "esp_mac.h"                      // For getting the ESP32 MAC address

//-----------------------------------------------------------------------------
//  Constants
//-----------------------------------------------------------------------------

// --- Task Configuration ---
static constexpr int       MAIN_TASK_PRIORITY   = 1;
static constexpr int       MAIN_TASK_CORE       = 0;
static constexpr uint32_t  MAIN_TASK_STACK_SIZE = 4096;

static constexpr int       LED_TASK_PRIORITY    = 1;
static constexpr int       LED_TASK_CORE        = 1;

static constexpr int       NET_TASK_PRIORITY    = 1;
static constexpr int       NET_TASK_CORE        = 1;
static constexpr uint32_t  NET_TASK_STACK_SIZE  = 4096;

// --- Queue Configuration ---
static constexpr uint8_t MQTT_QUEUE_LENGTH = 20;         // Number of items in each queue
static constexpr uint8_t MQTT_MSG_MAX_LEN = 255;         // Max len for a queued string
static constexpr TickType_t QUEUE_SEND_WAIT_TICKS = 10;  // 10 ticks to wait if queue is full

// --- Radio/Serial Configuration ---
static constexpr uint32_t SPI_CLK_RATE =  6000000;  // 6 MHz SPI clock for CC1101
static constexpr uint32_t RADIO_BAUD_RATE = 38400;  // 38400 bps for Evofw radio communication
static constexpr uint32_t HOST_BAUD_RATE = 115200;  // Baud rate for USB Serial
static constexpr uint8_t  GWAY_CLASS = 18;          // Gateway device class

// mac address buffer
uint8_t mac[6];

// needed to configure W5500 on the SPI bus, which is then used by EthernetESP32
W5500Driver driver(W5500_CS, -1, W5500_RST);

//-----------------------------------------------------------------------------
//  Application Context
//-----------------------------------------------------------------------------
/**
 * @brief A single struct to hold all application state, drivers, and handles.
 * This struct is created once and a pointer to it is passed to the
 * FreeRTOS tasks, avoiding the use of global variables.
 */
struct AppContext {
    // --- Hardware Drivers ---
    StatusLED&     led;
    CC1101_HW_SPI& cc1101_spi;
    CC1101_ESP32&  cc1101;
    EvofwProtocol& protocol;
    EvofwHandler&  app_handler;
    // --- Network Handler ---
    EthernetClient& ethClient;
    // --- RTOS Queues ---
    QueueHandle_t ledQueueHandle;
    QueueHandle_t txStringQueueHandle;
    QueueHandle_t cmdStringQueueHandle;
    QueueHandle_t publishQueueHandle;
    QueueHandle_t cc1101StateQueueHandle;
    // --- Gateway Handler ---
    GatewayHandler* gateway;
};

//-----------------------------------------------------------------------------
//  Global Variables
//-----------------------------------------------------------------------------

// --- Create all driver and object instances as static ---
static StatusLED statusLed(LED_PIN); // Status LED manager
static CC1101_HW_SPI cc1101_spi(
    CC1101_SCLK, 
    CC1101_MISO, 
    CC1101_MOSI, 
    CC1101_CSN, 
    SPI_CLK_RATE
);
static CC1101_ESP32 cc1101(cc1101_spi, CC1101_GDO0, CC1101_GDO2);          // CC1101 Driver
static EvofwProtocol protocol(cc1101, Serial1, CC1101_GDO0, CC1101_GDO2);  // We use Serial1 for the async RX data from the CC1101's GDO2 pin
static EvofwHandler app_handler(protocol, cc1101); 
static EthernetClient ethClient;                                           // Ethernet client for MQTT

// --- Global HA/MQTT objects ---
HADevice device;
HAMqtt mqtt(ethClient, device);
HASensor cc1101_state_sensor("cc1101_state");                              // Home Assistant sensor for CC1101 state
HABinarySensor tuning_active_sensor("tuning_active");                      // Home Assistant binary sensor for tuning active state
HASensor boot_time_sensor("boot_time");                                    // Home Assistant sensor for uptime in hours // Publish with 2 decimal places
char boot_timestamp_str[40];                                               // Buffer to hold boot timestamp string

// --- Global topic buffers (filled in setup) ---
char device_id_str[32];
char ha_device_id_str[32];
char macString[13];
char macStringLower[13];
char base_topic[64];
char ha_base_topic[64];
char rx_topic[80];
char tx_topic[80];
char cmd_topic[80];
char cmd_result_topic[80];
char boot_topic[128];             // Holds the topic string: RAMSES/GATEWAY/.../boot_time/stat_t
bool g_bootTimeVerified = false;  // Flag: True if we have received the timestamp back from MQTT

// --- Create the ONE Application Context ---
// We initialize it with references to our static objects.
static AppContext app = {
.led = statusLed,
    .cc1101_spi = cc1101_spi,
    .cc1101 = cc1101,
    .protocol = protocol,
    .app_handler = app_handler,
    .ethClient = ethClient,
    .ledQueueHandle = nullptr,
    .txStringQueueHandle = nullptr,
    .cmdStringQueueHandle = nullptr,
    .publishQueueHandle = nullptr,
    .cc1101StateQueueHandle = nullptr,
    .gateway = nullptr
};

// setup SPI for W5500
SPIClass hspi(HSPI); // Or use SPI3_HOST

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void setupNTP(void);                                             // Setup NTP time synchronization
void getTimestamp(char* buffer, size_t buffer_len);              // Get the current timestamp and place it in the provided buffer  

void gatewayTask(void *pvParameters);                            // Main gateway task

void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length);
void onMqttConnected();

void processPublishQueue();                                      // Checks the publish queue and sends any radio messages to MQTT.
void processCC1101StateQueue();                                  // Checks the state queue and publishes the CC1101 state to HA.
void publishCommandResult(const char* cmd, const char* result);  // Public callback function to be passed to GatewayHandler.

void verifyBootTime();                                           // Verifies the boot time by subscribing to the boot topic.

//-----------------------------------------------------------------------------
//  Setup
//-----------------------------------------------------------------------------
/**
 * @brief Arduino setup function. Runs once on Core 1.
 * Initializes hardware, network, RTOS queues, and all libraries.
 * Creates and starts the main application tasks.
 */
void setup() {
  Serial.begin(HOST_BAUD_RATE);
 
  Serial.setDebugOutput(true);  // Redirect ESP_LOG output to the USB Serial port
  
  // Wait for a an active serial connection.
  // This is critical for S3 native USB. It will wait up to 15 seconds.
  unsigned long serial_timeout = millis() + 15000;
  while (!Serial && (millis() < serial_timeout)) {
    ; // wait
  }
  
  ESP_LOGI(TAG_MAIN, "ESP32-S3 Evofw Gateway Starting, running on Core %d", xPortGetCoreID());

  // Install the global GPIO ISR service before using any interrupts
  // This is required by the EvofwProtocol library for the GDO0 interrupt
  gpio_install_isr_service(0);

  // --- Hardware Setup ---

  // Initialize the StatusLED library
  statusLed.begin();
  app.ledQueueHandle = app.led.getQueueHandle(); // Get queue for GatewayHandler
  
  // ETHERNET SETUP
  esp_read_mac(mac, ESP_MAC_BASE);  // Get the ESP32's MAC address
  ESP_LOGI(TAG_MAIN, "Device MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
   
  ESP_LOGI(TAG_NET,"NET_SETUP, Initializing W5500 Ethernet...");
  // Initialize the HSPI bus with specific pins
  hspi.begin(W5500_SCLK, W5500_MISO, W5500_MOSI);
  ESP_LOGI(TAG_NET,"NET_SETUP, SPI Initialized for W5500.");

  // Tell the W5500 driver to use the SPI1 bus
  driver.setSPI(hspi);

  // Initialize Ethernet using DHCP to obtain an IP address
  Ethernet.init(driver);

  // Attempt to get an IP address via DHCP
  ESP_LOGI(TAG_NET,"NET_SETUP, Starting DHCP, this may take a moment...");
    
  if (!Ethernet.begin((byte*)mac)) { // Cast mac to non-const, as Ethernet.h expects
    ESP_LOGE(TAG_NET,"NET_SETUP, Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        ESP_LOGE(TAG_NET,"NET_SETUP, Ethernet hardware was not found.");
    } else if (Ethernet.linkStatus() == LinkOFF) {
        ESP_LOGE(TAG_NET,"NET_SETUP, Ethernet cable is not connected.");
    }
    ESP_LOGE(TAG_MAIN, "Ethernet setup failed! Halting.");
    // Halt with a red light
    LedState errorState = LED_ERROR_FLASH;
    xQueueSend(app.ledQueueHandle, &errorState, (TickType_t)0); // Send error
    while(1) { vTaskDelay(1000); } // Halt
  }
  ESP_LOGI(TAG_NET,"NET_SETUP, --- Ethernet Connection Successful ---");
  ESP_LOGI(TAG_NET,"NET_SETUP, IP Address: %s", Ethernet.localIP().toString().c_str());

  // NTP TIME SYNC
  setupNTP();

  // Get the boot timestamp and store it in our char array
  getTimestamp(boot_timestamp_str, sizeof(boot_timestamp_str));
  ESP_LOGI(TAG_MAIN, "Boot time captured: %s", boot_timestamp_str);

  // Create MQTT command queues
  app.txStringQueueHandle = xQueueCreate(MQTT_QUEUE_LENGTH, MQTT_MSG_MAX_LEN);
  if (app.txStringQueueHandle == NULL) {
    ESP_LOGE(TAG_MAIN, "setup: FAILED to create TX String queue!");
  }
  app.cmdStringQueueHandle = xQueueCreate(MQTT_QUEUE_LENGTH, MQTT_MSG_MAX_LEN);
  if (app.cmdStringQueueHandle == NULL) {
    ESP_LOGE(TAG_MAIN, "setup: FAILED to create CMD String queue!");
  }
  app.publishQueueHandle = xQueueCreate(MQTT_QUEUE_LENGTH, MQTT_MSG_MAX_LEN);
  if (app.publishQueueHandle == NULL) {
    ESP_LOGE(TAG_MAIN, "setup: FAILED to create Publish queue!");
  }

  // Initialize the CC1101 library
  if (app.cc1101.begin()) {
    ESP_LOGI(TAG_MAIN, "CC1101 initialized successfully.");
  } else {
    ESP_LOGE(TAG_MAIN, "CC1101 initialization FAILED!");
    // Set error color and halt
    LedState errorState = LED_ERROR_FLASH;
    xQueueSend(app.ledQueueHandle, &errorState, (TickType_t)0);
    while(1) { vTaskDelay(1000); } // Halt
  }

    // Create CC1101 State Queue
  app.cc1101StateQueueHandle = xQueueCreate(1, MQTT_MSG_MAX_LEN); // Only need space for 1 item
  if (app.cc1101StateQueueHandle == NULL) {
    ESP_LOGE(TAG_MAIN, "setup: FAILED to create CC1101 State queue!");
  }

  // Initialize the Evofw Protocol stack
  app.protocol.begin(RADIO_BAUD_RATE);
  ESP_LOGI(TAG_MAIN, "Evofw protocol stack initialized.");

  // --- Initialize the App Handler to get the Device ID ---
  app.app_handler.begin(GWAY_CLASS);

  // --- MQTT & ArduinoHA Setup ---
  uint8_t devClass;
  uint32_t devId;
  
  app.app_handler.device_get_id(&devClass, &devId);  // Get device class and ID from EvofwHandler
  snprintf(device_id_str, sizeof(device_id_str), "%02u:%06lu", devClass, devId);
  ESP_LOGI(TAG_MAIN, "Device ID: %s", device_id_str);
  
  // Create a version of the ID with an underscore, which is safer for HA's unique_id
  snprintf(ha_device_id_str, sizeof(ha_device_id_str), "%02u_%06lu", devClass, devId);
  ESP_LOGI(TAG_MAIN, "HA Device ID: %s", ha_device_id_str);
  
  // Create the HA objects using the device MAC address as the unique ID
  snprintf(macString, sizeof(macString), "%02X%02X%02X%02X%02X%02X",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);            // convert MAC to uppercase string
  ESP_LOGI(TAG_MAIN, "HADevice Unique ID (MAC): %s", macString);
  snprintf(macStringLower, sizeof(macStringLower), "%02x%02x%02x%02x%02x%02x",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);            // convert MAC to lowercase string
  ESP_LOGI(TAG_MAIN, "HADevice Unique ID Lowercase (MAC): %s", macStringLower);
  
  // Build topic strings
  snprintf(base_topic, sizeof(base_topic), "%s/%s", MQTT_ROOT_TOPIC, device_id_str);
  ESP_LOGI(TAG_NET, "MQTT Base Topic: %s", base_topic);
  snprintf(ha_base_topic, sizeof(ha_base_topic), "%s/%s", MQTT_ROOT_TOPIC, ha_device_id_str);
  ESP_LOGI(TAG_NET, "MQTT HA Base Topic: %s", ha_base_topic);
  snprintf(rx_topic, sizeof(rx_topic), "%s/rx", base_topic);
  ESP_LOGI(TAG_NET, "MQTT RX Topic: %s", rx_topic);
  snprintf(tx_topic, sizeof(tx_topic), "%s/tx", base_topic);
  ESP_LOGI(TAG_NET, "MQTT TX Topic: %s", tx_topic);
  snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd/cmd", base_topic);
  ESP_LOGI(TAG_NET, "MQTT CMD Topic: %s", cmd_topic);
  snprintf(cmd_result_topic, sizeof(cmd_result_topic), "%s/cmd/result", base_topic);
  ESP_LOGI(TAG_NET, "MQTT CMD Result Topic: %s", cmd_result_topic);

  // Topic format: [DataPrefix]/[DeviceID]/[SensorID]/stat_t
  snprintf(boot_topic, sizeof(boot_topic), "%s/%s/boot_time/stat_t", MQTT_ROOT_TOPIC, macStringLower);
  ESP_LOGI(TAG_MAIN, "Boot Time Topic: %s", boot_topic);

  // --- Configure ArduinoHA Device ---
  device.setUniqueId(mac, sizeof(mac));
  device.setName(macString);
  device.setManufacturer("Espressif");
  device.setModel(HARDWARE_VERSION);
  device.setSoftwareVersion(SOFTWARE_VERSION); 
  
  // --- MQTT Discovery ---
  mqtt.setDiscoveryPrefix("homeassistant");
  mqtt.setDataPrefix(MQTT_ROOT_TOPIC);

  // --- Availability reporting ---
  device.enableSharedAvailability();
  device.enableLastWill();

    // --- MQTT Callbacks ---
  mqtt.onMessage(onMqttMessage);
  mqtt.onConnected(onMqttConnected);

  // --- Configure Sensors ---
  cc1101_state_sensor.setName("CC1101 State");
  cc1101_state_sensor.setIcon("mdi:radio-tower");
  tuning_active_sensor.setName("Tuning Active");
  tuning_active_sensor.setIcon("mdi:tune");
  boot_time_sensor.setName("Uptime");
  boot_time_sensor.setDeviceClass("timestamp");
  boot_time_sensor.setIcon("mdi:clock-start");


  // --- Connect to MQTT ---
  ESP_LOGI(TAG_MAIN, "Connecting to MQTT broker...");
  if (!mqtt.begin(MQTT_SERVER, MQTT_USER, MQTT_PASS)) {
    ESP_LOGE(TAG_MAIN, "MQTT connection failed! Halting.");
    LedState errorState = LED_ERROR_FLASH;
    xQueueSend(app.ledQueueHandle, &errorState, (TickType_t)0);
    while(1) { vTaskDelay(1000); } // Halt
  }
  ESP_LOGI(TAG_MAIN, "MQTT connected successfully");

  // Create our handler object instance
  app.gateway = new GatewayHandler (
      app.protocol,               // EvofwProtocol& proto
      app.app_handler,            // EvofwHandler& handler
      Serial,                     // HardwareSerial& host
      app.ledQueueHandle,         // QueueHandle_t ledQueue
      app.txStringQueueHandle,    // QueueHandle_t txQueue
      app.cmdStringQueueHandle,   // QueueHandle_t cmdQueue
      app.publishQueueHandle,     // QueueHandle_t pubQueue
      GWAY_CLASS,                 // const uint8_t gwayClass
      MQTT_MSG_MAX_LEN,           // const uint8_t maxMsgLen
      // Use a C++ lambda to bind the NetworkHandler's member function
      [=](const char* cmd, const char* result) {
          publishCommandResult(cmd, result);
      }
  );
  app.gateway->begin(); // This initializes the app handler and prints the version
  


  // --- Task Creation ---

  // Start the LED Flashing task (now using the library)
  statusLed.startTask(1, 1);
  ESP_LOGI(TAG_MAIN, "setup: Created and pinned ledTask to Core 1.");
  
  // Create the main Gateway task
  TaskHandle_t gatewayTaskHandle = nullptr;
  BaseType_t gatewayTaskCreated = xTaskCreatePinnedToCore(
      gatewayTask,           // 1. Static function to start the task
      "Gateway Task",        // 2. Name of the task
      MAIN_TASK_STACK_SIZE,  // 3. Stack size in bytes
      &app,                  // 4. Task input parameter (pointer to our object)
      MAIN_TASK_PRIORITY,    // 5. Priority of the task
      &gatewayTaskHandle,    // 6. Task handle
      MAIN_TASK_CORE         // 7. Core to pin the task to (0)
  );

  if (gatewayTaskCreated != pdPASS || gatewayTaskHandle == nullptr) {
      ESP_LOGE(TAG_MAIN, "FATAL: Failed to create gateway Task! Halting.");
      // Send error flash and halt
      LedState errorState = LED_ERROR_FLASH;
      xQueueSend(app.ledQueueHandle, &errorState, (TickType_t)0);
      while(1) { vTaskDelay(1000); } // Halt
  } else {
    ESP_LOGI(TAG_MAIN, "setup: Created and pinned gatewayTask to Core 0.");
  }
  // Publish the boot timestamp string.
  if (boot_time_sensor.setValue(boot_timestamp_str)) {
      ESP_LOGI(TAG_MAIN, "Published boot timestamp to HA: %s", boot_timestamp_str);
  } else {
      ESP_LOGE(TAG_MAIN, "Error: Failed to publish boot timestamp to HA (MQTT buffer full or disconnected).");
  }
}

//-----------------------------------------------------------------------------
//  Loop (Core 1)
//-----------------------------------------------------------------------------
void loop() {
  // This loop runs on Core 1.
  // --- Network Watchdog Logic ---
  static unsigned long linkLostTimestamp = 0;
  const unsigned long LINK_TIMEOUT_MS = 60000; // 60 seconds
  if (Ethernet.linkStatus() == LinkOFF) {
      if (linkLostTimestamp == 0) {
          linkLostTimestamp = millis(); // Start timer
          ESP_LOGW(TAG_NET, "Network Watchdog: Ethernet Link DOWN. Starting timer...");
      } else if (millis() - linkLostTimestamp > LINK_TIMEOUT_MS) {
          ESP_LOGE(TAG_NET, "Network Watchdog: Link down > 60s. Forcing W5500 Hardware Reset!");
          
          // 1. Hold Reset LOW for 100ms
          pinMode(W5500_RST, OUTPUT);
          digitalWrite(W5500_RST, LOW);
          delay(100);
          digitalWrite(W5500_RST, HIGH);
          
          // 2. Re-initialize Ethernet
          // Note: We use the global 'mac' buffer filled in setup()
          Ethernet.begin((byte*)mac); 
          
          linkLostTimestamp = 0; // Reset timer
          ESP_LOGI(TAG_NET, "Network Watchdog: W5500 Reset Complete.");
      }
  } else {
      // Link is ON
      if (linkLostTimestamp != 0) {
          ESP_LOGI(TAG_NET, "Network Watchdog: Ethernet Link RECOVERED.");
          linkLostTimestamp = 0;
      }
      
      // Only run maintain() if link is up
      Ethernet.maintain();
      mqtt.loop();
  }
    
  // hearbeat logging every 5 seconds
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    ESP_LOGD(TAG_NET, "Network Task: Heartbeat - Running mqtt.loop()...");
  }
  
  // Update tuning active sensor state
  tuning_active_sensor.setState(app.app_handler.cc_tuneEnabled());
  
  // Check boot time is correct on MQTT Server
  verifyBootTime();
  // Check if the gateway task has a message for us to publish
  processPublishQueue();
  // Check and publish CC1101 state
  processCC1101StateQueue();
  // Yield to other tasks on this core
  vTaskDelay(10 / portTICK_PERIOD_MS);  
}

//-----------------------------------------------------------------------------
//  Task Definitions
//-----------------------------------------------------------------------------

/**
 * @brief Initializes NTP time synchronization.
 */
void setupNTP(void) {
    ESP_LOGI(TAG_NET,"NET_SETUP, Configuring NTP...");

    // Set the sync interval to 1 hour (3600000 ms) - Default is usually 1 hour, but this ensures it.
    // Note: This function must be called BEFORE configTime in some SDK versions, 
    // or can be called anytime to update the interval.
    sntp_set_sync_interval(3600000);

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    ESP_LOGI(TAG_NET,"NET_SETUP, Waiting for NTP time sync...");
    time_t now = time(nullptr);
    
    // Wait loop with timeout (don't block forever if internet is down)
    int retry = 0;
    while (now < 1577836800 && retry < 20) { // Wait until time is > the year 2020 (i.e., NTP sync is valid)
        delay(500);
        now = time(nullptr);
        retry++;
    }
    if (now < 1577836800) {
        ESP_LOGW(TAG_NET, "NET_SETUP, NTP Sync Timed Out! Time may be incorrect.");
    } else {
        ESP_LOGI(TAG_NET,"NET_SETUP, NTP time sync complete.");
        struct tm timeinfo;
        gmtime_r(&now, &timeinfo);
        ESP_LOGI(TAG_NET,"NET_SETUP, Current time: %s", asctime(&timeinfo));
    }
}

/**
 * @brief Gets the current time as an ISO 8601 formatted string (UTC)
 * and writes it into a provided buffer.
 * @param buffer The char buffer to write the timestamp into.
 * @param buffer_len The size of the buffer.
 */
void getTimestamp(char* buffer, size_t buffer_len) {
    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    
    char time_buf[20];
    struct tm timeinfo;
    gmtime_r(&now_t, &timeinfo);
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    
    auto truncated = std::chrono::system_clock::from_time_t(now_t);
    auto delta = now - truncated;
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

    // Write directly to the provided buffer
    snprintf(buffer, buffer_len, "%s.%06ld+00:00", time_buf, (long)micros);
    ESP_LOGV(TAG_NET, "getTimestamp: Generated timestamp: %s", buffer);
}

/**
 * @brief FreeRTOS task for the main gateway logic. Runs on Core 0.
 *
 * This task has three main responsibilities:
 * 1. Calls app->gateway->run() to process the Evofw protocol state machine.
 * 2. Periodically checks the CC1101 radio status.
 * 3. Implements recovery logic to reset the radio if it gets stuck
 * in a non-IDLE/RX state.
 *
 * @param pvParameters A void pointer to the global AppContext struct.
 */
void gatewayTask(void *pvParameters) {
    ESP_LOGI(TAG_GWAY, "Gateway Task: Starting on Core %d", xPortGetCoreID());

    // Variables for CC1101 state monitoring
    unsigned long last_cc1101_update = 0;                  // Timestamp for CC1101 state updates
    const unsigned long CC1101_UPDATE_INTERVAL_MS = 5000;  // Check state every 5 seconds
    char state_str_buf[32];                                // Buffer for CC1101 state string

    unsigned long stuck_state_timestamp = 0; // Tracks when we first saw a stuck state
    const unsigned long STUCK_STATE_TIMEOUT_MS = 10000; // 10 seconds (2 check intervals)

    // Cast the void pointer to our AppContext
    AppContext *app = static_cast<AppContext*>(pvParameters);

    while(1) {
        app->gateway->run(); // Call the main run loop

        // Check and send CC1101 state updates
        unsigned long now = millis();
        if (now - last_cc1101_update > CC1101_UPDATE_INTERVAL_MS) {
            last_cc1101_update = now;

            // This is now safe, as we are on Core 0
            uint8_t status_byte = cc1101.strobe(CC1100_SNOP);
            uint8_t state = CC_STATE(status_byte);

            const char* state_str = "Unknown";
            switch(state) {
                case CC_STATE_IDLE:         state_str = "Idle"; break;
                case CC_STATE_RX:           state_str = "Receiving (RX)"; break;
                case CC_STATE_TX:           state_str = "Transmitting (TX)"; break;
                case CC_STATE_FSTXON:       state_str = "Freq. Synth On"; break;
                case CC_STATE_CALIBRATE:    state_str = "Calibrating"; break;
                case CC_STATE_SETTLING:     state_str = "Settling"; break;
                case CC_STATE_RX_OVERFLOW:  state_str = "RX Overflow"; break;
                case CC_STATE_TX_UNDERFLOW: state_str = "TX Underflow"; break;
                default:                    state_str = "Other"; break;
            }

            // --- Timeout Recovery Logic ---
            bool is_stuck = false;
            // The only "good" states are IDLE and RX. Anything else is temporary.
            if (state != CC_STATE_IDLE && state != CC_STATE_RX) {
                is_stuck = true;
            }

            if (is_stuck) {
                if (stuck_state_timestamp == 0) {
                    // This is the first time we've seen a stuck state.
                    stuck_state_timestamp = now;
                    ESP_LOGW(TAG_GWAY, "Gateway Task: WARNING - CC1101 in unexpected state: 0x%02X", state);
                } else if (now - stuck_state_timestamp > STUCK_STATE_TIMEOUT_MS) {
                    // We have been stuck for over 10 seconds. Time to recover.
                    ESP_LOGE(TAG_GWAY, "Gateway Task: ERROR - CC1101 is STUCK. Forcing recovery...");
                    // Call our new public reset function.
                    // This is safe as app->protocol is on the same core.
                    app->protocol.resetToRx(); 
                    ESP_LOGI(TAG_GWAY, "Gateway Task: Recovery complete. Radio reset to RX mode.");
                    stuck_state_timestamp = 0; // Reset timer
                    state_str = "RECOVERED_TO_RX"; // Set state string for MQTT
                }
            } else if (stuck_state_timestamp != 0) {
                // We were in a stuck state, but it cleared itself. Reset the timer.
                ESP_LOGI(TAG_GWAY, "Gateway Task: INFO - CC1101 recovered on its own.");
                stuck_state_timestamp = 0;
            }

            // Send the result to the network task. Use xQueueOverwrite
            // so we don't care if the network task is busy.
            strncpy(state_str_buf, state_str, sizeof(state_str_buf)-1);
            xQueueOverwrite(app->cc1101StateQueueHandle, &state_str_buf);
        }
        // Yield to the scheduler. 1ms is very responsive.
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//-----------------------------------------------------------------------------
//  Network & MQTT Functions
//-----------------------------------------------------------------------------

void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length) {
    ESP_LOGI(TAG_NET, "MQTT Message Received. Topic: %s", topic);

    if (strcmp(topic, tx_topic) == 0) {
        // --- Handle Radio TX Packet ---
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error) {
            ESP_LOGE(TAG_NET, "deserializeJson() failed: %s", error.c_str());
            return;
        }
        
        const char* msg_str = doc["msg"];
        if (msg_str) {
            char tx_buf[MQTT_MSG_MAX_LEN]; 
            strncpy(tx_buf, msg_str, MQTT_MSG_MAX_LEN - 1); 
            tx_buf[MQTT_MSG_MAX_LEN - 1] = '\0'; 
            
            if (xQueueSend(app.txStringQueueHandle, &tx_buf, QUEUE_SEND_WAIT_TICKS) != pdPASS) {
                ESP_LOGE(TAG_NET, "Error: TX String Queue is full.");
            } else {
                ESP_LOGI(TAG_NET, "MQTT TX message queued: %s", tx_buf);
            }
        }
    }
    else if (strcmp(topic, cmd_topic) == 0) {
        // --- Handle Remote Command ---
        char cmd_buf[MQTT_MSG_MAX_LEN];
        uint16_t len_to_copy = length;
        if (len_to_copy > MQTT_MSG_MAX_LEN - 1) {
            len_to_copy = MQTT_MSG_MAX_LEN - 1;
        }
        memcpy(cmd_buf, payload, len_to_copy);
        cmd_buf[len_to_copy] = '\0'; // Ensure null-terminated
        
        if (xQueueSend(app.cmdStringQueueHandle, &cmd_buf, QUEUE_SEND_WAIT_TICKS) != pdPASS) {
            ESP_LOGE(TAG_NET, "Error: CMD String Queue is full.");
        }
    }

    // Check for Boot Time Verification
    if (strcmp(topic, boot_topic) == 0) {
        // Temporarily null-terminate the payload to compare string safely
        char temp_payload[length + 1];
        memcpy(temp_payload, payload, length);
        temp_payload[length] = '\0';

        if (strcmp(temp_payload, boot_timestamp_str) == 0) {
            g_bootTimeVerified = true;
            ESP_LOGI(TAG_MAIN, "VERIFIED: Boot timestamp confirmed by MQTT Broker!");
        }
    }
}

void onMqttConnected() {
    ESP_LOGI(TAG_NET, "onMqttConnected: Subscribing to topics...");

    // Subscribe to TX topic
    if (mqtt.subscribe(tx_topic)) {
        ESP_LOGI(TAG_NET, "Subscribed to TX topic: %s", tx_topic);
    } else {
        ESP_LOGE(TAG_NET, "Failed to subscribe to TX topic: %s", tx_topic);
    }

    // Subscribe to CMD topic
    if (mqtt.subscribe(cmd_topic)) {
        ESP_LOGI(TAG_NET, "Subscribed to CMD topic: %s", cmd_topic);
    } else {
        ESP_LOGE(TAG_NET, "Failed to subscribe to CMD topic: %s", cmd_topic);
    }
        
    // Subscribe to Boot Topic for verification
    if (mqtt.subscribe(boot_topic)) {
        ESP_LOGI(TAG_NET, "Subscribed to Boot Topic for verification: %s", boot_topic);
    } else {
        ESP_LOGE(TAG_NET, "Failed to subscribe to Boot Topic: %s", boot_topic);
    }
    
    ESP_LOGI(TAG_NET, "Availability topic: %s", device.getAvailabilityTopic());
    ESP_LOGI(TAG_NET, "Discovery prefix: %s", mqtt.getDiscoveryPrefix());
    ESP_LOGI(TAG_NET, "Data prefix: %s", mqtt.getDataPrefix());
}

void processPublishQueue() {
    char msg_to_publish[MQTT_MSG_MAX_LEN];
    if (xQueueReceive(app.publishQueueHandle, &msg_to_publish, (TickType_t)0)) {
        // We got a message (the raw evofw string).
        
        // ArduinoJson v7 automatically uses the task's stack.
        JsonDocument doc;
        
        // A buffer on the stack for the timestamp
        char timestamp_buf[40]; 
        
        // A buffer on the stack for the final JSON output
        // space for the message (255) + timestamp (40) + overhead (~20) = 384 is a safe size.
        char json_output_buf[384];

        // 1. Generate timestamp into our buffer
        getTimestamp(timestamp_buf, sizeof(timestamp_buf));

        // 2. Build the JSON document
        doc["msg"] = msg_to_publish;
        doc["ts"] = timestamp_buf; // Use the char buffer

        // 3. Serialize JSON into our output buffer
        size_t len = serializeJson(doc, json_output_buf, sizeof(json_output_buf));

        // 4. Publish the buffer
        if (len > 0) {
            mqtt.publish(rx_topic, json_output_buf, false);
        } else {
            ESP_LOGE(TAG_NET, "Failed to serialize JSON for publishing (buffer too small?).");
        }
    }
}

void processCC1101StateQueue() {
    char new_cc1101_state[MQTT_MSG_MAX_LEN];
    if (xQueueReceive(app.cc1101StateQueueHandle, &new_cc1101_state, (TickType_t)0)) {
        // We got a new state string, publish it
        cc1101_state_sensor.setValue(new_cc1101_state);
    }
}

void publishCommandResult(const char* cmd, const char* result) {
    // Use stack-allocated buffers to avoid all heap allocation.
    
    // A buffer on the stack for the final JSON output.
    // 384 bytes is more than enough for a command + result.
    char json_output_buf[384];

    // ArduinoJson v7 uses the task's stack.
    JsonDocument doc;

    // 1. Build the JSON document
    doc["cmd"] = cmd;
    
    // Clean up the result string (remove # and spaces)
    if (result && result[0] == '#') {
        result += 2; // Skip "# "
    }
    doc["return"] = result;

    // 2. Serialize JSON into our output buffer
    size_t len = serializeJson(doc, json_output_buf, sizeof(json_output_buf));

    // 3. Publish the buffer
    if (len > 0) {
        mqtt.publish(cmd_result_topic, json_output_buf, false);
    } else {
         ESP_LOGE(TAG_NET, "Failed to serialize JSON for command result.");
    }
}

void verifyBootTime() {
    // If already verified, do nothing
    if (g_bootTimeVerified) {
        return; 
    }

    // Retry every 5 seconds, but only if connected
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 5000) {
        lastRetry = millis();
        
        if (mqtt.isConnected()) {
            ESP_LOGW(TAG_MAIN, "Boot timestamp NOT verified yet. Resending...");
            
            // Force publish with RETAIN flag (true)
            // We use the manual mqtt.publish to ensure we control the retain flag directly
            mqtt.publish(boot_topic, boot_timestamp_str, true);
        }
    }
}