# **ESP32-S3 Gateway Architecture**

This document describes the software architecture of the ESP32-S3 Evofw MQTT Gateway. This project is a modern C++ and FreeRTOS-based implementation inspired by the original evofw3 firmware.

## **Core Philosophy: Task & Core Separation**

The architecture is built on a key FreeRTOS principle: **separating time-critical logic from network/utility logic**. This prevents network lag or MQTT disconnections from ever causing the time-sensitive radio protocol to miss a packet.

This is achieved by pinning tasks to the ESP32-S3's dual cores:

* **Core 0 (Protocol Core):** Dedicated *exclusively* to running the gatewayTask. This task manages the GatewayHandler which in turn runs the Evofw protocol state machine and handles all interactions with the CC1101 radio. This ensures RF packet handling is never delayed by other application logic.  
* **Core 1 (Network/Utility Core):** Runs all other application tasks, including the main Arduino loop() (which functions as the network task) and the ledTask (managing the StatusLED).

All communication between tasks—especially between cores—is handled safely and asynchronously using **FreeRTOS Queues**.

## **High-Level Architecture Diagram**

           CORE 0 (Protocol)                             CORE 1 (Network/Utility)  
+-----------------------------------------+         +-----------------------------------------+  
|                                         |         |                                         |  
|  +-----------------------------------+  |         |  +-----------------------------------+  |  
|  |           gatewayTask             |  |         |  |      Arduino loop() (Core 1)      |  |  
|  | (GatewayHandler::run())           |  |         |  | (Ethernet.maintain(), mqtt.loop())|  |  
|  | +-------------------------------+ |  |         |  | +-------------------------------+ |  |  
|  | | EvofwHandler                  | |  |         |  | | HAMqtt (ArduinoHA)            | |  |  
|  | | (Commands, Tuning, DeviceID)  | |  |         |  | | (Manages MQTT & HA Discovery) | |  |  
|  | +-------------------------------+ |  |         |  | +-------------------------------+ |  |  
|  | +-------------------------------+ |  |         |  | +-------------------------------+ |  |  
|  | | EvofwProtocol                 | |  |         |  | | ledTask                       | |  |  
|  | | (Protocol State Machine,      | |  |         |  | | (StatusLED::runTask()         | |  |  
|  | |  Manchester Encoding/Decoding)| |  |         |  | +-------------------------------+ |  |  
|  | +-------------------------------+ |  |         |  |                                   |  |  
|  | +-------------------------------+ |  |         |  |                                   |  |  
|  | | CC1101_ESP32                  | |  |         |  |                                   |  |
|  | | (High-Level Radio Driver)     | |  |         |  |                                   |  |  
|  | +-------------------------------+ |  |         |  |                                   |  |  
|  | +-------------------------------+ |  |         |  |                                   |  |  
|  | | ESP_CC1101_GPIO_Interface     | |  |         |  |                                   |  |  
|  | | (Hardware SPI Driver)         | |  |         |  |                                   |  |  
|  | +-------------------------------+ |  |         |  +-----------------------------------+  |  
|  |                                   |  |         |                                         |  
+--|------------------+----------------|--+         +------------------+------------------+---+  
   |                  |                |                               |                  |  
   | (GDO0/GDO2/SPI)  |                |                               |                  |  
   v                  ^                | (Queue Send)                  | (Queue Send)     | (Queue Send)  
+-------+      +------+-------+        |                               |                  v  
| CC1101|      | Host Serial  | <------+ publishQueueHandle <----------+           ledQueueHandle  
| Radio |      | (Debug/Cmds) |        | (Radio RX -> MQTT)            |                  |  
+-------+      +--------------+        |                               |                  |  
   ^                                   | (Queue Receive)               |                  |  
   | (Queue Receive)                   |                               |                  |  
   + txStringQueueHandle <-------------+                               |                  |  
   | (MQTT TX -> Radio)                |                               |                  |  
   |                                   | (Queue Receive)               |                  |  
   + cmdStringQueueHandle <------------+                               |                  |  
     (MQTT CMD -> GWay)                |                               |                  |  
                                       | (Queue Receive)               |                  |  
     cc1101StateQueueHandle <----------+                               |                  |  
     (Radio State -> MQTT)             |                               |                  |  
                                       +-------------------------------+                  |  
                                                      (via AppContext)                    |  
                                                                                          |  
                                       +--------------------------------------------------+  
                                       |  
                                       v  
                                   +-------+  
                                   |  LED  |  
                                   +-------+

## **Component Breakdown**

### **Hardware Abstraction Libraries (lib/)**

* **ESP_CC1101_GPIO_Interface**  
  * **Purpose:** The lowest-level hardware driver for the CC1101.  
  * **Responsibilities:** Manages all SPI communication with the radio chip using the ESP-IDF's hardware SPI driver. It also directly handles the manual CS (Chip Select) pin logic and waits for the MISO pin to go low (CHIP_RDYn), as required by the CC1101.  
  * **Core:** Runs on **Core 0** (called by CC1101_ESP32).  
* **CC1101_ESP32**  
  * **Purpose:** A high-level, stateful driver for the CC1101.  
  * **Responsibilities:** Provides a clean API (e.g., enterRxMode(), readRSSI(), strobe()). It wraps the ESP_CC1101_GPIO_Interface and manages radio states (RX, TX, IDLE). It also handles saving/loading radio configuration to/from NVS (Non-Volatile Storage).  
  * **Core:** Runs on **Core 0** (called by EvofwProtocol and EvofwHandler).

### **Evofw Protocol Stack (lib/)**

* **EvofwProtocol**  
  * **Purpose:** The core protocol state machine. This is a C++ port of the original evofw3 frame.c, message.c, and uart.c.  
  * **Responsibilities:**  
    * Manages the receive state machine (listening for preamble, sync word).  
    * Performs Manchester decoding/encoding.  
    * Manages message buffers (msg_alloc, msg_free).  
    * Handles the transmit state machine.  
    * It uses Serial1 (mapped to GDO2_PIN) for asynchronous RX data from the CC1101 and the GDO0_PIN to monitor the TX FIFO.  
  * **Core:** Runs on **Core 0** (called by GatewayHandler).  
* **EvofwHandler**  
  * **Purpose:** The application logic for the Evofw protocol. This is a port of cmd.c, device.c, and cc1101_tune.c.  
  * **Responsibilities:**  
    * Parses and executes internal commands (e.g., !V, !T, !F).  
    * Manages the device ID generation.  
    * Implements the CC1101 frequency tuning state machine.  
  * **Core:** Runs on **Core 0** (called by GatewayHandler).

### **Application Logic**

* **src/main.cpp**  
  * **Purpose:** Main application entry point.  
  * **Responsibilities:**  
    * Contains the setup() and loop() functions.  
    * **setup() (Core 1):** Initializes all hardware (Ethernet, W5500, NTP, CC1101, StatusLED), FreeRTOS queues, MQTT, and Home Assistant (ArduinoHA) objects. Creates and starts the gatewayTask and ledTask.  
    * **loop() (Core 1):** Functions as the main **network task**. It continuously calls Ethernet.maintain() and mqtt.loop() to keep the network connection alive. It also processes outgoing messages from publishQueueHandle and cc1101StateQueueHandle.  
    * Defines the AppContext struct to safely pass all objects and queue handles to the tasks.  
    * Defines the MQTT message callbacks (onMqttMessage, onMqttConnected).  
* **GatewayHandler (lib/GatewayHandler)**  
  * **Purpose:** The "brain" of the application, running within gatewayTask.  
  * **Responsibilities:**  
    * Calls _protocol.loop() to run the protocol state machine.  
    * Checks all incoming queues (txStringQueueHandle, cmdStringQueueHandle) for work from the network.  
    * If a radio packet is received (from _protocol.msg_rx_get()), it formats it and sends it to the publishQueueHandle.  
    * If a command response is generated (from _handler), it sends it to the publishCommandResult callback.  
    * Calls _handler.loop() to run the command and tuning logic.  
  * **Core:** Runs on **Core 0**.  
* **gatewayTask (defined in src/main.cpp)**  
  * **Purpose:** The time-critical radio and protocol task.  
  * **Responsibilities:**  
    * Calls app->gateway->run() in a tight loop.  
    * Periodically checks the CC1101's MARCSTATE register to monitor its health (the "radio watchdog").  
    * Implements the "stuck radio" recovery logic: if the radio is not in IDLE or RX for too long, it calls app->protocol.resetToRx().  
    * Sends the current radio state to the cc1101StateQueueHandle.  
  * **Core:** Runs on **Core 0**.

### **Utility Modules**

* **StatusLED (lib/StatusLED)**  
  * **Purpose:** A non-blocking driver for the WS2812 "NeoPixel" LED.  
  * **Responsibilities:** Runs its own FreeRTOS task (ledTask) on **Core 1**. It listens on the ledQueueHandle for commands (e.g., LED_RX_FLASH, LED_HEARTBEAT_ON) and manages the LED's color and timing without blocking any other code.
  
## **Data Flow Examples (via RTOS Queues)**

### **1. Radio Packet (RF -> MQTT)**

1. **CC1101** (Hardware) receives a packet.  
2. EvofwProtocol (Core 0) detects and decodes the packet, creating a message struct.  
3. GatewayHandler (Core 0) retrieves the struct via _protocol.msg_rx_get().  
4. GatewayHandler formats a string and sends it to publishQueueHandle.  
5. loop() (Core 1) receives the string from publishQueueHandle.  
6. loop() (Core 1) calls mqtt.publish() to send the string to the MQTT broker.  
7. GatewayHandler (Core 0) also sends LED_RX_FLASH to ledQueueHandle.  
8. ledTask (Core 1) receives the command and flashes the LED.

### **2. MQTT Transmit (MQTT -> RF)**

1. loop() (Core 1) receives an MQTT message on the tx_topic.  
2. onMqttMessage() (Core 1) parses the JSON and sends the packet string to txStringQueueHandle.  
3. GatewayHandler (Core 0) receives the string from txStringQueueHandle.  
4. GatewayHandler allocates a message struct and uses _protocol.msg_scan() to parse the string into it.  
5. GatewayHandler queues the message for transmit via _protocol.msg_tx_ready().  
6. EvofwProtocol (Core 0) state machine picks up the TX message, enables the transmitter, and sends the packet.  
7. GatewayHandler (Core 0) also sends LED_TX_FLASH to ledQueueHandle.  
8. ledTask (Core 1) flashes the LED.