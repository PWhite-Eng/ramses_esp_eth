# **ESP32-S3 Evofw MQTT Ethernet Gateway**

This project provides a high-performance, wired Ethernet gateway for the Evohome/Ramses II (Evofw) protocol. It uses an ESP32-S3, a W5500 Ethernet module, and a CC1101 radio transceiver to bridge 868MHz RF packets to and from an MQTT broker.

It is designed for reliability, using a wired connection to avoid Wi-Fi issues and FreeRTOS tasks to separate network-handling logic from time-sensitive radio protocol logic. It also includes Home Assistant auto-discovery for seamless integration.

## **A Note on Wi-Fi / Serial Alternatives**

Please note: This project is exclusively designed for **wired Ethernet** and does not support Wi-Fi or a direct serial (USB) data output.

If you are looking for a solution that provides a serial connection or uses Wi-Fi, this is not the project for you. For that functionality, please see the excellent **ramses_esp** project:

* **https://github.com/IndaloTech/ramses_esp.git**

![Wiring Photo](docs/wiring_photo.jpg)

The "why" behind this project's design was specific:

1. To prioritize **very high reliability** which is best achieved with a wired Ethernet connection.  
2. To create a "plug and play" device: compile/build once with the user configuration variables, upload, and then just plug it into ethernet and power and not worry about it. It just does its job.  
3. To serve as a challenging project to learn the details of the ESP32 and FreeRTOS (coming from an Arduino background). It certainly provided an education!

## **Features**

* **Wired Ethernet:** Uses a W5500 module for a stable, reliable network connection.  
* **High-Performance Radio:** Employs the evofw protocol stack (ported from C) for full-duplex communication with Evohome/Ramses II devices.  
* **RTOS-Based:** Built on FreeRTOS, dedicating **Core 0** to radio and protocol logic and **Core 1** to network and LED management, ensuring no packets are missed.  
* **Home Assistant Integration:** Automatically creates a device in Home Assistant via MQTT auto-discovery, including a sensor for radio status.  
* **MQTT Command & Control:** All RF packets are published to MQTT, and packets can be sent via MQTT. The gateway's internal commands (e.g., frequency tuning, version) are also exposed.  
* **Robust & Self-Recovering:** Includes software watchdogs that monitor both the CC1101 radio and network connections, automatically recovering from a hardware hang or network drop.  
* **Persistent Configuration:** All CC1101 radio tuning parameters are saved to the ESP32's Non-Volatile Storage (NVS).

## **Design Philosophy & Robustness**

This project is built with a specific "hybrid" design philosophy to maximize stability and performance.

* **SPI Driver:** The custom ESP_CC1011_GPIO_Interface library uses the ESP-IDF's spi_master.h driver for hardware-accelerated SPI. It also provides precise, low-level GPIO control to manually assert Chip Select (CS) and poll the MISO line, which is necessary to detect when the CC1101 pulls the line low to signal it is ready (CHIP_RDYn).  
3. **Dual-Core Task Separation:** To guarantee reliability, the ESP32's dual cores are given separate responsibilities:  
   * **Core 0 (Protocol Core):** Runs the gatewayTask, which is dedicated *only* to managing the Evofw protocol and radio. This ensures that network lag can *never* cause a radio packet to be missed.  
   * **Core 1 (Network Core):** Runs the main Arduino loop() (as its own FreeRTOS task) and the ledTask. The loop() function is responsible for all network (Ethernet and MQTT) handling.  
4. **Self-Healing Recovery:**  
   * **Network/MQTT:** The main loop() task (on Core 1) continuously runs Ethernet.maintain() and mqtt.loop(). These functions automatically handle re-establishing an IP address if the cable is re-plugged and automatically reconnecting to the MQTT broker if the connection is lost.  
   * **Radio Hardware:** The gatewayTask includes a "radio watchdog." It periodically checks the CC1101's status. If the chip hangs or becomes unresponsive for more than 10 seconds, the gateway automatically executes a full-stack reset of the radio protocol, purging all queues and re-initializing the chip to a clean state.

## **Hardware Requirements**

* **ESP32-S3:** Developed on an esp32-s3-devkitc-1, but other ESP32-S3 boards should work. The photo shows it working on a Waveshare [ESP32-S3-ETH](https://www.waveshare.com/esp32-s3-eth.htm) with builtin W5500 Ethernet module.  
* **W5500 Module:** Any standard W5500-based Ethernet module.  
* **CC1101 Module:** A CC1101 868MHz radio transceiver module (e.g., from Ebyte or a generic module).  
* **Status LED (Optional):** A WS2812 "NeoPixel" RGB LED (this is built-in on the Waveshare ESP32-S3-ETH board, connected to GPIO 21).

### **Default Pinout**

The hardware pinout is defined in include/config_pins.h. If your wiring is different, you **must** update this file.

| Module | Pin | ESP32-S3 Pin | Purpose |
| :---- | :---- | :---- | :---- |
| **Status LED** | DIN | GPIO 21 | WS2812 "NeoPixel" |
|  |  |  |  |
| **CC1101 (SPI)** | SCLK | GPIO 42 | SPI Clock |
|  | MISO | GPIO 45 | SPI MISO |
|  | MOSI | GPIO 41 | SPI MOSI |
|  | CSN | GPIO 46 | SPI Chip Select |
|  | GDO0 | GPIO 47 | TX FIFO Interrupt |
|  | GDO2 | GPIO 48 | RX Async Data (to Serial1) |
|  |  |  |  |
| **W5500 (HSPI)** | SCLK | GPIO 13 | SPI Clock |
|  | MISO | GPIO 12 | SPI MISO |
|  | MOSI | GPIO 11 | SPI MOSI |
|  | CS | GPIO 14 | SPI Chip Select |
|  | RST | GPIO 9 | W5500 Reset |
|  | INT | GPIO 10 | W5500 Interrupt |

### **Status LED Meanings**

The on-board WS2812 "NeoPixel" (GPIO 48) provides a visual indication of the gateway's status.

| Color | Pattern | Meaning |
| :---- | :---- | :---- |
| **Blue** | Pulsing (On/Off) | Heartbeat. The application is running normally. |
| **Green** | Single Flash | A radio packet was successfully received (RX). |
| **Yellow** | Single Flash | A radio packet was successfully transmitted (TX). |
| **Red** | Single Flash | An error occurred (e.g., invalid radio packet, or a fatal error during setup). |

## **Software Requirements**

* **PlatformIO:** This project is designed to be built using [PlatformIO](https://platformio.org/), typically used as an extension within [Visual Studio Code](https://code.visualstudio.com/).  
* **MQTT Broker:** A running MQTT broker (e.g., Mosquitto, HA Add-on).  
* **Libraries:** All required libraries are defined in platformio.ini and will be **automatically installed by PlatformIO** when you build the project. There is no need to install them manually.

## **Installation & User Configuration**

This project is designed to be "clone and go." All user settings are stored in two files, which you must configure.

1. **Clone the Repository:**  
   git clone [https://github.com/PWhite-Eng/ramses_esp_eth.git](https://github.com/PWhite-Eng/ramses_esp_eth.git)  
   cd your-repo-name

2. **Configure Your Secrets:**  
   * Find the include/config_secrets.h.example file.  
   * **Rename or copy** this file to include/config_secrets.h.  
   * Open include/config_secrets.h and edit the settings (MQTT server IP, username, password) to match your local network. This file is ignored by git, so your secrets are safe.  
3. **Configure Your Pins (If Necessary):**  
   * Open include/config_pins.h.  
   * **If your wiring is different from the default pinout table above**, update the GPIO pin numbers in this file to match your hardware setup. If your wiring matches, you can skip this step.  
4. **Build and Upload:**  
   * Open the project in Visual Studio Code with the PlatformIO extension.  
   * Use the PlatformIO "Upload and Monitor" command (or run platformio run --target upload --target monitor from the terminal).

## **Usage: MQTT Topics**

The gateway uses a unique device ID (e.g., 18:109300) which is based on its class (18) and a unique ID from the ESP32. This ID is printed to the serial console on boot and is used in all MQTT topics.

* **Base Topic:** RAMSES/GATEWAY/<device-id>  
* **Device ID Example:** 18:109300

### **Radio RX (Gateway -> MQTT)**

All valid RF packets (both sent and received) are published here as JSON.

* **Topic:** RAMSES/GATEWAY/18:109300/rx  
* **Payload (Example):**  
```
  {  
    "msg": "060 --- 10:052018 18:109300 3150 001 00",  
    "ts": "2025-10-31T18:06:00.123456+00:00"  
  }
```
### **Radio TX (MQTT -> Gateway)**

Publish a JSON payload to this topic to transmit an RF packet.

* **Topic:** RAMSES/GATEWAY/18:109300/tx  
* **Payload (Example):**  
```
  {  
    "msg": "RQ --- 18:109300 01:050000 --:------ 1F09 001 00"  
  }
```
### **Gateway Command (MQTT -> Gateway)**

Send internal commands to the gateway firmware.

* **Topic:** RAMSES/GATEWAY/18:109300/cmd/cmd  
* **Payload:** A raw string (not JSON) representing the command (e.g., V, T 01, F).  
* **Common Commands:**  
  * V: Get firmware version.  
  * I: Get device ID.  
  * T 01: Enable TRC_RAW (raw packet debugging) trace level.  
  * T 00: Disable trace.  
  * F: Get current CC1101 frequency.  
  * FS: Save current frequency to NVS.  
  * FR: Reset frequency from NVS.

### **Command Result (Gateway -> MQTT)**

The gateway's response to any command is published here.

* **Topic:** RAMSES/GATEWAY/18:109300/cmd/result  
* **Payload (Example):**  
```
  {  
    "cmd": "V",  
    "return": "# !V evofw4-esp32 0.0.1"  
  }
```
## **Development & Debugging**

### **Serial Monitor (Host)**

The primary method for debugging is the USB serial port.

* **Baud Rate:** 115200  
* This port acts as the "host" serial, printing all log messages (via ESP_LOG) and allowing you to send ! commands (e.g., !V, !T 01).

### **Logging System**

The project uses the ESP_LOG framework for all console output. This allows for fine-grained control over log verbosity.

* **Log Tags:** Each message is prefixed with a TAG (e.g., [Main], [Network], [Gateway]) defined in include/log_config.h.  
* **Log Level:** The verbosity of the ESP-IDF logging framework can be controlled from the platformio.ini file:  
  * build_flags = -DCORE_DEBUG_LEVEL=3 (Info)  
  * build_flags = -DCORE_DEBUG_LEVEL=4 (Debug)  
  * build_flags = -DCORE_DEBUG_LEVEL=5 (Verbose)

## **Architecture**

This project includes two architecture documents:

* **Architecture of ESP32-S3 Gateway.md**: A detailed breakdown of this ESP32 gateway's modern, RTOS-based design.  
* **Architecture of the original evofw3.md**: The architecture of the original C-based evofw3 firmware, which serves as a valuable reference for the protocol logic this project was based on.

## **Acknowledgements**

This project is a port of the original evofw3 firmware and would not have been possible without the amazing reverse-engineering and protocol implementation work done by **@ghoti57**.

The original evofw3 project can be found at [https://github.com/ghoti57/evofw3.git](https://github.com/ghoti57/evofw3.git).

## **License**

This project is licensed under the **MIT License**.