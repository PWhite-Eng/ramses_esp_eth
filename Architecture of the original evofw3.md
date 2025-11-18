This document describes the architecture of the original evofw3 firmware, which this ESP32 port is based on.

## **High-Level Architecture Diagram**

+--------------------------------------------------------------------------+  
|                        HOST COMPUTER (e.g., PC)                          |  
+--------+-----------------------------------------------------------------+  
       ^ |                     (Serial Text Commands)  
       | |  
       | v                     (Formatted Text Messages)  
+------+-------------------------------------------------------------------+  
| TTY (tty_usb.cpp / tty_usart.c)                                          |  
| (Handles USB or UART communication with the host)                        |  
+--------+-----------------------------------------------------------------+  
       ^ |                     (Raw bytes)  
       | |  
       | v                     (String buffers)  
+------+-------------------------------------------------------------------+  
|                        GATEWAY (gateway.c)                               |  
| (Main application logic: routes data between TTY and Message layers)     |  
+------+-------------------------------------------------------------------+  
       |                          ^                          ^  
       | (Command string)         | (Text to print)          | (Bytes to parse)  
       v                          |                          |  
+----------------------+          |                          |  
|  CMD (cmd.c)         |          |                          |  
| (Parses '!' commands |          |                          |  
| from host to query/  |          |                          |  
| configure system)    |          |                          |  
+----------------------+          |                          |  
       |                          |                          |  
       | (e.g., set freq)         | (Message obj)            | (Message obj)  
       v                          v                          |  
+------------------------------------------------------------+-------------+  
|                        MESSAGE (message.c)                               |  
| (Handles message logic: addressing, opcodes, payload, checksums.         |  
|  Manages RX/TX buffers.)                                                 |  
|  - msg_scan(): Parses host text into a message object for TX             |  
|  - msg_print(): Formats a received message object into text              |  
+--------+-----------------------------------------------------------------+  
       ^ |                     (Logical Message bytes)  
       | |  
       | v                     (Raw on-air bytes)  
+------+-------------------------------------------------------------------+  
|                         FRAME (frame.c)                                  |  
| (Handles on-air packet structure: Preamble, Sync Word, Manchester        |  
|  encoding/decoding, Trailer.)                                            |  
+--------+-----------------------------------------------------------------+  
       ^ |                     (Bytes)  
       | |  
       | v                     (Bytes)  
+------+-------------------------------------------------------------------+  
|                   UART (uart.c)                                          |  
| (Byte-level driver for the CC1101's asynchronous serial mode.            |  
|  Uses GDO0/GDO2 interrupts for FIFO and RX data.)                        |  
+--------+-----------------------------------------------------------------+  
       ^ |                     (FIFO Write commands)  
       | |  
       | v                     (FIFO Status)  
+------+-------------------------------------------------------------------+  
|                      CC1101 Driver (cc1101.c)                            |  
| (Low-level driver for the radio chip. Manages state (RX/TX/IDLE)         |  
|  and provides access to registers and FIFOs.)                            |  
+--------+-----------------------------------------------------------------+  
       ^ |                     (SPI Commands/Data)  
       | |  
       | v                     (SPI Data/Status)  
+------+-------------------------------------------------------------------+  
|                         SPI (spi.c)                                      |  
| (Hardware driver for the SPI bus)                                        |  
+--------------------------------------------------------------------------+  
       ^ |  
       | |  
       v |  
+--------------------------------------------------------------------------+  
|                      CC1101 RADIO CHIP (Hardware)                        |  
+--------------------------------------------------------------------------+

## **Supporting Modules**

These modules provide services to the main layers:

**evo.c / evofw3.ino:** The main entry point. Initializes all modules and runs the main loop, which consists of calling gateway\_work() and tty\_work().

**cc1101_param.c:** Manages loading and saving CC1101 register settings.

* Connects to: CC1101 Driver (to apply settings) and NV (to store settings).

**nv.c:** Non-Volatile (EEPROM) memory driver. Used to persist configuration.

* Connects to: cc1101_param.c.

**cc1101_tune.c:** Implements an automatic frequency tuning algorithm.

* Connects to: Gateway (to get messages for tuning) and Cmd (to set new frequency values).

**device.c:** Generates a unique device ID from the microcontroller's signature.

* Connects to: Gateway (to set the device's ID).

**config.h / *_pins.h:** Header files that define the specific hardware pins and configuration for different microcontrollers (ATmega328P vs. ATmega32U4).

**led.c:** Simple driver for the status LED.

* Connects to: evo.c (on init) and frame.c (toggles on RX/TX activity).

## **Data Flow Examples**

### **1. Host -> Radio (Transmit Message)**

1. Host sends a text command (e.g., W 12:345678 ...) over USB/Serial.  
2. **TTY** receives the raw bytes.  
3. **Gateway** gets the bytes from tty_rx_get(). Since it's not a ! command, it passes them to msg_scan().  
4. **Message** (msg_scan()) parses the text and builds a message struct with address, payload, etc.. When a full message is parsed, gateway_work() calls msg_tx_ready().  
5. **Gateway** (msg_work()) sees a message in the TX queue and calls frame_tx_start().  
6. **Frame** (frame_tx_start()) gets the logical bytes from msg_tx_byte(), Manchester encodes them, and stores the result in a buffer.  
7. **UART** (uart_tx_enable()) initiates the TX process. It fills the radio's FIFO by calling frame_tx_byte() to get the next on-air byte (preamble, sync word, or data).  
8. **CC1101 Driver** (cc_write_fifo()) takes each byte and writes it to the radio chip via SPI.  
9. **SPI** handles the low-level byte transfer to the chip.

### **2. Radio -> Host (Receive Message)**

1. **CC1101 Chip** receives a radio packet and (in async serial mode) sends data bytes out of its GDO2 pin.  
2. **UART** (rx_isr() or USART1_RX_vect) triggers on the pin edges (or UART RX interrupt), reconstructs the byte, and passes it to frame_rx_byte().  
3. **Frame** (frame_rx_byte()) processes the stream of bytes. It looks for the preamble and sync word. Once synced, it Manchester-decodes subsequent bytes. When the trailer byte is seen, it passes the complete, decoded message to the Message layer via msg_rx_end().  
4. **Message** (msg_rx_end()) validates the checksum and places the complete message struct into the rx_list.  
5. **Gateway** (gateway_work()) discovers the message with msg_rx_get().  
6. **Gateway** calls msg_print() to format the message struct back into human-readable text.  
7. **Gateway** sends the resulting string to the host using tty_put_str().  
8. **TTY** transmits the string bytes over USB/Serial to the Host.