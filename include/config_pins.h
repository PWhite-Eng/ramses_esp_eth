#pragma once

// --- Hardware Pin Definitions ---
// These pins are specific to your PCB layout.

// --- LED Pin ---
static constexpr int LED_PIN = 21;

// --- CC1101 (Radio) SPI Pins ---
static constexpr int CC1101_GDO2 = 48;  // GDO2 pin (RX Async Data)
static constexpr int CC1101_GDO0 = 47;  // GDO0 pin (TX FIFO interrupt)
static constexpr int CC1101_CSN  = 46;  // CSN (SS) pin
static constexpr int CC1101_MISO = 45;  // MISO pin
static constexpr int CC1101_SCLK = 42;  // SCLK pin
static constexpr int CC1101_MOSI = 41;  // MOSI pin

// --- W5500 (Ethernet) SPI Pins (HSPI bus) ---
static constexpr int W5500_SCLK = 13;
static constexpr int W5500_MISO = 12;
static constexpr int W5500_MOSI = 11;
static constexpr int W5500_CS   = 14;
static constexpr int W5500_RST  = 9;
static constexpr int W5500_INT  = 10;