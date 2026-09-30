#pragma once

// ============================================================================
// WAVESHARE ESP32-S3-TOUCH-LCD-4.3B HARDWARE PINOUT MAP
// ============================================================================

// --- CAN BUS (External SN65HVD230 Transceiver Bypass) ---
// Remapped from damaged onboard TJA1051 to free IO pads:
// - PIN_CAN_TX (1)  -> Wired to IO1 Pad -> CTX on module
// - PIN_CAN_RX (46) -> Wired to IO46 Pad -> CRX on module
// (Frees GPIO 20 so Native USB CDC stays continuously connected)
#define PIN_CAN_TX          1
#define PIN_CAN_RX          46

// --- I2C BUS (PCF8563 RTC & GT911 Touch Controller) ---
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9

// --- SD CARD (SPI; card select is CH422G EXIO4) ---
#define PIN_SD_CLK          12
#define PIN_SD_MOSI         11
#define PIN_SD_MISO         13
#define PIN_SD_CS_EXPANDER  4
#define PIN_SD_DUMMY_CS     6

// --- LCD RGB INTERFACE (ST7701) & BACKLIGHT ---
#define PIN_LCD_BL          2
// (Display RGB data lines DE, VSYNC, HSYNC, PCLK are hardware mapped)