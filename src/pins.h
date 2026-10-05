#pragma once

// ============================================================================
// WAVESHARE ESP32-S3-TOUCH-LCD-4.3B HARDWARE PINOUT MAP (DO NOT ALTER)
// ============================================================================

// Waveshare RS485 interface UART pins
// Waveshare RS485 interface: GPIO43 = RS485_RXD (data input), GPIO44 = RS485_TXD (data output)
#define UART_RX_PIN GPIO_NUM_43
#define UART_TX_PIN GPIO_NUM_44

// --- I2C BUS (PCF8563 RTC & GT911 Touch Controller) ---
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9

// --- SD CARD (SDMMC 1-Bit / SPI Mode) ---
//#define PIN_SD_CLK          14
//#define PIN_SD_CMD          11
//#define PIN_SD_D0           13

// --- SD CARD (SPI; card select is CH422G EXIO4) ---
#define PIN_SD_CLK          12
#define PIN_SD_MOSI         11
#define PIN_SD_MISO         13
#define PIN_SD_CS_EXPANDER  4
#define PIN_SD_DUMMY_CS     6

// --- LCD RGB INTERFACE (ST7701) & BACKLIGHT ---
#define PIN_LCD_BL          2
// (Display RGB data lines DE, VSYNC, HSYNC, PCLK are hardware mapped)