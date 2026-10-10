// Pins of the 2.8" ESP32-32E display board (found with the probe firmware in probe/)
#pragma once

// Display: ILI9341 on the HSPI bus, set in platformio.ini (SCK 14, MOSI 13, MISO 12, CS 15, DC 2)
#define PIN_BACKLIGHT 21

// Resistive touch controller XPT2046, on pins of its own
#define PIN_TOUCH_CLK 25
#define PIN_TOUCH_MOSI 32
#define PIN_TOUCH_MISO 39
#define PIN_TOUCH_CS 33
#define PIN_TOUCH_IRQ 36

// SD card on the VSPI bus; the "SPI" connector carries the same bus plus IO27
#define PIN_SD_SCK 18
#define PIN_SD_MISO 19
#define PIN_SD_MOSI 23
#define PIN_SD_CS 5
#define PIN_LORA_CS 27      // radio module (RFM95W), when fitted: must be held high otherwise

// GPS module: only its TX is wired, to this input-only pin
#define PIN_GPS_RX 35

// Speaker amplifier
#define PIN_AUDIO 26
#define PIN_AUDIO_ENABLE 4  // LOW = amplifier on

// Battery voltage through a 1:2 divider
#define PIN_BATTERY 34

// RGB LED, active low
#define PIN_LED_R 22
#define PIN_LED_G 16
#define PIN_LED_B 17
