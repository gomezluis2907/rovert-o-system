#pragma once
#include <Arduino.h>

// ===============================
// CONFIGURACIÓN ESP32 DEL ROVER
// ===============================

// MAC Wi-Fi STA real de la ESP32 Ground Station entregada por Telecomunicaciones.
static const uint8_t GROUND_STATION_MAC[6] = {
    0x8C, 0x94, 0xDF, 0x70, 0x64, 0x34
};

// Ambos nodos ESP-NOW DEBEN usar el mismo canal.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

// Pines UART1 hacia la Raspberry Pi 4:
// - En ESP32 WROOM (esp32dev): RX = GPIO 34 (input-only, libre), TX = GPIO 23 (libre).
//   Nota: GPIO 16 y 17 están ocupados por Motor fr en main.cpp.
// - En ESP32-S3 (esp32s3):     RX = GPIO 1,  TX = GPIO 2.
#if CONFIG_IDF_TARGET_ESP32S3
constexpr int PI_UART_RX_PIN = 1;  // ESP32 RX <- Raspberry Pi TX (GPIO 14 / Pin 8)
constexpr int PI_UART_TX_PIN = 2;  // ESP32 TX -> Raspberry Pi RX (GPIO 15 / Pin 10)
constexpr int I2C_SDA_PIN    = 8;  // El bus I2C compartido usa este pin de datos
constexpr int I2C_SCL_PIN    = 9;  // El bus I2C compartido usa este pin de reloj
#else
constexpr int PI_UART_RX_PIN = 34; // ESP32 RX <- Raspberry Pi TX (GPIO 14 / Pin 8)
constexpr int PI_UART_TX_PIN = 23; // ESP32 TX -> Raspberry Pi RX (GPIO 15 / Pin 10)
constexpr int I2C_SDA_PIN    = 15; // El bus I2C compartido en WROOM usa este pin de datos
constexpr int I2C_SCL_PIN    = 5;  // El bus I2C compartido en WROOM usa este pin de reloj
#endif
constexpr uint32_t PI_UART_BAUD = 115200;

// Timeout de pérdida de enlace (docs/ICD.md: > 2 s -> STANDBY).
constexpr uint32_t LINK_TIMEOUT_MS = 2000;

// Cola local para sacar procesamiento del callback Wi-Fi de ESP-NOW.
constexpr uint8_t RADIO_RX_QUEUE_DEPTH = 16;
