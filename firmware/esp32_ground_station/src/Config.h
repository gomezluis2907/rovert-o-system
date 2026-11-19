#pragma once
#include <Arduino.h>

// ======================================
// CONFIGURACIÓN ESP32 GROUND STATION
// ======================================

// MAC Wi-Fi STA real de la ESP32-Onboard (Rover) entregada por Telecomunicaciones.
static const uint8_t ROVER_MAC[6] = {
    0x58, 0x2A, 0xBD, 0x71, 0xE5, 0xE4
};

// Ambos nodos ESP-NOW DEBEN usar el mismo canal.
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

// En Ground Station, Serial (USB) es el enlace binario con la laptop (GUI/main.py).
constexpr uint32_t LAPTOP_SERIAL_BAUD = 115200;

// IMPORTANTE: debe permanecer false en operación normal para no contaminar
// las tramas binarias hacia la GUI con texto de depuración.
constexpr bool ENABLE_DEBUG = false;

constexpr uint8_t RADIO_RX_QUEUE_DEPTH = 16;
