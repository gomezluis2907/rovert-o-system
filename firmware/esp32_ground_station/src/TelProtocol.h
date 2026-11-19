#pragma once

#include <Arduino.h>

namespace TelProtocol {

constexpr uint8_t SYNC_BYTE = 0xAA;

// Mensajes externos TEL (docs/ICD.md)
constexpr uint8_t MSG_COMMAND    = 0x01;
constexpr uint8_t MSG_NAV        = 0x02;
constexpr uint8_t MSG_POWER      = 0x03;
constexpr uint8_t MSG_THERMAL    = 0x04;
constexpr uint8_t MSG_HEALTH     = 0x05;
constexpr uint8_t MSG_MAP        = 0x06;
constexpr uint8_t MSG_MAP_LEGACY = 0x77;

// Mensajes internos Raspberry Pi <-> ESP32-Onboard (docs/ICD.md)
constexpr uint8_t MSG_DRIVE_STATE  = 0x66;
constexpr uint8_t MSG_PI_TELEMETRY = 0x90;
constexpr uint8_t MSG_ENCODERS     = 0x55;

constexpr size_t LEN_COMMAND      = 13; // <BBBBffB
constexpr size_t LEN_NAV          = 36; // <BBBIfffffffB
constexpr size_t LEN_POWER        = 17; // <BBBIffBB
constexpr size_t LEN_THERMAL      = 21; // <BBBIfffBB
constexpr size_t LEN_HEALTH       = 15; // <BBBIBBBBHBB (incluye uint16 rpi_ram_mb)
constexpr size_t LEN_DRIVE_STATE  = 4;  // <BBBB
constexpr size_t LEN_PI_TELEMETRY = 12; // <BBfBBHBB (incluye uint16 rpi_ram_mb)
constexpr size_t LEN_ENCODERS     = 12; // <BBiiBB

constexpr uint8_t MAP_MIN_PAYLOAD = 1;
constexpr uint8_t MAP_MAX_PAYLOAD = 198;
constexpr size_t MAP_HEADER_BYTES = 6;  // sync,id,map_seq_id,chunk_id,total_chunks,payload_len
constexpr size_t MAP_TRAILER_BYTES = 1; // xor
constexpr size_t MAX_FRAME_SIZE = MAP_HEADER_BYTES + MAP_MAX_PAYLOAD + MAP_TRAILER_BYTES; // 205 B

bool isKnownId(uint8_t id);
bool isExternalTelemetry(uint8_t id);
bool isInternalMessage(uint8_t id);

// 0 = longitud variable o ID desconocido.
size_t fixedFrameLength(uint8_t id);

// Devuelve la longitud total esperada cuando ya es posible determinarla.
// 0 = todavía faltan bytes o el ID es desconocido.
// SIZE_MAX = trama inválida.
size_t expectedFrameLength(const uint8_t *data, size_t received);

uint8_t xorChecksum(const uint8_t *data, size_t lenWithoutChecksum);
bool validateFrame(const uint8_t *data, size_t len);

} // namespace TelProtocol
