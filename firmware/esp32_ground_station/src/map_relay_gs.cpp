// Relevo ESP32-GS: Packet 3 recibido por radio -> USB serial hacia la GUI.
#include <Arduino.h>

// TODO (Telecom): llamar a esta función dentro de OnDataRecv cuando llegue un frame Packet 3
bool gs_forward_map_chunk(const uint8_t *data, int len) {
    // Validar cabecera 0xAA 0x77, largo y checksum XOR antes de reenviar
    if (len < 7 || data[0] != 0xAA || data[1] != 0x77 || len != 6 + data[5] + 1) return false;
    uint8_t x = 0;
    for (int i = 0; i < len - 1; i++) x ^= data[i];
    if (x != data[len - 1]) return false;
    Serial.write(data, len);
    return true;
}
