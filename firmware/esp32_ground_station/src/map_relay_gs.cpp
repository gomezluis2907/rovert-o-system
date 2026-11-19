// Relevo ESP32-GS: tramas ICD (0x02..0x06 y 0x77) recibidas por radio -> USB serial hacia la GUI.
#include <Arduino.h>
#include "TelProtocol.h"

bool gs_forward_map_chunk(const uint8_t *data, int len) {
    if (len <= 0 || !TelProtocol::validateFrame(data, (size_t)len)) return false;
    if (!TelProtocol::isExternalTelemetry(data[1])) return false;
    Serial.write(data, (size_t)len);
    return true;
}
