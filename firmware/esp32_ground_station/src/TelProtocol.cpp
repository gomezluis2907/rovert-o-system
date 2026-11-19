#include "TelProtocol.h"

namespace TelProtocol {

bool isKnownId(uint8_t id) {
    switch (id) {
        case MSG_COMMAND:
        case MSG_NAV:
        case MSG_POWER:
        case MSG_THERMAL:
        case MSG_HEALTH:
        case MSG_MAP:
        case MSG_MAP_LEGACY:
        case MSG_DRIVE_STATE:
        case MSG_PI_TELEMETRY:
        case MSG_ENCODERS:
            return true;
        default:
            return false;
    }
}

bool isExternalTelemetry(uint8_t id) {
    return (id >= MSG_NAV && id <= MSG_MAP) || (id == MSG_MAP_LEGACY);
}

bool isInternalMessage(uint8_t id) {
    return id == MSG_DRIVE_STATE ||
           id == MSG_PI_TELEMETRY ||
           id == MSG_ENCODERS;
}

size_t fixedFrameLength(uint8_t id) {
    switch (id) {
        case MSG_COMMAND:      return LEN_COMMAND;
        case MSG_NAV:          return LEN_NAV;
        case MSG_POWER:        return LEN_POWER;
        case MSG_THERMAL:      return LEN_THERMAL;
        case MSG_HEALTH:       return LEN_HEALTH;
        case MSG_DRIVE_STATE:  return LEN_DRIVE_STATE;
        case MSG_PI_TELEMETRY: return LEN_PI_TELEMETRY;
        case MSG_ENCODERS:     return LEN_ENCODERS;
        case MSG_MAP:
        case MSG_MAP_LEGACY:   return 0; // variable
        default:               return 0;
    }
}

size_t expectedFrameLength(const uint8_t *data, size_t received) {
    if (data == nullptr || received == 0) {
        return 0;
    }

    if (data[0] != SYNC_BYTE) {
        return SIZE_MAX;
    }

    if (received < 2) {
        return 0;
    }

    const uint8_t id = data[1];
    if (!isKnownId(id)) {
        return SIZE_MAX;
    }

    if (id != MSG_MAP && id != MSG_MAP_LEGACY) {
        return fixedFrameLength(id);
    }

    // MAP: necesitamos llegar hasta payload_len (byte 5).
    if (received < MAP_HEADER_BYTES) {
        return 0;
    }

    const uint8_t payloadLen = data[5];
    if (payloadLen < MAP_MIN_PAYLOAD || payloadLen > MAP_MAX_PAYLOAD) {
        return SIZE_MAX;
    }

    return MAP_HEADER_BYTES + payloadLen + MAP_TRAILER_BYTES;
}

uint8_t xorChecksum(const uint8_t *data, size_t lenWithoutChecksum) {
    uint8_t value = 0;
    for (size_t i = 0; i < lenWithoutChecksum; ++i) {
        value ^= data[i];
    }
    return value;
}

bool validateFrame(const uint8_t *data, size_t len) {
    if (data == nullptr || len < 3 || len > MAX_FRAME_SIZE) {
        return false;
    }

    if (data[0] != SYNC_BYTE) {
        return false;
    }

    const size_t expected = expectedFrameLength(data, len);
    if (expected == 0 || expected == SIZE_MAX || expected != len) {
        return false;
    }

    return xorChecksum(data, len - 1) == data[len - 1];
}

} // namespace TelProtocol
