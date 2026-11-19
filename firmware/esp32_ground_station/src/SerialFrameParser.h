#pragma once

#include <Arduino.h>
#include "TelProtocol.h"

class SerialFrameParser {
public:
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 100;

    explicit SerialFrameParser(uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);

    // Alimenta un byte. Si completa una trama válida, la copia en outFrame,
    // coloca la longitud en outLen y devuelve true.
    bool feed(uint8_t byte, uint32_t nowMs, uint8_t *outFrame, size_t &outLen);

    void reset();

private:
    uint8_t buffer_[TelProtocol::MAX_FRAME_SIZE];
    size_t index_;
    uint32_t lastByteMs_;
    uint32_t timeoutMs_;

    void startWithSync(uint32_t nowMs);
};
