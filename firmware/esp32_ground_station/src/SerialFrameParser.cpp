#include "SerialFrameParser.h"
#include <string.h>

SerialFrameParser::SerialFrameParser(uint32_t timeoutMs)
    : index_(0), lastByteMs_(0), timeoutMs_(timeoutMs) {}

void SerialFrameParser::reset() {
    index_ = 0;
    lastByteMs_ = 0;
}

void SerialFrameParser::startWithSync(uint32_t nowMs) {
    buffer_[0] = TelProtocol::SYNC_BYTE;
    index_ = 1;
    lastByteMs_ = nowMs;
}

bool SerialFrameParser::feed(uint8_t byte, uint32_t nowMs,
                             uint8_t *outFrame, size_t &outLen) {
    outLen = 0;

    // Si una trama quedó incompleta demasiado tiempo, resincroniza.
    if (index_ > 0 && (int32_t)(nowMs - lastByteMs_) > (int32_t)timeoutMs_) {
        reset();
    }

    if (index_ == 0) {
        if (byte == TelProtocol::SYNC_BYTE) {
            startWithSync(nowMs);
        }
        return false;
    }

    if (index_ >= sizeof(buffer_)) {
        reset();
        if (byte == TelProtocol::SYNC_BYTE) {
            startWithSync(nowMs);
        }
        return false;
    }

    buffer_[index_++] = byte;
    lastByteMs_ = nowMs;

    const size_t expected = TelProtocol::expectedFrameLength(buffer_, index_);

    if (expected == SIZE_MAX) {
        reset();
        if (byte == TelProtocol::SYNC_BYTE) {
            startWithSync(nowMs);
        }
        return false;
    }

    if (expected == 0) {
        return false;
    }

    if (index_ > expected) {
        reset();
        return false;
    }

    if (index_ < expected) {
        return false;
    }

    // index_ == expected
    if (!TelProtocol::validateFrame(buffer_, index_)) {
        reset();
        return false;
    }

    memcpy(outFrame, buffer_, index_);
    outLen = index_;
    reset();
    return true;
}
