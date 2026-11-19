#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "Config.h"
#include "TelProtocol.h"
#include "SerialFrameParser.h"

// Serial = canal binario Laptop (GUI/main.py) <-> ESP32 Ground Station.
SerialFrameParser laptopParser;

struct RadioPacket {
    uint16_t len;
    uint8_t srcMac[6];
    uint8_t data[250];
};

QueueHandle_t radioRxQueue = nullptr;

uint32_t radioRxValid = 0;
uint32_t radioRxInvalid = 0;
uint32_t radioTxRequested = 0;
uint32_t laptopRxValid = 0;

static bool macEquals(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

static bool peerMacConfigured(const uint8_t *mac) {
    for (int i = 0; i < 6; ++i) {
        if (mac[i] != 0x00) return true;
    }
    return false;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowReceive(const esp_now_recv_info_t *info,
                     const uint8_t *data,
                     int len) {
    const uint8_t *srcMac = (info != nullptr) ? info->src_addr : nullptr;
#else
void onEspNowReceive(const uint8_t *srcMac,
                     const uint8_t *data,
                     int len) {
#endif
    if (srcMac == nullptr || data == nullptr || len <= 0 || len > 250) {
        return;
    }

    RadioPacket packet{};
    packet.len = static_cast<uint16_t>(len);
    memcpy(packet.srcMac, srcMac, 6);
    memcpy(packet.data, data, len);

    // Callback Wi-Fi: no hacer Serial.write() aquí.
    if (radioRxQueue != nullptr) {
        xQueueSend(radioRxQueue, &packet, 0);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    (void)info;
    (void)status;
}
#else
void onEspNowSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    (void)macAddr;
    (void)status;
}
#endif

bool initEspNow() {
    WiFi.mode(WIFI_STA);
    delay(100);

    if (esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_register_recv_cb(onEspNowReceive);
    esp_now_register_send_cb(onEspNowSent);

    if (!peerMacConfigured(ROVER_MAC)) {
        return false;
    }

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, ROVER_MAC, 6);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(ROVER_MAC)) {
        if (esp_now_add_peer(&peer) != ESP_OK) {
            return false;
        }
    }

    return true;
}

bool sendCommandToRover(const uint8_t *frame, size_t len) {
    if (!TelProtocol::validateFrame(frame, len)) {
        return false;
    }

    if (frame[1] != TelProtocol::MSG_COMMAND) {
        return false;
    }

    ++radioTxRequested;
    return esp_now_send(ROVER_MAC, frame, len) == ESP_OK;
}

void processLaptopSerial() {
    uint8_t completed[TelProtocol::MAX_FRAME_SIZE];
    size_t completedLen = 0;

    while (Serial.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(Serial.read());

        if (laptopParser.feed(b, millis(), completed, completedLen)) {
            ++laptopRxValid;

            // Laptop -> Ground solo acepta 0x01 COMMAND (13 B) según docs/ICD.md.
            if (completed[1] == TelProtocol::MSG_COMMAND) {
                sendCommandToRover(completed, completedLen);
            }
        }
    }
}

void processRadioRx() {
    RadioPacket packet{};

    while (xQueueReceive(radioRxQueue, &packet, 0) == pdTRUE) {
        if (!macEquals(packet.srcMac, ROVER_MAC)) {
            ++radioRxInvalid;
            continue;
        }

        if (!TelProtocol::validateFrame(packet.data, packet.len)) {
            ++radioRxInvalid;
            continue;
        }

        // Rover -> Ground: 0x02 NAV, 0x03 POWER, 0x04 THERMAL, 0x05 HEALTH, 0x06 MAP.
        if (!TelProtocol::isExternalTelemetry(packet.data[1])) {
            ++radioRxInvalid;
            continue;
        }

        ++radioRxValid;

        // La laptop recibe exactamente la misma trama binaria del ICD.
        Serial.write(packet.data, packet.len);
    }
}

void setup() {
    // No imprimir texto en Serial: es el canal binario del ICD hacia GUI/main.py.
    Serial.setRxBufferSize(1024);
    Serial.begin(LAPTOP_SERIAL_BAUD);
    delay(200);

    radioRxQueue = xQueueCreate(
        RADIO_RX_QUEUE_DEPTH,
        sizeof(RadioPacket)
    );

    if (radioRxQueue == nullptr) {
        while (true) delay(1000);
    }

    if (!initEspNow()) {
        while (true) delay(1000);
    }
}

void loop() {
    processLaptopSerial();
    processRadioRx();
    delay(1);
}
