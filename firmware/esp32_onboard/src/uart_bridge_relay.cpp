// Puente integrado UART1 (Raspberry Pi <-> ESP32-Onboard) + ESP-NOW (ESP32-Onboard <-> ESP32-GS).
// Arranca solo (constructor + tarea FreeRTOS en núcleo 1); no requiere tocar main.cpp.
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "Config.h"
#include "TelProtocol.h"
#include "SerialFrameParser.h"
#include "uart_bridge_relay.h"

#if defined(ARDUINO) && !defined(BRIDGE_HOST_TEST)
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#endif

// Pines UART1 hacia la Raspberry Pi (definidos en Config.h: RX=34, TX=23 en WROOM; RX=1, TX=2 en S3)
#define UART_PI_RX   PI_UART_RX_PIN
#define UART_PI_TX   PI_UART_TX_PIN
#define UART_PI_BAUD PI_UART_BAUD

// Identificadores de trama según docs/ICD.md
static constexpr uint8_t SYNC        = TelProtocol::SYNC_BYTE;
static constexpr uint8_t ID_CMD      = TelProtocol::MSG_COMMAND;      // 13 B: GS -> ESP32 -> Pi (<BBBBffB)
static constexpr uint8_t ID_NAV      = TelProtocol::MSG_NAV;          // 36 B: ESP32 -> GS (<BBBIfffffffB)
static constexpr uint8_t ID_POWER    = TelProtocol::MSG_POWER;        // 17 B: ESP32 -> GS (<BBBIffBB)
static constexpr uint8_t ID_THERMAL  = TelProtocol::MSG_THERMAL;      // 21 B: ESP32 -> GS (<BBBIfffBB)
static constexpr uint8_t ID_HEALTH   = TelProtocol::MSG_HEALTH;       // 15 B: ESP32 -> GS (<BBBIBBBBHBB)
static constexpr uint8_t ID_MAP      = TelProtocol::MSG_MAP;          // <=205 B: Pi -> ESP32 -> GS
static constexpr uint8_t ID_P1       = TelProtocol::MSG_ENCODERS;     // 12 B: ESP32 -> Pi encoders (<BBiiBB)
static constexpr uint8_t ID_P2       = TelProtocol::MSG_DRIVE_STATE;  // 4 B:  Pi -> ESP32 obstacle_state (<BBBB)
static constexpr uint8_t ID_P3_LEG   = TelProtocol::MSG_MAP_LEGACY;   // ID heredado de MAP (0x77)
static constexpr uint8_t ID_P4_LEG   = 0x88;                          // ID heredado de COMMAND (8 B)
static constexpr uint8_t ID_PI_TEL   = TelProtocol::MSG_PI_TELEMETRY; // 12 B: Pi -> ESP32 (<BBfBBHBB)

static constexpr size_t P3_MAX_PAYLOAD = TelProtocol::MAP_MAX_PAYLOAD;
static constexpr size_t P3_MAX_FRAME   = TelProtocol::MAX_FRAME_SIZE; // 205 B

// Estado compartido con el resto del firmware (Drivetrain, sensores y Telecom)
volatile uint8_t  bridge_obstacle_state  = 3;     // arranca en STOP; luego lo actualiza 0x66 (3=STOP si cae el enlace)
volatile uint32_t bridge_last_pi_ms      = 0;     // millis() de la última trama válida desde la Pi
volatile uint32_t bridge_last_gs_ms      = 0;     // millis() del último comando/ACK desde GS
volatile bool     bridge_link_lost       = false; // true si el enlace GS supera 2 s de fallos continuos
volatile float    bridge_pi_velocity_mps = 0.0f;  // velocidad lineal desde /odom (trama 0x90)
volatile uint8_t  bridge_pi_cpu_temp_c   = 0;     // temperatura SoC RPi en °C (trama 0x90)
volatile uint8_t  bridge_pi_cpu_pct      = 0;     // uso de CPU RPi en % (trama 0x90)
volatile uint16_t bridge_pi_ram_mb       = 0;     // RAM usada en RPi en MB (trama 0x90)
volatile uint8_t  bridge_pi_flags        = 0;     // bit0=UART Pi ok, bit1=LiDAR /scan ok, bits2..3=rover_status (trama 0x90)
volatile uint8_t  bridge_pi_rover_status = 0;     // rover_status (0..3) reportado por la Pi en 0x90
volatile uint8_t  bridge_sensor_flags    = 0;     // bits 2..4 de health_flags (0x04=BNO085, 0x08=INA226, 0x10=DS18B20)
volatile uint8_t  bridge_last_cmd_id     = 0;     // último cmd_id recibido (0..3)
volatile float    bridge_last_cmd_dist_m = 0.0f;  // última distancia recibida (m)
volatile float    bridge_last_cmd_angle  = 0.0f;  // último ángulo recibido (°)

// Cola circular de baja prioridad para fragmentos de mapa (0x06)
static constexpr size_t MAP_QUEUE_SLOTS = 16;
static uint8_t  map_q_buf[MAP_QUEUE_SLOTS][P3_MAX_FRAME];
static uint8_t  map_q_len[MAP_QUEUE_SLOTS];
static volatile size_t map_q_head = 0, map_q_tail = 0;
static volatile uint32_t last_hp_tx_ms     = 0;
static volatile uint32_t last_health_tx_ms = 0;
static volatile uint32_t last_nav_tx_ms    = 0;
static volatile uint32_t radio_fail_start_ms = 0;  // 0 = enlace ESP-NOW sano; >0 = inicio de fallos de ACK
static int16_t  last_cmd_seq = -1;

static uint8_t xor8(const uint8_t *d, size_t n) {
    uint8_t x = 0;
    while (n--) x ^= *d++;
    return x;
}

static void trigger_link_lost() {
    bridge_obstacle_state = 3;  // forzar STOP (STANDBY seguro)
    if (!bridge_link_lost) {
        bridge_link_lost = true;
        // Notificar STOP (cmd_id=0) a la Raspberry Pi con un seq distinto al último comando GS
        uint8_t stop_seq = (last_cmd_seq >= 0) ? (uint8_t)((last_cmd_seq + 128) & 0xFF) : 0xFF;
        uint8_t f[13] = {SYNC, ID_CMD, stop_seq, 0};
        f[12] = xor8(f, 12);
        Serial1.write(f, sizeof(f));
    }
}

// ============================================================
//  Integración ESP-NOW de Telecomunicaciones (ROVERT0_TEL_Code)
// ============================================================
#if defined(ARDUINO) && !defined(BRIDGE_HOST_TEST)

struct RadioPacket {
    uint16_t len;
    uint8_t  srcMac[6];
    uint8_t  data[250];
};

static QueueHandle_t radioRxQueue = nullptr;
static bool espnow_ready = false;

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
static void onEspNowReceive(const esp_now_recv_info_t *info,
                            const uint8_t *data,
                            int len) {
    const uint8_t *srcMac = (info != nullptr) ? info->src_addr : nullptr;
#else
static void onEspNowReceive(const uint8_t *srcMac,
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

    if (radioRxQueue != nullptr) {
        xQueueSend(radioRxQueue, &packet, 0);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
static void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    (void)info;
    bridge_notify_radio_tx_status(status == ESP_NOW_SEND_SUCCESS);
}
#else
static void onEspNowSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    (void)macAddr;
    bridge_notify_radio_tx_status(status == ESP_NOW_SEND_SUCCESS);
}
#endif

static bool initEspNow() {
    WiFi.mode(WIFI_STA);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        return false;
    }
    if (esp_now_init() != ESP_OK) {
        return false;
    }
    esp_now_register_recv_cb(onEspNowReceive);
    esp_now_register_send_cb(onEspNowSent);

    if (!peerMacConfigured(GROUND_STATION_MAC)) {
        return false;
    }

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, GROUND_STATION_MAC, 6);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(GROUND_STATION_MAC)) {
        if (esp_now_add_peer(&peer) != ESP_OK) {
            return false;
        }
    }
    return true;
}

static void processRadioRx() {
    if (radioRxQueue == nullptr) return;
    RadioPacket packet{};
    while (xQueueReceive(radioRxQueue, &packet, 0) == pdTRUE) {
        if (!macEquals(packet.srcMac, GROUND_STATION_MAC)) {
            continue;
        }
        if (!TelProtocol::validateFrame(packet.data, packet.len)) {
            continue;
        }
        if (packet.data[1] == ID_CMD) {
            bridge_on_gs_frame(packet.data, packet.len);
        }
    }
}

void telecom_send_frame(const uint8_t *frame, size_t len) {
    if (!espnow_ready || !TelProtocol::validateFrame(frame, len)) {
        return;
    }
    if (!TelProtocol::isExternalTelemetry(frame[1])) {
        return;
    }
    esp_now_send(GROUND_STATION_MAC, frame, len);
}

#else
// Stub para pruebas unitarias en host (Linux g++)
__attribute__((weak)) void telecom_send_frame(const uint8_t *frame, size_t len) {
    (void)frame; (void)len;
}
#endif

// Hook específico de mapa (delega a telecom_send_frame)
__attribute__((weak)) void telecom_send_map_chunk(const uint8_t *frame, size_t len) {
    telecom_send_frame(frame, len);
}

static void enqueue_map_chunk(const uint8_t *frame, size_t len) {
    if (len > P3_MAX_FRAME) return;
    size_t next = (map_q_head + 1) % MAP_QUEUE_SLOTS;
    if (next == map_q_tail) {
        // Cola llena: descartar el fragmento más viejo para no bloquear
        map_q_tail = (map_q_tail + 1) % MAP_QUEUE_SLOTS;
    }
    memcpy(map_q_buf[map_q_head], frame, len);
    map_q_len[map_q_head] = (uint8_t)len;
    map_q_head = next;
}

// Envía 0x01 COMMAND (13 B, <BBBBffB) hacia la Raspberry Pi con deduplicación por seq
void bridge_send_command_to_pi(uint8_t seq, uint8_t cmd_id, float distance_m, float angle_deg) {
    bridge_last_gs_ms = millis();
    if ((int16_t)seq == last_cmd_seq) return;
    last_cmd_seq        = (int16_t)seq;
    radio_fail_start_ms = 0;
    bridge_link_lost    = false;

    bridge_last_cmd_id     = cmd_id;
    bridge_last_cmd_dist_m = distance_m;
    bridge_last_cmd_angle  = angle_deg;
    if (cmd_id == 0 || cmd_id == 3) {
        bridge_obstacle_state = 3;  // DETENER / DETENER MISIÓN frena de inmediato
    }

    uint8_t f[13] = {SYNC, ID_CMD, seq, cmd_id};
    memcpy(&f[4], &distance_m, 4);
    memcpy(&f[8], &angle_deg, 4);
    f[12] = xor8(f, 12);
    Serial1.write(f, sizeof(f));
}

// Compatibilidad hacia atrás (sin ángulo ni seq explícito)
void bridge_send_gs_command(uint8_t cmd_id, float distance_m) {
    static uint8_t auto_seq = 0;
    bridge_send_command_to_pi(auto_seq++, cmd_id, distance_m, 0.0f);
}

// Reporta el estado de ACK de transmisión ESP-NOW (llamado desde onEspNowSent)
void bridge_notify_radio_tx_status(bool ack_ok) {
    uint32_t now = millis();
    if (ack_ok) {
        bridge_last_gs_ms   = now;
        radio_fail_start_ms = 0;
        // No limpiar bridge_link_lost automáticamente: exige nuevo comando del operador
    } else {
        uint32_t fail_start = radio_fail_start_ms;
        if (fail_start == 0) {
            fail_start = (now != 0) ? now : 1;
            radio_fail_start_ms = fail_start;
        }
        if ((int32_t)(now - fail_start) > (int32_t)LINK_TIMEOUT_MS) {
            trigger_link_lost();
        }
    }
}

// Punto de entrada cuando llega una trama por ESP-NOW desde ESP32-GS
bool bridge_on_gs_frame(const uint8_t *frame, size_t len) {
    if (!frame || len < 4 || frame[0] != SYNC) return false;
    if (xor8(frame, len - 1) != frame[len - 1]) return false;

    if (frame[1] == ID_CMD && len == 13) {
        float dist = 0.0f, angle = 0.0f;
        memcpy(&dist, &frame[4], 4);
        memcpy(&angle, &frame[8], 4);
        bridge_send_command_to_pi(frame[2], frame[3], dist, angle);
        return true;
    }
    if (frame[1] == ID_P4_LEG && len == 8) {
        float dist = 0.0f;
        memcpy(&dist, &frame[3], 4);
        bridge_send_gs_command(frame[2], dist);
        return true;
    }
    return false;
}

// 0x02 NAV (36 B, <BBBIfffffffB) hacia Estación Terrena (10 Hz, alta prioridad)
void bridge_send_nav(float yaw, float pitch, float roll,
                     float accel_x, float accel_y, float accel_z,
                     float velocity_mps) {
    static uint8_t seq = 0;
    uint32_t ts = millis();
    float vel = isnan(velocity_mps) ? (float)bridge_pi_velocity_mps : velocity_mps;
    uint8_t f[36] = {SYNC, ID_NAV, seq++};
    memcpy(&f[3],  &ts,      4);
    memcpy(&f[7],  &yaw,     4);
    memcpy(&f[11], &pitch,   4);
    memcpy(&f[15], &roll,    4);
    memcpy(&f[19], &accel_x, 4);
    memcpy(&f[23], &accel_y, 4);
    memcpy(&f[27], &accel_z, 4);
    memcpy(&f[31], &vel,     4);
    f[35] = xor8(f, 35);
    last_hp_tx_ms  = ts;
    last_nav_tx_ms = ts;
    telecom_send_frame(f, sizeof(f));
    Serial1.write(f, sizeof(f));  // espejo hacia Raspberry Pi para registro local (RV-SW05)
}

// 0x03 POWER (17 B, <BBBIffBB) hacia Estación Terrena (1 Hz, alta prioridad)
void bridge_send_power(float voltage_v, float current_a, uint8_t power_flags) {
    static uint8_t seq = 0;
    uint32_t ts = millis();
    uint8_t f[17] = {SYNC, ID_POWER, seq++};
    memcpy(&f[3],  &ts,        4);
    memcpy(&f[7],  &voltage_v, 4);
    memcpy(&f[11], &current_a, 4);
    f[15] = power_flags;
    f[16] = xor8(f, 16);
    last_hp_tx_ms = ts;
    telecom_send_frame(f, sizeof(f));
    Serial1.write(f, sizeof(f));  // espejo hacia Raspberry Pi para registro local (RV-SW05)
}

// 0x04 THERMAL (21 B, <BBBIfffBB) hacia Estación Terrena (0.1 Hz, alta prioridad)
void bridge_send_thermal(float temp_battery_c, float temp_pcb_c, float temp_lidar_c,
                         uint8_t thermal_flags) {
    static uint8_t seq = 0;
    uint32_t ts = millis();
    uint8_t f[21] = {SYNC, ID_THERMAL, seq++};
    memcpy(&f[3],  &ts,             4);
    memcpy(&f[7],  &temp_battery_c, 4);
    memcpy(&f[11], &temp_pcb_c,     4);
    memcpy(&f[15], &temp_lidar_c,   4);
    f[19] = thermal_flags;
    f[20] = xor8(f, 20);
    last_hp_tx_ms = ts;
    telecom_send_frame(f, sizeof(f));
    Serial1.write(f, sizeof(f));  // espejo hacia Raspberry Pi para registro local (RV-SW05)
}

// 0x05 HEALTH (15 B, <BBBIBBBBHBB) hacia Estación Terrena (0.5 Hz, alta prioridad)
// sensor_flags aporta bits 2..4 (0x04=BNO085 OK, 0x08=INA226 OK, 0x10=DS18B20 OK)
void bridge_send_health(uint8_t rover_status, uint8_t sensor_flags) {
    static uint8_t seq = 0;
    uint32_t last_pi = bridge_last_pi_ms;
    uint32_t ts = millis();
    bool uart_ok  = (last_pi != 0) && ((int32_t)(ts - last_pi) <= 2000);
    bool lidar_ok = uart_ok && ((bridge_pi_flags & 0x02) != 0);
    uint8_t pi_status = (bridge_pi_flags >> 2) & 0x03;
    uint8_t eff_status = 0;
    if (!bridge_link_lost && uart_ok) {
        if (bridge_pi_flags & 0x01) {
            eff_status = (pi_status == 1 && rover_status == 2) ? 2 : pi_status;
        } else {
            eff_status = rover_status;
        }
    }
    uint8_t flags = ((sensor_flags | bridge_sensor_flags) & 0x1C)
                  | (uart_ok  ? 0x01 : 0x00)
                  | (lidar_ok ? 0x02 : 0x00)
                  | (bridge_link_lost ? 0x20 : 0x00);
    uint16_t ram = bridge_pi_ram_mb;

    uint8_t f[15] = {
        SYNC,
        ID_HEALTH,
        seq++,
        0, 0, 0, 0,
        eff_status,
        bridge_obstacle_state,
        bridge_pi_cpu_temp_c,
        bridge_pi_cpu_pct,
        0, 0,
        flags,
        0
    };
    memcpy(&f[3],  &ts,  4);
    memcpy(&f[11], &ram, 2);
    f[14] = xor8(f, 14);
    last_hp_tx_ms     = ts;
    last_health_tx_ms = ts;
    telecom_send_frame(f, sizeof(f));
}

void bridge_send_health(uint8_t sensor_flags) {
    bridge_send_health(bridge_pi_rover_status, sensor_flags);
}

// 0x55 ENCODERS (12 B, <BBiiBB) hacia la Pi (opcional si hay encoders físicos)
void bridge_send_packet1(int32_t left_ticks, int32_t right_ticks, uint8_t status) {
    uint8_t f[12] = {SYNC, ID_P1};
    memcpy(&f[2], &left_ticks, 4);
    memcpy(&f[6], &right_ticks, 4);
    f[10] = status;
    f[11] = xor8(f, 11);
    Serial1.write(f, sizeof(f));
}

static void handle_frame(const uint8_t *f, size_t n) {
    if (xor8(f, n - 1) != f[n - 1]) return;  // checksum inválido: descartar
    bridge_last_pi_ms = millis();
    uint8_t id = f[1];
    if (id == ID_P2 && n == 4) {
        if (!bridge_link_lost) {
            bridge_obstacle_state = f[2];
        }
    } else if (id == ID_PI_TEL && n == 12) {
        float vel = 0.0f;
        uint16_t ram = 0;
        memcpy(&vel, &f[2], 4);
        memcpy(&ram, &f[8], 2);
        bridge_pi_velocity_mps = vel;
        bridge_pi_cpu_temp_c   = f[6];
        bridge_pi_cpu_pct      = f[7];
        bridge_pi_ram_mb       = ram;
        bridge_pi_flags        = f[10];
        bridge_pi_rover_status = (f[10] >> 2) & 0x03;
    } else if (id == ID_MAP || id == ID_P3_LEG) {
        enqueue_map_chunk(f, n);
    } else if (id >= ID_NAV && id <= ID_HEALTH) {
        // Permite reenviar tramas externas 0x02..0x05 ya formadas si se inyectan por UART en pruebas
        last_hp_tx_ms = bridge_last_pi_ms;
        telecom_send_frame(f, n);
    }
}

static void bridge_task(void *) {
    // M14: esperar 50 ms para que terminen todos los constructores globales (Serial1) e initArduino()
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial1.setRxBufferSize(1024);  // holgura para ráfagas de 0x06 MAP
    Serial1.begin(UART_PI_BAUD, SERIAL_8N1, UART_PI_RX, UART_PI_TX);

#if defined(ARDUINO) && !defined(BRIDGE_HOST_TEST)
    radioRxQueue = xQueueCreate(RADIO_RX_QUEUE_DEPTH, sizeof(RadioPacket));
    if (radioRxQueue != nullptr) {
        espnow_ready = initEspNow();
    }
    temp_sensors_init();
    mpu6050_init();
#endif

    static SerialFrameParser piParser;
    static uint8_t completed[P3_MAX_FRAME];
    size_t completedLen = 0;

    for (;;) {
#if defined(ARDUINO) && !defined(BRIDGE_HOST_TEST)
        processRadioRx();
#endif

        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (piParser.feed(b, millis(), completed, completedLen)) {
                handle_frame(completed, completedLen);
            }
        }

        uint32_t fail_start = radio_fail_start_ms;
        uint32_t last_hp    = last_hp_tx_ms;
        uint32_t now        = millis();

        // Watchdog de enlace: solo se dispara si hay fallos continuos de radio durante > 2 s
        if (fail_start != 0 && (int32_t)(now - fail_start) > (int32_t)LINK_TIMEOUT_MS) {
            trigger_link_lost();
        }

#if defined(ARDUINO) && !defined(BRIDGE_HOST_TEST)
        // Lectura no bloqueante de 3x DS18B20 -> 0x04 THERMAL cada 10 s (0.1 Hz)
        temp_sensors_step();

        // La tarea lee el MPU6050 por I2C y envía 0x02 NAV a 10 Hz
        mpu6050_step();

        now = millis();
        uint32_t last_health = last_health_tx_ms;
        uint32_t last_nav    = last_nav_tx_ms;
        last_hp              = last_hp_tx_ms;

        // Envío automático de 0x05 HEALTH cada 2 s (0.5 Hz) si ningún otro módulo lo llamó
        if (last_health == 0 || (int32_t)(now - last_health) >= 2000) {
            bridge_send_health(bridge_pi_rover_status, bridge_sensor_flags);
            last_hp = millis();
        }

        // El respaldo de 0x02 NAV envía ceros a 10 Hz si el MPU6050 no transmitió
        if (bridge_last_pi_ms != 0 && (last_nav == 0 || (int32_t)(now - last_nav) >= 100)) {
            bridge_send_nav(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            last_hp = millis();
        }
#endif

        // Drenar 1 fragmento de mapa de baja prioridad si no hubo telemetría urgente en los últimos 5 ms
        if (map_q_tail != map_q_head && (int32_t)(now - last_hp) >= 5) {
            size_t slot = map_q_tail;
            uint8_t len = map_q_len[slot];
            map_q_tail = (map_q_tail + 1) % MAP_QUEUE_SLOTS;
            telecom_send_map_chunk(map_q_buf[slot], len);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Se ejecuta antes de setup(); la tarea corre en el núcleo 1 cuando arranca el scheduler
__attribute__((constructor)) static void bridge_autostart() {
    xTaskCreatePinnedToCore(bridge_task, "uart_bridge", 6144, nullptr, 2, nullptr, 1);
}
