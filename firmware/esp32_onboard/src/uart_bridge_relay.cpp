// Puente UART Pi <-> ESP32-Onboard y constructores ICD (ver docs/ICD.md).
// Arranca solo (constructor + tarea FreeRTOS); no requiere cambios en main.cpp.
#include <Arduino.h>
#include <math.h>
#include <string.h>

// Pines UART hacia la Raspberry Pi (cambiar libremente)
#if CONFIG_IDF_TARGET_ESP32S3
#define UART_PI_RX 1
#define UART_PI_TX 2
#else
#define UART_PI_RX 34
#define UART_PI_TX 23
#endif
#define UART_PI_BAUD 115200

// Identificadores de trama según docs/ICD.md
static constexpr uint8_t SYNC        = 0xAA;
static constexpr uint8_t ID_CMD      = 0x01;  // 13 B: GS -> ESP32 -> Pi (<BBBBffB)
static constexpr uint8_t ID_NAV      = 0x02;  // 36 B: ESP32 -> GS (<BBBIfffffffB)
static constexpr uint8_t ID_POWER    = 0x03;  // 17 B: ESP32 -> GS (<BBBIffBB)
static constexpr uint8_t ID_THERMAL  = 0x04;  // 21 B: ESP32 -> GS (<BBBIfffBB)
static constexpr uint8_t ID_HEALTH   = 0x05;  // 15 B: ESP32 -> GS (<BBBIBBBBHBB)
static constexpr uint8_t ID_MAP      = 0x06;  // <=205 B: Pi -> ESP32 -> GS (<BBBBBB + payload + xor)
static constexpr uint8_t ID_P1       = 0x55;  // 12 B: ESP32 -> Pi encoders (<BBiiBB)
static constexpr uint8_t ID_P2       = 0x66;  // 4 B:  Pi -> ESP32 obstacle_state (<BBBB)
static constexpr uint8_t ID_P3_LEG   = 0x77;  // ID heredado de MAP
static constexpr uint8_t ID_P4_LEG   = 0x88;  // ID heredado de COMMAND (8 B)
static constexpr uint8_t ID_PI_TEL   = 0x90;  // 12 B: Pi -> ESP32 (<BBfBBHBB)

static constexpr size_t P3_MAX_PAYLOAD = 198;
static constexpr size_t P3_MAX_FRAME   = 6 + P3_MAX_PAYLOAD + 1;  // 205 B
static constexpr uint32_t LINK_TIMEOUT_MS = 2000;                 // > 2 s sin enlace -> STANDBY

// Estado compartido con el resto del firmware (Drivetrain, sensores y Telecom)
volatile uint8_t  bridge_obstacle_state  = 3;     // arranca en STOP; luego lo actualiza 0x66 (3=STOP si cae el enlace)
volatile uint32_t bridge_last_pi_ms      = 0;     // millis() de la última trama válida desde la Pi
volatile uint32_t bridge_last_gs_ms      = 0;     // millis() del último comando/heartbeat desde GS
volatile bool     bridge_link_lost       = false; // true si el enlace GS supera 2 s de inactividad
volatile float    bridge_pi_velocity_mps = 0.0f;  // velocidad lineal desde /odom (trama 0x90)
volatile uint8_t  bridge_pi_cpu_temp_c   = 0;     // temperatura SoC RPi en °C (trama 0x90)
volatile uint8_t  bridge_pi_cpu_pct      = 0;     // uso de CPU RPi en % (trama 0x90)
volatile uint16_t bridge_pi_ram_mb       = 0;     // RAM usada en RPi en MB (trama 0x90)
volatile uint8_t  bridge_pi_flags        = 0;     // bit0=UART Pi ok, bit1=LiDAR /scan ok, bits2..3=rover_status (trama 0x90)
volatile uint8_t  bridge_pi_rover_status = 0;     // rover_status (0..3) reportado por la Pi en 0x90
volatile uint8_t  bridge_last_cmd_id     = 0;     // último cmd_id recibido (0..3)
volatile float    bridge_last_cmd_dist_m = 0.0f;  // última distancia recibida (m)
volatile float    bridge_last_cmd_angle  = 0.0f;  // último ángulo recibido (°)

// Cola circular de baja prioridad para fragmentos de mapa (0x06)
static constexpr size_t MAP_QUEUE_SLOTS = 16;
static uint8_t  map_q_buf[MAP_QUEUE_SLOTS][P3_MAX_FRAME];
static uint8_t  map_q_len[MAP_QUEUE_SLOTS];
static volatile size_t map_q_head = 0, map_q_tail = 0;
static volatile uint32_t last_hp_tx_ms = 0;
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

// Hook general para Telecom: envía cualquier trama ICD (0x02..0x06) por ESP-NOW hacia ESP32-GS
__attribute__((weak)) void telecom_send_frame(const uint8_t *frame, size_t len) {
    (void)frame; (void)len;
}

// Hook específico de mapa (mantenido por compatibilidad; por defecto delega a telecom_send_frame)
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

// Reporta el estado de ACK de transmisión ESP-NOW (llamar desde OnDataSent en Telecom)
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

// Punto de entrada para Telecom cuando llega una trama por ESP-NOW desde ESP32-GS
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
                     float velocity_mps = NAN) {
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
    last_hp_tx_ms = ts;
    telecom_send_frame(f, sizeof(f));
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
    uint8_t flags = (sensor_flags & 0x1C)
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
    last_hp_tx_ms = ts;
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
    }
}

static void bridge_task(void *) {
    Serial1.setRxBufferSize(1024);  // holgura para ráfagas de 0x06 MAP
    Serial1.begin(UART_PI_BAUD, SERIAL_8N1, UART_PI_RX, UART_PI_TX);
    static uint8_t buf[P3_MAX_FRAME];
    size_t idx = 0, need = 0;
    for (;;) {
        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (idx == 0) {                      // esperar 0xAA
                if (b == SYNC) buf[idx++] = b;
                continue;
            }
            if (idx == 1) {                      // tipo de paquete desde la Pi
                if (b == ID_P2) {
                    buf[idx++] = b;
                    need = 4;
                } else if (b == ID_PI_TEL) {
                    buf[idx++] = b;
                    need = 12;
                } else if (b == ID_MAP || b == ID_P3_LEG) {
                    buf[idx++] = b;
                    need = 0;
                } else {
                    idx = (b == SYNC) ? 1 : 0;
                }
                continue;
            }
            buf[idx++] = b;
            if ((buf[1] == ID_MAP || buf[1] == ID_P3_LEG) && idx == 6) {
                if (b > P3_MAX_PAYLOAD) { idx = 0; need = 0; continue; }
                need = 6 + b + 1;
            }
            if (need && idx == need) {
                handle_frame(buf, idx);
                idx = need = 0;
            }
        }

        uint32_t fail_start = radio_fail_start_ms;
        uint32_t last_hp = last_hp_tx_ms;
        uint32_t now = millis();
        // Watchdog de enlace: solo se dispara si hay fallos continuos de radio durante > 2 s
        if (fail_start != 0 && (int32_t)(now - fail_start) > (int32_t)LINK_TIMEOUT_MS) {
            trigger_link_lost();
        }

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
    xTaskCreatePinnedToCore(bridge_task, "uart_bridge", 4096, nullptr, 2, nullptr, 1);
}
