// Puente UART Pi <-> ESP32-Onboard (Packets 1/2/3, ver docs/ARCHITECTURE.md).
// Arranca solo (constructor + tarea FreeRTOS); no requiere cambios en main.cpp.
#include <Arduino.h>

// Pines UART hacia la Raspberry Pi (cambiar libremente)
#if CONFIG_IDF_TARGET_ESP32S3
#define UART_PI_RX 1
#define UART_PI_TX 2
#else
#define UART_PI_RX 34
#define UART_PI_TX 23
#endif
#define UART_PI_BAUD 115200

static constexpr uint8_t SYNC = 0xAA, ID_P1 = 0x55, ID_P2 = 0x66, ID_P3 = 0x77;
static constexpr size_t P3_MAX_PAYLOAD = 198, P3_MAX_FRAME = 6 + P3_MAX_PAYLOAD + 1;

// Estado compartido con el resto del firmware
volatile uint8_t  bridge_obstacle_state = 0;  // último Packet 2 válido
volatile uint32_t bridge_last_pi_ms = 0;      // millis() del último frame válido de la Pi

static uint8_t xor8(const uint8_t *d, size_t n) {
    uint8_t x = 0;
    while (n--) x ^= *d++;
    return x;
}

// TODO (Telecom): enviar frame Packet 3 por ESP-NOW a ESP32-GS (redefinir esta función en su archivo)
__attribute__((weak)) void telecom_send_map_chunk(const uint8_t *frame, size_t len) {
    (void)frame; (void)len;
}

// Packet 1 (12 B, <BBiiBB) hacia la Pi
// TODO (Electrónica): leer ticks de encoders y transmitir Packet 1 (0xAA 0x55) a 10 Hz por Serial1 llamando a esta función
void bridge_send_packet1(int32_t left_ticks, int32_t right_ticks, uint8_t status) {
    uint8_t f[12] = {SYNC, ID_P1};
    memcpy(&f[2], &left_ticks, 4);   // ESP32 es little-endian
    memcpy(&f[6], &right_ticks, 4);
    f[10] = status;
    f[11] = xor8(f, 11);
    Serial1.write(f, sizeof(f));
}

static void handle_frame(const uint8_t *f, size_t n) {
    if (xor8(f, n - 1) != f[n - 1]) return;  // checksum inválido: descartar
    bridge_last_pi_ms = millis();
    if (f[1] == ID_P2) {
        bridge_obstacle_state = f[2];
        // TODO (Electrónica): aplicar estado de evasión de Packet 2 al Drivetrain
    } else {
        telecom_send_map_chunk(f, n);
    }
}

static void bridge_task(void *) {
    Serial1.setRxBufferSize(1024);  // holgura para ráfagas de Packet 3
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
            if (idx == 1) {                      // tipo de paquete
                if (b == ID_P2 || b == ID_P3) { buf[idx++] = b; need = (b == ID_P2) ? 4 : 0; }
                else idx = (b == SYNC) ? 1 : 0;
                continue;
            }
            buf[idx++] = b;
            if (buf[1] == ID_P3 && idx == 6) {   // byte 5 = payload_len
                if (b > P3_MAX_PAYLOAD) { idx = 0; continue; }
                need = 6 + b + 1;
            }
            if (need && idx == need) { handle_frame(buf, idx); idx = need = 0; }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Se ejecuta antes de setup(); la tarea corre en el núcleo 1 cuando arranca el scheduler
__attribute__((constructor)) static void bridge_autostart() {
    xTaskCreatePinnedToCore(bridge_task, "uart_bridge", 4096, nullptr, 2, nullptr, 1);
}
