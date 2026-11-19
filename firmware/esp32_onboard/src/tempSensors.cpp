// Lectura no bloqueante de 3x DS18B20 (OneWire en GPIO 4 en ESP32 WROOM) -> 0x04 THERMAL (0.1 Hz).
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "uart_bridge_relay.h"

// En ESP32 WROOM (esp32dev), GPIO 4 está libre (los motores usan 13,14,27,26,25,33,32,16,17,18,19,21,22).
#ifndef ONE_WIRE_BUS
#define ONE_WIRE_BUS 4
#endif

static OneWire oneWire(ONE_WIRE_BUS);
static DallasTemperature sensors(&oneWire);

static DeviceAddress sensor1 = { 0x28, 0xD3, 0x76, 0x48, 0x00, 0x00, 0x00, 0x8E };
static DeviceAddress sensor2 = { 0x28, 0x24, 0xF5, 0x47, 0x00, 0x00, 0x00, 0x66 };
static DeviceAddress sensor3 = { 0x28, 0x78, 0xEE, 0x48, 0x00, 0x00, 0x00, 0xA3 };

static bool     temp_initialized = false;
static bool     conversion_pending = false;
static uint32_t conversion_start_ms = 0;
static uint32_t last_thermal_tx_ms = 0;

static constexpr uint32_t CONVERSION_WAIT_MS = 800;   // >= 750 ms a 12 bits, sin bloquear loop()
static constexpr uint32_t THERMAL_PERIOD_MS  = 10000; // 0.1 Hz (cada 10 s según docs/ICD.md)

static bool valid_temp_c(float t) {
    return (t > -55.0f) && (t < 125.0f) && (t != DEVICE_DISCONNECTED_C);
}

void temp_sensors_init() {
    if (temp_initialized) return;
    sensors.begin();
    sensors.setWaitForConversion(false);  // evita bloquear ~750 ms en requestTemperatures()
    temp_initialized = true;
}

void temp_sensors_step() {
    if (!temp_initialized) {
        temp_sensors_init();
    }
    uint32_t now = millis();

    if (!conversion_pending) {
        if (last_thermal_tx_ms == 0 || (int32_t)(now - last_thermal_tx_ms) >= (int32_t)(THERMAL_PERIOD_MS - CONVERSION_WAIT_MS)) {
            sensors.requestTemperatures();
            conversion_start_ms = now;
            conversion_pending = true;
        }
        return;
    }

    if ((int32_t)(now - conversion_start_ms) < (int32_t)CONVERSION_WAIT_MS) {
        return;
    }

    conversion_pending = false;
    last_thermal_tx_ms = now;

    float t1 = sensors.getTempC(sensor1);
    float t2 = sensors.getTempC(sensor2);
    float t3 = sensors.getTempC(sensor3);

    // Si los sensores se buscan por índice en vez de ROM fija, respaldar con getTempCByIndex
    if (!valid_temp_c(t1)) t1 = sensors.getTempCByIndex(0);
    if (!valid_temp_c(t2)) t2 = sensors.getTempCByIndex(1);
    if (!valid_temp_c(t3)) t3 = sensors.getTempCByIndex(2);

    bool ok1 = valid_temp_c(t1);
    bool ok2 = valid_temp_c(t2);
    bool ok3 = valid_temp_c(t3);

    if (ok1 || ok2 || ok3) {
        bridge_sensor_flags |= 0x10;  // bit 4 de health_flags: DS18B20 OK
    } else {
        bridge_sensor_flags &= ~0x10;
    }

    float t_bat   = ok1 ? t1 : 0.0f;
    float t_pcb   = ok2 ? t2 : 0.0f;
    float t_lidar = ok3 ? t3 : 0.0f;

    uint8_t thermal_flags = 0;
    if (ok1 && t_bat < 5.0f)    thermal_flags |= 0x01; // batería < 5 °C
    if (ok1 && t_bat > 30.0f)   thermal_flags |= 0x02; // batería > 30 °C
    if (ok2 && t_pcb > 70.0f)   thermal_flags |= 0x04; // PCB > 70 °C
    if (ok3 && t_lidar > 60.0f) thermal_flags |= 0x08; // LiDAR > 60 °C

    bridge_send_thermal(t_bat, t_pcb, t_lidar, thermal_flags);
}
