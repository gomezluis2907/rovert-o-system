// El módulo lee el MPU6050 por I2C sin bloquear la tarea de red ni el UART.
#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "uart_bridge_relay.h"

#if defined(ARDUINO) && !defined(BRIDGE_HOST_TEST)
#include <Wire.h>

// Las direcciones y registros I2C corresponden al mapa estándar del MPU6050.
static constexpr uint8_t  MPU_ADDR_PRIMARY   = 0x68; // La dirección primaria aplica con AD0 en GND.
static constexpr uint8_t  MPU_ADDR_SECONDARY = 0x69; // La dirección alterna aplica con AD0 en VCC.
static constexpr uint8_t  REG_SMPLRT_DIV     = 0x19;
static constexpr uint8_t  REG_CONFIG         = 0x1A;
static constexpr uint8_t  REG_GYRO_CONFIG    = 0x1B;
static constexpr uint8_t  REG_ACCEL_CONFIG   = 0x1C;
static constexpr uint8_t  REG_ACCEL_XOUT_H   = 0x3B;
static constexpr uint8_t  REG_PWR_MGMT_1     = 0x6B;
static constexpr uint8_t  REG_WHO_AM_I       = 0x75;

// Los periodos y escalas corresponden al rango ±2g, ±250 °/s y 10 Hz.
static constexpr uint32_t NAV_PERIOD_MS      = 100;
static constexpr uint32_t REPROBE_PERIOD_MS  = 5000;
static constexpr float    ACCEL_SCALE_MPS2   = 9.80665f / 16384.0f;
static constexpr float    GYRO_SCALE_DPS     = 1.0f / 131.0f;
static constexpr float    RAD_TO_DEG_F       = 57.2957795f;

// Las variables internas conservan el estado de conexión y el ritmo de muestreo.
static bool     wire_started   = false;
static bool     mpu_ready      = false;
static uint8_t  mpu_addr       = MPU_ADDR_PRIMARY;
static uint32_t last_sample_ms = 0;
static uint32_t last_probe_ms  = 0;
static uint8_t  fail_count     = 0;

// El filtro complementario estima orientación y compensa el sesgo del eje Z.
static float    est_yaw_deg    = 0.0f;
static float    est_pitch_deg  = 0.0f;
static float    est_roll_deg   = 0.0f;
static bool     angles_seeded  = false;
static float    gz_bias_sum    = 0.0f;
static float    gz_bias_dps    = 0.0f;
static uint8_t  gz_bias_count  = 0;
static constexpr uint8_t GZ_BIAS_SAMPLES = 10; // La calibración inicial promedia diez muestras.

// La función escribe un registro de 8 bits en el dispositivo I2C indicado.
static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission(true) == 0;
}

// La función lee una ráfaga de registros consecutivos desde el sensor.
static bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
    if (got != len) {
        return false;
    }
    for (size_t i = 0; i < len; ++i) {
        buf[i] = (uint8_t)Wire.read();
    }
    return true;
}

// La función verifica el registro WHO_AM_I y configura los rangos del sensor.
static bool probe_and_config(uint8_t addr) {
    uint8_t who = 0;
    if (!i2c_read_bytes(addr, REG_WHO_AM_I, &who, 1)) {
        return false;
    }
    // El identificador válido excluye los valores de bus vacío 0x00 y 0xFF.
    if (who == 0x00 || who == 0xFF) {
        return false;
    }
    // El sensor despierta con el reloj PLL del eje X y un filtro DLPF de 44 Hz.
    if (!i2c_write_reg(addr, REG_PWR_MGMT_1, 0x01))   return false;
    if (!i2c_write_reg(addr, REG_CONFIG, 0x03))       return false;
    if (!i2c_write_reg(addr, REG_SMPLRT_DIV, 0x09))   return false; // La tasa interna queda en 100 Hz.
    if (!i2c_write_reg(addr, REG_GYRO_CONFIG, 0x00))  return false;
    if (!i2c_write_reg(addr, REG_ACCEL_CONFIG, 0x00)) return false;

    mpu_addr      = addr;
    mpu_ready     = true;
    fail_count    = 0;
    angles_seeded = false;
    gz_bias_sum   = 0.0f;
    gz_bias_count = 0;
    bridge_sensor_flags |= 0x04; // El bit 2 de health_flags indica IMU disponible.
    Serial.printf("[IMU] MPU6050 listo en 0x%02X (WHO_AM_I=0x%02X, SDA=%d, SCL=%d)\n",
                  addr, who, I2C_SDA_PIN, I2C_SCL_PIN);
    return true;
}

// La rutina busca el sensor en la dirección primaria y luego en la alterna.
static void try_detect_mpu() {
    last_probe_ms = millis();
    if (probe_and_config(MPU_ADDR_PRIMARY) || probe_and_config(MPU_ADDR_SECONDARY)) {
        return;
    }
    mpu_ready = false;
    bridge_sensor_flags &= ~0x04;
}

// La rutina inicializa el bus I2C a 100 kHz con timeout corto y detecta la IMU.
void mpu6050_init() {
    if (!wire_started) {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.setClock(100000); // La frecuencia de 100 kHz evita errores con cables largos.
        Wire.setTimeOut(20);   // El límite de 20 ms protege el lazo del puente UART.
        wire_started = true;
    }
    try_detect_mpu();
    if (!mpu_ready) {
        Serial.printf("[IMU] MPU6050 no detectado en SDA=%d SCL=%d (reintentando cada 5 s)\n",
                      I2C_SDA_PIN, I2C_SCL_PIN);
    }
}

// La rutina muestrea la IMU cada 100 ms y publica la trama 0x02 NAV.
void mpu6050_step() {
    if (!wire_started) {
        mpu6050_init();
    }
    uint32_t now = millis();

    // El controlador reintenta la detección cada 5 s si el sensor está ausente.
    if (!mpu_ready) {
        if ((int32_t)(now - last_probe_ms) >= (int32_t)REPROBE_PERIOD_MS) {
            try_detect_mpu();
        }
        return;
    }

    if (last_sample_ms != 0 && (int32_t)(now - last_sample_ms) < (int32_t)NAV_PERIOD_MS) {
        return;
    }

    float dt = (last_sample_ms == 0) ? 0.1f : ((float)(int32_t)(now - last_sample_ms) * 0.001f);
    if (dt <= 0.0f || dt > 1.0f) dt = 0.1f;
    last_sample_ms = now;

    // La lectura de 14 bytes obtiene acelerómetro, temperatura interna y giroscopio.
    uint8_t raw[14];
    if (!i2c_read_bytes(mpu_addr, REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        if (++fail_count >= 5) {
            mpu_ready = false;
            bridge_sensor_flags &= ~0x04;
            last_probe_ms = now;
            Serial.println("[IMU] Lectura I2C fallida; marcando MPU6050 desconectado");
        }
        return;
    }
    fail_count = 0;

    // Los registros de 16 bits llegan en orden big-endian.
    int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    // Las unidades se convierten a m/s² para aceleración y °/s para giro.
    float ax = (float)ax_raw * ACCEL_SCALE_MPS2;
    float ay = (float)ay_raw * ACCEL_SCALE_MPS2;
    float az = (float)az_raw * ACCEL_SCALE_MPS2;

    float gx = (float)gx_raw * GYRO_SCALE_DPS;
    float gy = (float)gy_raw * GYRO_SCALE_DPS;
    float gz = (float)gz_raw * GYRO_SCALE_DPS;

    // El promedio inicial estima el sesgo estático del giroscopio en el eje Z.
    if (gz_bias_count < GZ_BIAS_SAMPLES) {
        gz_bias_sum += gz;
        gz_bias_count++;
        gz_bias_dps = gz_bias_sum / (float)gz_bias_count;
    }

    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG_F;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG_F;

    // El filtro complementario fusiona el giroscopio con la referencia gravitacional.
    if (!angles_seeded) {
        est_roll_deg  = acc_roll;
        est_pitch_deg = acc_pitch;
        est_yaw_deg   = 0.0f;
        angles_seeded = true;
    } else {
        est_roll_deg  = 0.96f * (est_roll_deg  + gx * dt) + 0.04f * acc_roll;
        est_pitch_deg = 0.96f * (est_pitch_deg + gy * dt) + 0.04f * acc_pitch;
        est_yaw_deg  += (gz - gz_bias_dps) * dt;
        if (est_yaw_deg > 180.0f)  est_yaw_deg -= 360.0f;
        if (est_yaw_deg < -180.0f) est_yaw_deg += 360.0f;
    }

    bridge_sensor_flags |= 0x04;
    // El puente transmite 0x02 NAV por ESP-NOW y lo refleja por UART hacia la Pi.
    bridge_send_nav(est_yaw_deg, est_pitch_deg, est_roll_deg, ax, ay, az);
}

#else
// Los stubs vacíos permiten compilar las pruebas en PC sin hardware Arduino.
void mpu6050_init() {}
void mpu6050_step() {}
#endif
