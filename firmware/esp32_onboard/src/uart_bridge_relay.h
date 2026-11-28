#pragma once
#include <stddef.h>
#include <stdint.h>
#include <math.h>

// Estado compartido entre el puente UART/ESP-NOW, la máquina de estados y sensores
extern volatile uint8_t  bridge_obstacle_state;
extern volatile uint32_t bridge_last_pi_ms;
extern volatile uint32_t bridge_last_gs_ms;
extern volatile bool     bridge_link_lost;
extern volatile float    bridge_pi_velocity_mps;
extern volatile uint8_t  bridge_pi_cpu_temp_c;
extern volatile uint8_t  bridge_pi_cpu_pct;
extern volatile uint16_t bridge_pi_ram_mb;
extern volatile uint8_t  bridge_pi_flags;
extern volatile uint8_t  bridge_pi_rover_status;
extern volatile uint8_t  bridge_sensor_flags;
extern volatile uint8_t  bridge_last_cmd_id;
extern volatile float    bridge_last_cmd_dist_m;
extern volatile float    bridge_last_cmd_angle;

// Funciones de envío ICD hacia la Estación Terrena (ESP-NOW) y hacia la Raspberry Pi (UART1)
void bridge_send_command_to_pi(uint8_t seq, uint8_t cmd_id, float distance_m, float angle_deg);
void bridge_send_gs_command(uint8_t cmd_id, float distance_m);
void bridge_notify_radio_tx_status(bool ack_ok);
bool bridge_on_gs_frame(const uint8_t *frame, size_t len);

void bridge_send_nav(float yaw, float pitch, float roll,
                     float accel_x, float accel_y, float accel_z,
                     float velocity_mps = NAN);
void bridge_send_power(float voltage_v, float current_a, uint8_t power_flags);
void bridge_send_thermal(float temp_battery_c, float temp_pcb_c, float temp_lidar_c,
                         uint8_t thermal_flags);
void bridge_send_health(uint8_t rover_status, uint8_t sensor_flags);
void bridge_send_health(uint8_t sensor_flags);
void bridge_send_packet1(int32_t left_ticks, int32_t right_ticks, uint8_t status);

// Sensores térmicos DS18B20 no bloqueantes (tempSensors.cpp)
void temp_sensors_init();
void temp_sensors_step();

// El módulo mpu6050_sensor.cpp expone estas rutinas no bloqueantes a 10 Hz
void mpu6050_init();
void mpu6050_step();
