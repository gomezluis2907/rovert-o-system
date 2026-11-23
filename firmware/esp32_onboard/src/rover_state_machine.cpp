// Máquina de estados reactiva para evasión de obstáculos (ver docs/ICD.md y docs/STATE_MACHINE.md).
#include "rover_state_machine.h"
#include <Arduino.h>

#ifndef BRIDGE_HOST_TEST
#include "Drivetrain.h"
extern Drivetrain drive;
#endif

// Variables compartidas desde uart_bridge_relay.cpp
extern volatile uint8_t  bridge_obstacle_state;
extern volatile uint32_t bridge_last_pi_ms;
extern volatile bool     bridge_link_lost;

static constexpr uint32_t PI_UART_TIMEOUT_MS = 1000;
static uint8_t current_state = ROVER_STANDBY;
#ifndef BRIDGE_HOST_TEST
static uint8_t last_applied_action = 0xFF;
static float   last_applied_cruise = -1.0f;
#endif

void rover_state_machine_force_sync() {
#ifndef BRIDGE_HOST_TEST
    last_applied_action = 0xFF;
#endif
}

uint8_t rover_state_machine_step() {
    uint32_t last_pi = bridge_last_pi_ms;
    uint32_t now = millis();
    bool pi_timeout = (last_pi == 0) ||
                      ((int32_t)(now - last_pi) > (int32_t)PI_UART_TIMEOUT_MS);

    uint8_t action = 0;  // 0=stop, 1=left, 2=right, 3=forward

    // Seguridad: detener si falla el enlace GS o el UART con la Raspberry Pi
    if (bridge_link_lost || pi_timeout) {
        current_state = ROVER_STANDBY;
        action = 0;
    } else {
        switch (bridge_obstacle_state) {
            case OBS_AVOID_LEFT:
                current_state = ROVER_AVOIDING;
                action = 1;
                break;

            case OBS_AVOID_RIGHT:
                current_state = ROVER_AVOIDING;
                action = 2;
                break;

            case OBS_CLEAR:
            case OBS_HOLD_HEADING:
                current_state = ROVER_EXECUTING;
                action = 3;
                break;

            case OBS_STOP:
            default:
                current_state = ROVER_STANDBY;
                action = 0;
                break;
        }
    }

#ifndef BRIDGE_HOST_TEST
    float cruise = drive.cruiseSpeed();
    if (action != last_applied_action || (action != 0 && cruise != last_applied_cruise)) {
        switch (action) {
            case 1:  drive.turnLeft();  break;
            case 2:  drive.turnRight(); break;
            case 3:  drive.forward();   break;
            default: drive.stop();      break;
        }
        last_applied_action = action;
        last_applied_cruise = cruise;
        Serial.printf("[SM] obs=%u link_lost=%u pi_to=%u -> L=%.2f R=%.2f (crucero=%.2f cm/s)\n",
                      (unsigned)bridge_obstacle_state,
                      (unsigned)bridge_link_lost,
                      (unsigned)pi_timeout,
                      drive.leftSpeed(),
                      drive.rightSpeed(),
                      cruise);
    }
#endif

    return current_state;
}
