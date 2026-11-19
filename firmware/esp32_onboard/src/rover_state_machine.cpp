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

uint8_t rover_state_machine_step() {
    uint32_t last_pi = bridge_last_pi_ms;
    uint32_t now = millis();
    bool pi_timeout = (last_pi == 0) ||
                      ((int32_t)(now - last_pi) > (int32_t)PI_UART_TIMEOUT_MS);

    // Seguridad: detener si falla el enlace GS o el UART con la Raspberry Pi
    if (bridge_link_lost || pi_timeout) {
        current_state = ROVER_STANDBY;
#ifndef BRIDGE_HOST_TEST
        drive.stop();
#endif
        return current_state;
    }

    switch (bridge_obstacle_state) {
        case OBS_AVOID_LEFT:
            current_state = ROVER_AVOIDING;
#ifndef BRIDGE_HOST_TEST
            drive.turnLeft();
#endif
            break;

        case OBS_AVOID_RIGHT:
            current_state = ROVER_AVOIDING;
#ifndef BRIDGE_HOST_TEST
            drive.turnRight();
#endif
            break;

        case OBS_CLEAR:
        case OBS_HOLD_HEADING:
            current_state = ROVER_EXECUTING;
#ifndef BRIDGE_HOST_TEST
            drive.forward();
#endif
            break;

        case OBS_STOP:
        default:
            current_state = ROVER_STANDBY;
#ifndef BRIDGE_HOST_TEST
            drive.stop();
#endif
            break;
    }

    return current_state;
}
