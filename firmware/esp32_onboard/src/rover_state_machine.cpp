// Máquina de estados reactiva para evasión de obstáculos (ver docs/ICD.md).
#include "rover_state_machine.h"
#include <Arduino.h>

// Variables compartidas desde uart_bridge_relay.cpp
extern volatile uint8_t  bridge_obstacle_state;
extern volatile uint32_t bridge_last_pi_ms;
extern volatile bool     bridge_link_lost;

static constexpr uint32_t PI_UART_TIMEOUT_MS = 1000;
static uint8_t current_state = ROVER_STANDBY;

uint8_t rover_state_machine_step() {
    uint32_t now = millis();
    bool pi_timeout = (bridge_last_pi_ms == 0) ||
                      ((uint32_t)(now - bridge_last_pi_ms) > PI_UART_TIMEOUT_MS);

    // Seguridad: detener si falla el enlace GS o el UART con la Raspberry Pi
    if (bridge_link_lost || pi_timeout) {
        current_state = ROVER_STANDBY;
        // INSTRUCCIÓN PARA DETENER MOTORES
        return current_state;
    }

    switch (bridge_obstacle_state) {
        case OBS_AVOID_LEFT:
            current_state = ROVER_AVOIDING;
            // INSTRUCCIÓN PARA ROTAR A LA IZQUIERDA
            break;

        case OBS_AVOID_RIGHT:
            current_state = ROVER_AVOIDING;
            // INSTRUCCIÓN PARA ROTAR A LA DERECHA
            break;

        case OBS_CLEAR:
        case OBS_HOLD_HEADING:
            current_state = ROVER_EXECUTING;
            // INSTRUCCIÓN PARA AVANZAR HACIA ADELANTE
            break;

        case OBS_STOP:
        default:
            current_state = ROVER_STANDBY;
            // INSTRUCCIÓN PARA DETENER MOTORES
            break;
    }

    return current_state;
}
