#pragma once
#include <stdint.h>

// Estados de evasión recibidos de la Raspberry Pi (trama 0x66 DRIVE_STATE)
enum ObstacleState : uint8_t {
    OBS_CLEAR        = 0,
    OBS_AVOID_LEFT   = 1,
    OBS_AVOID_RIGHT  = 2,
    OBS_STOP         = 3,
    OBS_HOLD_HEADING = 4
};

// Estados internos del rover (coinciden con rover_status en docs/ICD.md)
enum RoverState : uint8_t {
    ROVER_STANDBY      = 0,
    ROVER_EXECUTING    = 1,
    ROVER_AVOIDING     = 2,
    ROVER_GOAL_REACHED = 3
};

// Ejecuta un paso de la máquina de estados y devuelve el estado actual del rover
uint8_t rover_state_machine_step();
