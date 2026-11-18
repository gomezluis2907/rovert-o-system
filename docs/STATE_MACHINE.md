# Máquina de Estados y Evasión de Obstáculos

Este documento explica cómo el rover detecta obstáculos, esquiva piedras y protege los motores.

---

## 1. Idea general (el guía y el chofer)

La evasión funciona con dos cerebros que se reparten el trabajo:

* **La Raspberry Pi 4 (el guía):** Este equipo lee el LiDAR, decide si el camino está libre y envía un número del `0` al `4`.
* **La ESP32-Onboard (el chofer):** Esta tarjeta recibe ese número y mueve los cuatro motores sin preocuparse por el mapa.

---

## 2. Flujo completo de la información

El movimiento sigue seis pasos en bucle continuo:

1. **La Estación Terrena** envía el comando `0x01 COMMAND` (`AVANZAR / GIRAR`) por radio hacia la `ESP32-Onboard`.
2. **La `ESP32-Onboard`** (`uart_bridge_relay.cpp`) recibe esa orden y la reenvía por cable UART a la `Raspberry Pi`.
3. **El nodo `sim_esp32_emulator_node`** en la `Raspberry Pi` arranca la misión y mide con `/odom` cuánto avanza o gira el rover.
4. **El nodo `lidar_guard_node`** lee el láser `/scan` (`10` veces por segundo) y decide un estado del `0` al `4`.
5. **El puente `esp32_bridge_node`** envía la trama `0x66 DRIVE_STATE` (`4` bytes) hacia la `ESP32-Onboard` cada vez que cambia el estado (y cada `200 ms` como respaldo).
6. **La función `rover_state_machine_step()`** en `rover_state_machine.cpp` lee ese estado y elige qué deben hacer los motores.

---

## 3. Qué mira el LiDAR (`lidar_guard_node.py`)

El escáner LiDAR ignora la parte trasera (`> 135°` de cada lado) porque el propio chasis estorba en esa zona.

| Zona | Ángulo | Distancia límite | Qué revisa |
|---|---|---|---|
| **Frente** | `-30°` a `+30°` | `< 0.45 m` (`45 cm`) | Esta zona detecta si hay un objeto adelante. |
| **Decisión lateral** | `15°` a `80°` (izq. y der.) | Promedio libre | Esta zona compara qué lado tiene más espacio para girar. |
| **Costados** | `35°` a `105°` (izq. y der.) | `< 0.30 m` (`30 cm`) | Esta zona detecta si una pared roza el costado del rover. |

La lógica mantiene el giro elegido durante `0.5 s` extra (`avoid_release_sec`) después de despejar el frente para librar la esquina del obstáculo.

---

## 4. Estados que viajan por UART (`0x66 DRIVE_STATE`)

El puente `esp32_bridge_node` fuerza el estado `3` (`STOP`) cuando la misión está en `STANDBY` o `GOAL_REACHED`.

| Código | Nombre (`ObstacleState`) | Estado resultante (`RoverState`) | Qué hace el rover |
|---|---|---|---|
| `0` | `OBS_CLEAR` | `ROVER_EXECUTING` (`1`) | El rover avanza hacia adelante (`≤ 0.04 m/s`). |
| `1` | `OBS_AVOID_LEFT` | `ROVER_AVOIDING` (`2`) | El rover gira sobre su eje hacia la izquierda. |
| `2` | `OBS_AVOID_RIGHT` | `ROVER_AVOIDING` (`2`) | El rover gira sobre su eje hacia la derecha. |
| `3` | `OBS_STOP` | `ROVER_STANDBY` (`0`) | El rover frena los motores de inmediato. |
| `4` | `OBS_HOLD_HEADING` | `ROVER_EXECUTING` (`1`) | El rover sigue recto sin enderezarse para no chocar de lado. |

---

## 5. Protecciones automáticas de seguridad

La función `rover_state_machine_step()` y la `Raspberry Pi` detienen los motores (`ROVER_STANDBY`) cuando ocurre una de estas tres fallas:

* **Pérdida de radio (`bridge_link_lost`):** La `ESP32-Onboard` fuerza `OBS_STOP`, envía un `0x01 STOP` por UART a la `Raspberry Pi` y queda enclavada en `STANDBY` cuando el radio falla por más de `2000 ms` (`2 s`) hasta recibir un comando nuevo.
* **Pérdida de UART (`PI_UART_TIMEOUT_MS`):** La máquina de estados frena el rover cuando la `Raspberry Pi` nunca respondió o lleva más de `1000 ms` (`1 s`) sin mandar tramas válidas.
* **Falta de datos ROS 2 (`input_timeout_sec`):** La `Raspberry Pi` fuerza `OBS_STOP` y vuelve a `STANDBY` cuando `/scan`, `/odom`, `/rover_status` u `/obstacle_state` dejan de llegar por más de `1.0 s`.

---

## 6. Consideraciones importantes (qué falta en `esp32_onboard`)

Los archivos actuales en `firmware/esp32_onboard/src/` todavía requieren cuatro ajustes del equipo de Electrónica y Telecomunicaciones para mover el rover real:

* **Llamada en `main.cpp`:** El archivo `main.cpp` no tiene función `loop()`; su dueño debe agregar `loop()`, llamar a `rover_state_machine_step()` y llamar a `drive.update()`.
* **Instrucciones de motor comentadas:** El archivo `rover_state_machine.cpp` tiene comentarios donde faltan las llamadas reales a `Drivetrain`:
  * `// INSTRUCCIÓN PARA AVANZAR HACIA ADELANTE` → El dueño debe colocar `drive.forward();`.
  * `// INSTRUCCIÓN PARA ROTAR A LA IZQUIERDA` → El dueño debe colocar `drive.turnLeft();`.
  * `// INSTRUCCIÓN PARA ROTAR A LA DERECHA` → El dueño debe colocar `drive.turnRight();`.
  * `// INSTRUCCIÓN PARA DETENER MOTORES` → El dueño debe colocar `drive.stop();` (o `drive.brake();`).
* **Errores previos de compilación en Linux:** Los archivos `main.cpp` y `Drivetrain.h` escriben `#include "Motor.h"` con mayúscula aunque el archivo se llama `motor.h`, y el archivo `tempSensors.cpp` define otro `setup()` y `loop()` duplicados.
* **Enlace ESP-NOW:** El código de Telecomunicaciones debe redefinir `telecom_send_frame()` y llamar a `bridge_on_gs_frame()` para que los comandos `0x01` y la telemetría viajen por radio.
