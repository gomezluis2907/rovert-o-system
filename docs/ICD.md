# ICD de Comunicaciones (SW – TEL – ELEC)

Este documento define los mensajes entre la Estación Terrena, la ESP32-Onboard y la Raspberry Pi 4.

---

## 1. Reglas del enlace

* **Orden de bytes:** Todos los campos numéricos usan formato Little-Endian (`<`) y estructuras empaquetadas sin relleno.
* **Formato general:** Cada paquete inicia con `0xAA`, sigue con el `ID` de un byte, continúa con los datos y termina con un byte `XOR`.
* **Integridad y secuencia:** Cada mensaje incluye un número de secuencia (`seq`) y un byte de verificación `XOR` calculado sobre todos los bytes anteriores.
* **Ancho de banda:** El enlace ESP-NOW opera a `100 kbps` nominales (`85 kbps` mínimos), con tope de `8.81 kbps` de bajada y `0.20 kbps` de subida.
* **Prioridad de envío:** Los comandos (`0x01`) y la telemetría (`0x02`–`0x05`) tienen prioridad sobre los fragmentos de mapa (`0x06`).
* **Pérdida de enlace:** El rover entra en modo `STANDBY` cuando la comunicación falla por más de `2 s`.
* **Datos vencidos:** La interfaz marca un dato como vencido cuando su antigüedad supera `3` periodos de su frecuencia normal.

---

## 2. Tabla general de mensajes (ESP-NOW y USB)

| ID | Mensaje | Dirección | Sensores / Origen | Frecuencia | Tamaño total | Ancho de banda |
|---|---|---|---|---|---|---|
| `0x01` | **COMMAND** | Estación → Rover | GUI operador | Por evento (`≤ 1 s` latencia) | `13 B` | `≤ 0.11 kbps` |
| `0x02` | **NAV** | Rover → Estación | BNO085 + `/odom` Pi | `10 Hz` | `36 B` | `2.88 kbps` |
| `0x03` | **POWER** | Rover → Estación | INA226 | `1 Hz` | `17 B` | `0.14 kbps` |
| `0x04` | **THERMAL** | Rover → Estación | 3× DS18B20 | `0.1 Hz` (c/ `10 s`) | `21 B` | `0.02 kbps` |
| `0x05` | **HEALTH** | Rover → Estación | RPi 4 + ESP32 | `0.5 Hz` (c/ `2 s`) | `13 B` | `0.05 kbps` |
| `0x06` | **MAP** | Rover → Estación | `slam_toolbox` (Pi) | `≤ 0.08 Hz` (≥ `12.5 s`) | `8 a 205 B` (`≤ 242 B`) | `~1.29 kbps` |

El tráfico total promedio suma `~4.38 kbps` de bajada y `≤ 0.11 kbps` de subida.

---

## 3. Detalle de cada mensaje

### `0x01` — COMMAND (`13 B`, `<BBBBffB`)
La Estación Terrena envía órdenes de misión y movimiento al rover.

| Campo | Tipo | Bytes | Descripción |
|---|---|---|---|
| `sync` | `uint8` | `1` | Byte fijo `0xAA`. |
| `id` | `uint8` | `1` | Identificador fijo `0x01`. |
| `seq` | `uint8` | `1` | Contador de secuencia (`0..255`). |
| `cmd_id` | `uint8` | `1` | Código de acción (`0`–`3`). |
| `distance_m` | `float32` | `4` | Distancia objetivo en metros (`m`). |
| `angle_deg` | `float32` | `4` | Giro relativo en grados (`°`; `+` izquierda, `-` derecha). |
| `xor` | `uint8` | `1` | Suma `XOR` de los `12` bytes anteriores. |

Los valores válidos de `cmd_id` son cuatro:
* **`0` (DETENER):** El rover frena los motores y mantiene la misión activa.
* **`1` (AVANZAR / GIRAR):** El rover gira `angle_deg` grados y avanza `distance_m` metros (`≤ 0.04 m/s`).
* **`2` (INICIAR MISIÓN):** El sistema reinicia odometría, mapa y registro local desde cero.
* **`3` (DETENER MISIÓN):** El rover frena, apaga motores, envía el último mapa y cierra el registro local.

---

### `0x02` — NAV (`36 B`, `<BBBIfffffffB`)
La ESP32-Onboard envía orientación, aceleración y velocidad lineal a `10 Hz`.

| Campo | Tipo | Bytes | Descripción |
|---|---|---|---|
| `sync` | `uint8` | `1` | Byte fijo `0xAA`. |
| `id` | `uint8` | `1` | Identificador fijo `0x02`. |
| `seq` | `uint8` | `1` | Contador de secuencia (`0..255`). |
| `timestamp_ms` | `uint32` | `4` | Tiempo del sistema en milisegundos (`ms`). |
| `yaw` | `float32` | `4` | Rumbo en grados (`°`) del BNO085. |
| `pitch` | `float32` | `4` | Inclinación longitudinal en grados (`°`) del BNO085. |
| `roll` | `float32` | `4` | Inclinación lateral en grados (`°`) del BNO085. |
| `accel_x` | `float32` | `4` | Aceleración lineal en X (`m/s²`) del BNO085. |
| `accel_y` | `float32` | `4` | Aceleración lineal en Y (`m/s²`) del BNO085. |
| `accel_z` | `float32` | `4` | Aceleración lineal en Z (`m/s²`) del BNO085. |
| `velocity` | `float32` | `4` | Velocidad lineal de avance (`m/s`) desde `/odom` de la Pi. |
| `xor` | `uint8` | `1` | Suma `XOR` de los `35` bytes anteriores. |

---

### `0x03` — POWER (`17 B`, `<BBBIffBB`)
La ESP32-Onboard envía los datos eléctricos del sensor INA226 a `1 Hz`.

| Campo | Tipo | Bytes | Descripción |
|---|---|---|---|
| `sync` | `uint8` | `1` | Byte fijo `0xAA`. |
| `id` | `uint8` | `1` | Identificador fijo `0x03`. |
| `seq` | `uint8` | `1` | Contador de secuencia (`0..255`). |
| `timestamp_ms` | `uint32` | `4` | Tiempo del sistema en milisegundos (`ms`). |
| `voltage_v` | `float32` | `4` | Voltaje de batería en Volts (`V`). |
| `current_a` | `float32` | `4` | Corriente total en Amperes (`A`). |
| `power_flags` | `uint8` | `1` | Máscara de bits de estado eléctrico. |
| `xor` | `uint8` | `1` | Suma `XOR` de los `16` bytes anteriores. |

La máscara `power_flags` usa cuatro bits (`1` = activo/alerta):
* **Bit 0 (`0x01`):** El voltaje de batería está por debajo del umbral mínimo.
* **Bit 1 (`0x02`):** El consumo de corriente supera el límite nominal.
* **Bit 2 (`0x04`):** Los drivers de motores (`STBY`) están habilitados.
* **Bit 3 (`0x08`):** El sensor INA226 responde correctamente.

---

### `0x04` — THERMAL (`21 B`, `<BBBIfffBB`)
La ESP32-Onboard envía las lecturas de los tres sensores DS18B20 a `0.1 Hz`.

| Campo | Tipo | Bytes | Descripción |
|---|---|---|---|
| `sync` | `uint8` | `1` | Byte fijo `0xAA`. |
| `id` | `uint8` | `1` | Identificador fijo `0x04`. |
| `seq` | `uint8` | `1` | Contador de secuencia (`0..255`). |
| `timestamp_ms` | `uint32` | `4` | Tiempo del sistema en milisegundos (`ms`). |
| `temp_battery_c` | `float32` | `4` | Temperatura de batería en `°C` (rango nominal `5` a `30 °C`). |
| `temp_pcb_c` | `float32` | `4` | Temperatura ambiente / PCB en `°C`. |
| `temp_lidar_c` | `float32` | `4` | Temperatura en zona LiDAR / motores en `°C`. |
| `thermal_flags` | `uint8` | `1` | Máscara de bits de validez y calefactores. |
| `xor` | `uint8` | `1` | Suma `XOR` de los `20` bytes anteriores. |

La máscara `thermal_flags` usa seis bits (`1` = activo):
* **Bit 0 (`0x01`):** El sensor DS18B20 de batería entrega lecturas válidas.
* **Bit 1 (`0x02`):** El sensor DS18B20 de PCB entrega lecturas válidas.
* **Bit 2 (`0x04`):** El sensor DS18B20 de LiDAR entrega lecturas válidas.
* **Bit 3 (`0x08`):** El calefactor de batería está encendido.
* **Bit 4 (`0x10`):** El calefactor de PCB está encendido.
* **Bit 5 (`0x20`):** El calefactor de LiDAR está encendido.

---

### `0x05` — HEALTH (`13 B`, `<BBBIBBBBBB`)
La ESP32-Onboard consolida el estado de la Raspberry Pi y de los sensores a `0.5 Hz`.

| Campo | Tipo | Bytes | Descripción |
|---|---|---|---|
| `sync` | `uint8` | `1` | Byte fijo `0xAA`. |
| `id` | `uint8` | `1` | Identificador fijo `0x05`. |
| `seq` | `uint8` | `1` | Contador de secuencia (`0..255`). |
| `timestamp_ms` | `uint32` | `4` | Tiempo del sistema en milisegundos (`ms`). |
| `rover_status` | `uint8` | `1` | Estado del rover: `0=STANDBY`, `1=EXECUTING`, `2=AVOIDING`, `3=GOAL_REACHED`. |
| `obstacle_state` | `uint8` | `1` | Guardia LiDAR: `0=CLEAR`, `1=LEFT`, `2=RIGHT`, `3=STOP`, `4=HOLD_HEADING`. |
| `rpi_cpu_temp_c` | `uint8` | `1` | Temperatura del procesador de la Raspberry Pi en `°C` (límite `75 °C`). |
| `rpi_cpu_pct` | `uint8` | `1` | Carga de CPU de la Raspberry Pi en porcentaje (`0..100 %`; límite `70 %`). |
| `health_flags` | `uint8` | `1` | Máscara de bits de salud del sistema. |
| `xor` | `uint8` | `1` | Suma `XOR` de los `12` bytes anteriores. |

La máscara `health_flags` usa seis bits (`1` = normal / activo, excepto el bit 5):
* **Bit 0 (`0x01`):** El enlace UART entre la Raspberry Pi y la ESP32-Onboard está activo.
* **Bit 1 (`0x02`):** El escáner LiDAR LD19 publica datos en `/scan`.
* **Bit 2 (`0x04`):** La unidad inercial BNO085 responde correctamente.
* **Bit 3 (`0x08`):** El sensor eléctrico INA226 responde correctamente.
* **Bit 4 (`0x10`):** Los sensores térmicos DS18B20 responden correctamente.
* **Bit 5 (`0x20`):** El sistema activó el modo seguro por pérdida de enlace (`> 2 s`).

---

### `0x06` — MAP (`8 a 205 B`, `<BBBBBB` + `payload` + `xor`)
La Raspberry Pi comprime el mapa de `0.10 m/celda` cada `0.5 m` recorridos (`≤ 0.08 Hz`, hasta `1950 B` comprimidos) y lo divide en tramas numeradas.

| Campo | Tipo | Bytes | Descripción |
|---|---|---|---|
| `sync` | `uint8` | `1` | Byte fijo `0xAA`. |
| `id` | `uint8` | `1` | Identificador fijo `0x06` (o `0x77` en modo heredado). |
| `map_seq_id` | `uint8` | `1` | Número de mapa transmitido (`0..255`). |
| `chunk_id` | `uint8` | `1` | Índice de la rebanada (`0 .. total_chunks - 1`). |
| `total_chunks` | `uint8` | `1` | Cantidad total de rebanadas de ese mapa. |
| `payload_len` | `uint8` | `1` | Longitud de los datos de esta trama (`1..198`). |
| `payload` | `bytes` | `payload_len` | Segmento del archivo `zlib` con cabecera `<HHHhh` y celdas de `2` bits. |
| `xor` | `uint8` | `1` | Suma `XOR` de todos los bytes anteriores. |

El bloque descomprimido comienza con `10` bytes (`width`, `height`, `resolution_mm`, `origin_x_cm`, `origin_y_cm`) y sigue con `4` celdas por byte (`0=desconocido`, `1=libre`, `2=ocupado`).

---

## 4. Puente interno UART (Raspberry Pi ↔ ESP32-Onboard)

La conexión serie opera a `115200` baudios y reutiliza los paquetes anteriores más dos tramas internas:

* **Tramas pasantes:** La ESP32-Onboard reenvía `0x01 COMMAND` hacia la Raspberry Pi y reenvía `0x06 MAP` hacia la ESP32-GS sin alterar su contenido.
* **Trama `0x66` (`DRIVE_STATE`, Pi → ESP32, `4 B`):** La Raspberry Pi envía `<BBBB` (`0xAA`, `0x66`, `obstacle_state`, `xor`) para indicar evasión, giro o paro al `Drivetrain`.
* **Trama `0x90` (`PI_TELEMETRY`, Pi → ESP32, `10 B`):** La Raspberry Pi envía `<BBfBBBB` (`0xAA`, `0x90`, `velocity_mps`, `cpu_temp_c`, `cpu_pct`, `pi_flags`, `xor`) a `10 Hz` para alimentar los mensajes `0x02 NAV` y `0x05 HEALTH`.
* **Trama `0x55` (`ENCODERS`, ESP32 → Pi, `12 B`):** La ESP32-Onboard envía `<BBiiBB` (`0xAA`, `0x55`, `left_ticks`, `right_ticks`, `rover_status`, `xor`) cuando existan encoders de rueda conectados.
