# Puente de Comunicación (`Raspberry Pi` ↔ `ESP32-Onboard`)

Este documento explica cómo se hablan la `Raspberry Pi 4` y la `ESP32-Onboard` por cable y qué falta definir con los demás equipos.

---

## 1. Para qué sirve este puente

El rover tiene dos tarjetas a bordo que necesitan pasarse información todo el tiempo:

* **La `Raspberry Pi 4`** lee el LiDAR, arma el mapa y decide hacia dónde moverse, pero no tiene radio `ESP-NOW` ni maneja motores.
* **La `ESP32-Onboard`** controla los cuatro motores, lee los sensores (`BNO085`, `INA226`, `DS18B20`) y tiene la antena de radio `ESP-NOW`, pero no puede procesar el mapa.
* **El puente serie (`UART` a `115200` baudios)** une ambas tarjetas con tres cables (`TX`, `RX` y `GND`) para que trabajen como un solo sistema.

---

## 2. Qué viaja por el cable en cada dirección

### De la `ESP32-Onboard` hacia la `Raspberry Pi`
* **Órdenes de misión (`0x01 COMMAND`, `13 B`):** La `ESP32-Onboard` recibe por radio la orden de la Estación Terrena (`Iniciar misión`, `Detener misión`, `Avanzar / Girar`, `Detener`) y la pasa por el cable hacia la `Raspberry Pi`.
* **Encoders opcionales (`0x55 ENCODERS`, `12 B`):** La `ESP32-Onboard` puede enviar pulsos de rueda si en el futuro se conectan encoders físicos.

### De la `Raspberry Pi` hacia la `ESP32-Onboard`
* **Estado de manejo (`0x66 DRIVE_STATE`, `4 B`):** La `Raspberry Pi` envía un número del `0` al `4` (`0=Avanzar`, `1=Girar izq.`, `2=Girar der.`, `3=Frenar`, `4=Mantener rumbo`) cada vez que cambia el camino y lo repite cada `200 ms`.
* **Salud y velocidad de la Pi (`0x90 PI_TELEMETRY`, `12 B`):** La `Raspberry Pi` envía a `10 Hz` su velocidad lineal (`/odom`), temperatura de procesador (`°C`), uso de `CPU (%)`, memoria `RAM (MB)` y estado del LiDAR para que la `ESP32` arme los paquetes `0x02 NAV` y `0x05 HEALTH`.
* **Pedazos del mapa (`0x06 MAP`, `8 a 205 B`):** La `Raspberry Pi` envía el mapa comprimido en rebanadas (una cada `50 ms`) para que la `ESP32-Onboard` las transmita por radio en segundo plano.

---

## 3. Cómo funciona por dentro

1. **En la `Raspberry Pi` (`esp32_bridge_node.py`):**
   * El nodo abre el puerto `/dev/ttyAMA0` a `115200` baudios sin bloquear el procesador.
   * Un reloj de `100 Hz` lee los bytes entrantes con `Packet1Parser`, verifica el sello `XOR` y publica las órdenes en `/mission_command`.
   * Tres relojes de salida envían el estado de manejo (`5 Hz`), los datos de la Pi (`10 Hz`) y la cola de pedazos de mapa (`20 Hz`).
2. **En la `ESP32-Onboard` (`uart_bridge_relay.cpp`):**
   * El archivo arranca solo una tarea `FreeRTOS` (`bridge_task`) en el núcleo `1` antes de `setup()`.
   * La tarea escucha `Serial1`, valida el byte `XOR` y guarda los datos en variables compartidas (`bridge_obstacle_state`, `bridge_pi_velocity_mps`, `bridge_pi_cpu_temp_c`, `bridge_pi_cpu_pct`, `bridge_pi_ram_mb`).
   * Una cola circular de `16` espacios guarda los pedazos de mapa y solo los suelta al radio cuando no hay telemetría urgente (`NAV`, `POWER`, `THERMAL`, `HEALTH`).

---

## 4. Archivos y dependencias involucradas

### Archivos del repositorio
* **En `roverto_ws` (Raspberry Pi):**
  * `roverto_ws/src/roverto_autonomy/roverto_autonomy/esp32_bridge_node.py` (nodo principal del puente).
  * `roverto_ws/src/roverto_autonomy/roverto_autonomy/protocol.py` (empaquetado binario y `XOR`).
  * `roverto_ws/src/roverto_bringup/launch/real_rover.launch.xml` (arranque del nodo con `serial_port:=/dev/ttyAMA0`).
* **En `firmware/esp32_onboard` (ESP32):**
  * `firmware/esp32_onboard/src/uart_bridge_relay.cpp` (lector/escritor `Serial1` y constructores `ICD`).
  * `firmware/esp32_onboard/src/rover_state_machine.cpp` y `rover_state_machine.h` (máquina de estados que consume `bridge_obstacle_state`).
  * `firmware/esp32_onboard/platformio.ini` (configuración para `esp32s3` y `esp32dev`).

### Dependencias de sistema y librerías
* **En la `Raspberry Pi` (Ubuntu 24.04 + ROS 2 Jazzy):**
  * El paquete `python3-serial` (`pyserial`) permite abrir el puerto serie desde Python.
  * El usuario de Linux debe pertenecer al grupo `dialout` para tener permiso sobre `/dev/ttyAMA0`.
  * El archivo `/boot/firmware/config.txt` requiere `enable_uart=1` y `dtoverlay=disable-bt`, y el archivo `/boot/firmware/cmdline.txt` no debe tener `console=serial0,115200`.
* **En la `ESP32-Onboard`:**
  * El framework `Arduino-ESP32` provee `Serial1` y `FreeRTOS` (`xTaskCreatePinnedToCore`).

---

## 5. Dependencias abiertas (pendientes de cerrar con el equipo)

| Área | Dependencia abierta | Estado actual en código | Qué se debe acordar / hacer |
|---|---|---|---|
| **Pines UART en `ESP32`** | Pines `UART_PI_RX` y `UART_PI_TX` en `uart_bridge_relay.cpp` (líneas `8`–`14`). | El código usa `RX=1, TX=2` en `ESP32-S3` y `RX=34, TX=23` en `ESP32 WROOM`. | **Electrónica** debe confirmar qué modelo exacto de `ESP32` soldaron y qué pines dejaron libres para el cable hacia la Pi. |
| **Cableado físico Pi ↔ ESP32** | Cruce de líneas `TX`/`RX` y tierra común (`3.3 V`). | La Pi usa `GPIO 14` (`Pin 8`, `TX`), `GPIO 15` (`Pin 10`, `RX`) y `GND` (`Pin 6`). | **Electrónica** debe conectar `TX Pi → RX ESP32`, `RX Pi → TX ESP32` y `GND → GND` (ambas trabajan a `3.3 V`). |
| **Funciones de radio (`ESP-NOW`)** | Envío y recepción inalámbrica en `uart_bridge_relay.cpp`. | Las funciones `telecom_send_frame()`, `bridge_on_gs_frame()` y `bridge_notify_radio_tx_status()` están listas pero sin llamar. | **Telecomunicaciones** debe conectar esas tres funciones con `esp_now_send()`, `OnDataRecv` y `OnDataSent`. |
| **Lectura de sensores a bordo** | Envío de `BNO085`, `INA226` y `3× DS18B20`. | Las funciones `bridge_send_nav()`, `bridge_send_power()`, `bridge_send_thermal()` y `bridge_send_health()` ya arman las tramas. | **Electrónica** debe leer sus sensores y llamar a esas cuatro funciones a `10 Hz`, `1 Hz`, `0.1 Hz` y `0.5 Hz`. |
| **Conexión con motores** | Llamada a `rover_state_machine_step()` y `Drivetrain`. | El archivo `rover_state_machine.cpp` tiene las cuatro acciones de motor comentadas y `main.cpp` no tiene `loop()`. | **Electrónica** debe descomentar las llamadas a `drive` y agregar `loop()` en `main.cpp`. |
