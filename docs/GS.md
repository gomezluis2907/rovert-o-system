# Guía de la Estación Terrena (`Ground Station`)

Este documento explica qué es la Estación Terrena y cómo encenderla paso a paso desde tu computadora.

---

## 1. Qué es la Estación Terrena (`GS`)

La Estación Terrena es el puesto de mando desde donde controlas el rover y ves sus datos:

* **Tu computadora (`PC`):** Este equipo corre el programa visual en Python, muestra el mapa `2D`, dibuja las gráficas y guarda los datos en disco.
* **La tarjeta `ESP32-GS`:** Esta placa se conecta por cable `USB` a tu computadora y sirve como antena `ESP-NOW` para hablar con la `ESP32-Onboard` del rover.

---

## 2. Archivos involucrados

### En tu computadora (`ground_station_gui/`)
* **`GUI/main.py`:** Este archivo es el programa principal que abre la ventana, lee el puerto `USB` y conecta los botones.
* **`GUI/gs_serial_sender.py`:** Este archivo arma y envía las órdenes `0x01 COMMAND` (`13` bytes) por el cable `USB`.
* **`gs_map_decoder.py`:** Este archivo junta las rebanadas `0x06 MAP`, descomprime el `.zip` y reconstruye el mapa.
* **`GUI/interface.py` y `GUI/parameters.py`:** Estos archivos dibujan las pestañas **`Parámetros`** y **`Gráficas`**, los botones y los cuadros de texto.
* **`GUI/graphics.py`, `GUI/telemetry.py`, `GUI/charts.py` y `GUI/sessions.py`:** Estos archivos dibujan las gráficas en vivo y guardan las sesiones en `CSV` o `Excel`.
* **`requirements.txt`:** Este archivo lista las cinco librerías de Python necesarias para abrir la interfaz.

### En la tarjeta `ESP32-GS` (`firmware/esp32_ground_station/`)
* **`src/main.cpp` y `src/map_relay_gs.cpp`:** Estos archivos contienen el programa de la `ESP32-GS` que pasa datos entre el cable `USB` y la antena `ESP-NOW`.

---

## 3. Preparación única en tu PC (solo se hace la primera vez)

### Paso 1: Instalar soporte gráfico de Python y permisos `USB`
Tu terminal de Linux debe ejecutar estos dos comandos una sola vez:

```bash
sudo apt update
sudo apt install -y python3-pip python3-tk python3-venv
sudo usermod -aG dialout $USER
```

*(Nota: Tu sesión de Linux requiere cerrarse y volverse a abrir una vez después de `usermod` para activar el permiso del puerto `USB`).*

### Paso 2: Instalar las librerías de la interfaz
Tu terminal debe entrar a la carpeta del proyecto e instalar los paquetes de `requirements.txt`:

```bash
cd ~/rovert-o-system/ground_station_gui
pip3 install --break-system-packages -r requirements.txt
```

---

## 4. Cómo encender la Ground Station paso a paso

### Caso A: Probar la interfaz en tu PC sin conectar ninguna tarjeta (modo Demo)
Este modo sirve para verificar que la ventana abre y que las gráficas se mueven con datos falsos:

```bash
cd ~/rovert-o-system/ground_station_gui/GUI
python3 main.py --demo
```

### Caso B: Encender la interfaz con la tarjeta `ESP32-GS` conectada por `USB`

1. **El operador** atornilla la antena monopolo de `2.4 GHz` en la tarjeta `ESP32-GS` antes de conectarla.
2. **El operador** conecta la tarjeta `ESP32-GS` a un puerto `USB` de la computadora usando un cable de datos.
3. **La terminal** verifica el nombre del puerto `USB` (casi siempre `/dev/ttyUSB0` o `/dev/ttyACM0`):
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
4. **La terminal** consulta la dirección `MAC` de la placa conectada (opcional, para confirmar que es la `ESP32-GS` `8c:94:df:70:64:34` o anotar la `MAC` de una placa nueva del rover):
   ```bash
   ~/.platformio/packages/tool-esptool/esptool --port /dev/ttyUSB0 read-mac
   ```
5. **El operador** actualiza `ROVER_MAC` en `firmware/esp32_ground_station/src/Config.h` y vuelve a flashear la `ESP32-GS` **solo si cambió la placa física del rover**:
   ```bash
   nano ~/rovert-o-system/firmware/esp32_ground_station/src/Config.h
   ~/.platformio/penv/bin/pio run -d ~/rovert-o-system/firmware/esp32_ground_station -e esp32dev -t upload --upload-port /dev/ttyUSB0
   ```
   *(Nota: El usuario debe mantener presionado el botón `BOOT` durante `2 s` si la subida se detiene en `Connecting....`).*
6. **La terminal** inicia la interfaz gráfica:
   * **Cuando el puerto es `/dev/ttyUSB0`:**
     ```bash
     cd ~/rovert-o-system/ground_station_gui/GUI
     GS_PORT=/dev/ttyUSB0 python3 main.py
     ```
   * **Cuando el puerto tiene otro nombre (por ejemplo `/dev/ttyACM0`):**
     ```bash
     cd ~/rovert-o-system/ground_station_gui/GUI
     GS_PORT=/dev/ttyACM0 python3 main.py
     ```
7. **El operador** enciende el rover (o presiona el botón `EN` de la `ESP32-Onboard`) cuando la interfaz ya está abierta.
8. **La interfaz** cambia el campo `Acción en proceso` a **`En espera (Standby)`** en `2` a `3` segundos (si muestra `Enlace perdido (Standby)`, el operador presiona `Iniciar misión` o reinicia la placa del rover).

---

## 5. Cómo usar los botones ya con la ventana abierta

1. **Botón `Iniciar misión` (verde, arriba del mapa):** El operador presiona este botón primero para limpiar el mapa, poner el reloj `Tiempo [s]` en `0`, iniciar la grabación de datos y despertar la misión en el rover (`cmd_id = 2`).
2. **Cajas `Distancia [m]` y `Dirección [°]` + botón `Enviar avance` (amarillo):** El operador escribe cuántos metros avanzar (p. ej. `5`) y/o cuántos grados girar (p. ej. `90` o `0`) y presiona `Enviar avance` (`cmd_id = 1`).
3. **Botón `Detener avance` (rojo, a la derecha):** El operador presiona este botón para frenar las ruedas de inmediato sin apagar la misión (`cmd_id = 0`).
4. **Botón `Detener misión` (rojo, arriba del mapa):** El operador presiona este botón al terminar el recorrido para frenar el rover, recibir el último mapa y cerrar el archivo de grabación (`cmd_id = 3`).
5. **Pestaña `Gráficas` (arriba al centro):** El operador cambia a esta pestaña para ver las curvas en vivo y exportar los datos guardados en formato `CSV` o `Excel`.

---

## 6. Estado del enlace con `ESP32-GS`

La interfaz gráfica y el firmware de `firmware/esp32_ground_station/` ya están integrados con `ESP-NOW` y el protocolo `ICD`:

| Elemento | Estado actual | Acción requerida |
|---|---|---|
| **Conexión física de la `ESP32-GS`** | El programa entra en modo `dry-run` cuando no detecta la placa en `/dev/ttyUSB0`. | **El operador** debe conectar la `ESP32-GS` con su antena por `USB` antes de correr `main.py`. |
| **Firmware `ESP-NOW` (`0x01`–`0x06`)** | El código en `firmware/esp32_ground_station/src/` ya reenvía comandos de `13 B` y telemetría/mapas binarios sin texto extra. | **El equipo** solo debe actualizar `ROVER_MAC` en `src/Config.h` y reflashear si cambia la placa física del rover (ver `docs/TELECOM.md`). |
