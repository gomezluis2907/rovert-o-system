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

1. **Tu mano** conecta la tarjeta `ESP32-GS` a un puerto `USB` de tu computadora usando un cable que transfiera datos.
2. **Tu terminal** verifica cómo se llama el puerto `USB` (casi siempre es `/dev/ttyUSB0` o `/dev/ttyACM0`):
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
3. **Tu terminal** inicia el programa:
   * **Cuando el puerto es `/dev/ttyUSB0`:**
     ```bash
     cd ~/rovert-o-system/ground_station_gui/GUI
     python3 main.py
     ```
   * **Cuando el puerto tiene otro nombre (por ejemplo `/dev/ttyACM0`):**
     ```bash
     cd ~/rovert-o-system/ground_station_gui/GUI
     GS_PORT=/dev/ttyACM0 python3 main.py
     ```

---

## 5. Cómo usar los botones ya con la ventana abierta

1. **Botón `Iniciar misión` (verde, arriba del mapa):** El operador presiona este botón primero para limpiar el mapa, poner el reloj `Tiempo [s]` en `0`, iniciar la grabación de datos y despertar la misión en el rover (`cmd_id = 2`).
2. **Cajas `Distancia [m]` y `Dirección [°]` + botón `Enviar avance` (amarillo):** El operador escribe cuántos metros avanzar (p. ej. `5`) y/o cuántos grados girar (p. ej. `90` o `0`) y presiona `Enviar avance` (`cmd_id = 1`).
3. **Botón `Detener avance` (rojo, a la derecha):** El operador presiona este botón para frenar las ruedas de inmediato sin apagar la misión (`cmd_id = 0`).
4. **Botón `Detener misión` (rojo, arriba del mapa):** El operador presiona este botón al terminar el recorrido para frenar el rover, recibir el último mapa y cerrar el archivo de grabación (`cmd_id = 3`).
5. **Pestaña `Gráficas` (arriba al centro):** El operador cambia a esta pestaña para ver las curvas en vivo y exportar los datos guardados en formato `CSV` o `Excel`.

---

## 6. Dependencias pendientes por cerrar con Telecomunicaciones

La interfaz de tu computadora ya está terminada, pero el enlace físico con el rover depende de tres cierres en la tarjeta `ESP32-GS`:

| Dependencia abierta | Estado actual | Qué falta hacer |
|---|---|---|
| **Conexión física de la `ESP32-GS`** | El programa entra en modo `dry-run` cuando no detecta la placa en `/dev/ttyUSB0`. | **El operador** debe conectar la `ESP32-GS` por `USB` antes de correr `python3 main.py`. |
| **Limpieza de `firmware/esp32_ground_station/src/main.cpp`** | El archivo actual todavía tiene el código viejo de prueba (`enviarBloque100KbitsLidar()`) que imprime texto cada `5 s`. | **Telecomunicaciones** debe borrar ese envío de prueba y cualquier `Serial.print` de texto para no ensuciar el cable `USB`. |
| **Enlace `ESP-NOW` bidireccional en la `ESP32-GS`** | El archivo lee `8 B` antiguos y tiene `esp_now_send()` comentado. | **Telecomunicaciones** debe leer los `13 B` de `0x01 COMMAND` desde `Serial`, enviarlos por `ESP-NOW` hacia la `ESP32-Onboard` y reenviar por `Serial.write(data, len)` las tramas `0x02`–`0x06` que lleguen del rover. |
