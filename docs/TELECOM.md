# Guía de Telecomunicaciones en Windows (`ESP32-Onboard` y `ESP32-GS`)

Esta guía explica cómo programar las dos placas `ESP32 WROOM` del rover desde **Windows**, paso a paso y sin usar comandos avanzados de GitHub.

---

## 1. Estado actual del enlace

* **El enlace de radio ya funciona:** Las pruebas previas confirmaron la comunicación `ESP-NOW` entre la `ESP32-GS` (con antena monopolo de `2.4 GHz`, MAC `8C:94:DF:70:64:34`) y una placa `ESP32 DEVKIT V1` de prueba (MAC `28:05:A5:30:E7:A8`).
* **El objetivo ahora es pasar el programa a la `ESP32 WROOM` final del rover:** Esta placa final llevará conectados los motores, los sensores y la `Raspberry Pi`.
* **Cada placa física tiene una dirección MAC única:** El equipo debe leer la MAC real de la `ESP32 WROOM` final del rover y escribirla en la `ESP32-GS`, porque la `ESP32-GS` rechaza cualquier mensaje que venga de otra dirección.

---

## 2. Qué archivos deben conocer (y cuáles modificar)

El repositorio tiene dos carpetas separadas dentro de `firmware/`:

| Carpeta | Para qué placa es | Archivos importantes | Qué deben modificar |
|---|---|---|---|
| `firmware/esp32_onboard/` | `ESP32 WROOM` del rover | `src/Config.h`, `src/uart_bridge_relay.cpp`, `src/TelProtocol.h` | Solo revisar `src/Config.h` (la MAC de la `ESP32-GS` ya está puesta). |
| `firmware/esp32_ground_station/` | `ESP32 WROOM` de la laptop (`GS`) | `src/Config.h`, `src/main.cpp`, `src/TelProtocol.h` | **Cambiar `ROVER_MAC`** en `src/Config.h` por la MAC real de la placa final del rover. |

* **Los archivos `TelProtocol.h`, `TelProtocol.cpp`, `SerialFrameParser.h` y `SerialFrameParser.cpp` no deben modificarse:** Esos archivos ya tienen los tamaños exactos del protocolo (`0x05 HEALTH = 15 B`, `0x90 PI_TELEMETRY = 12 B`).
* **La línea `ENABLE_DEBUG = false` en `firmware/esp32_ground_station/src/Config.h` no debe cambiarse a `true`:** Cualquier texto impreso en el USB de la `ESP32-GS` rompe la interfaz gráfica.

---

## 3. Preparación en Windows (solo se hace la primera vez)

### Paso 1: Descargar el código desde GitHub (sin usar Git)
1. El navegador debe abrir la página `https://github.com/gomezluis2907/rovert-o-system`.
2. El usuario debe verificar que el botón de la izquierda diga **`main`**.
3. El usuario debe hacer clic en el botón verde **`<> Code`** y luego en **`Download ZIP`**.
4. El explorador de Windows guardará el archivo `rovert-o-system-main.zip` en la carpeta `Descargas`.
5. El usuario debe dar clic derecho sobre `rovert-o-system-main.zip`, elegir **`Extraer todo...`** y extraerlo en un lugar fácil de encontrar (por ejemplo `C:\rovert-o-system-main`).

### Paso 2: Instalar Visual Studio Code y PlatformIO
1. El navegador debe abrir `https://code.visualstudio.com/`, descargar el instalador de Windows e instalar **Visual Studio Code** aceptando las opciones por defecto.
2. El usuario debe abrir **Visual Studio Code**.
3. El usuario debe hacer clic en el ícono de **Extensiones** en la barra lateral izquierda (los cuatro cuadrados).
4. El usuario debe escribir **`PlatformIO IDE`** en el buscador y presionar el botón **`Install`**.
5. La instalación descargará las herramientas automáticamente durante `3` a `5` minutos.
6. La ventana mostrará un aviso abajo a la derecha pidiendo reiniciar (`Reload Now`); el usuario debe hacer clic en **`Reload Now`** o cerrar y volver a abrir **Visual Studio Code**.

### Paso 3: Verificar el driver USB de la ESP32 en Windows
1. El usuario debe conectar una placa `ESP32` a la computadora usando un **cable USB de datos** (los cables que solo cargan celular no sirven).
2. El usuario debe dar clic derecho en el botón de **Inicio** de Windows y abrir **Administrador de dispositivos**.
3. El usuario debe expandir la sección **`Puertos (COM y LPT)`**.
4. La lista debe mostrar un puerto como **`Silicon Labs CP210x USB to UART Bridge (COM3)`** o **`USB-SERIAL CH340 (COM4)`** (el número `COM` puede cambiar en cada computadora).
5. **Si no aparece ningún puerto `COM` (o aparece con un triángulo amarillo):**
   * El cable puede ser solo de carga; el usuario debe probar otro cable USB.
   * El chip de la placa puede requerir driver; el usuario debe buscar en internet **`CP210x Universal Windows Driver`** (de Silicon Labs) o **`CH341SER.EXE`** (de WCH), instalarlo y volver a conectar la placa.

---

## 4. Paso a paso para programar la `ESP32 WROOM` final del rover

*(Nota: Los motores deben estar desconectados o con las ruedas en el aire antes de conectar el USB).*

### Paso 1: Abrir el proyecto del rover en VS Code
1. El usuario debe desconectar cualquier otra `ESP32` y conectar **únicamente la `ESP32 WROOM` final del rover** por USB.
2. El usuario debe anotar qué número de **`COM`** apareció en el **Administrador de dispositivos** (por ejemplo `COM3`).
3. En **Visual Studio Code**, el usuario debe ir al menú superior **`File` → `Open Folder...`**.
4. El usuario debe entrar a la carpeta descomprimida y seleccionar **exactamente** la subcarpeta:
   `C:\rovert-o-system-main\firmware\esp32_onboard`
5. El usuario debe presionar **`Select Folder`**.
6. La barra azul inferior de **Visual Studio Code** tardará unos segundos en cargar el proyecto y mostrará **`env:esp32dev`**.

### Paso 2: Revisar `Config.h` del rover
1. En el panel izquierdo de **Visual Studio Code**, el usuario debe abrir `src` y dar clic en **`Config.h`**.
2. El archivo `firmware/esp32_onboard/src/Config.h` debe tener estos valores:
   ```cpp
   static const uint8_t GROUND_STATION_MAC[6] = {
       0x8C, 0x94, 0xDF, 0x70, 0x64, 0x34
   };
   constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;
   ```
3. Los pines `UART` hacia la `Raspberry Pi` deben quedar en `34` (`RX`) y `23` (`TX`) para `ESP32 WROOM` (no usar `16` ni `17` porque esos pines los usa el motor `fr`).

### Paso 3: Subir el programa y anotar la MAC real de esta placa
1. En la barra azul inferior de **Visual Studio Code**, el usuario debe hacer clic en la flecha **`→`** (**PlatformIO: Upload**).
2. La primera compilación tardará de `2` a `4` minutos porque descarga las librerías de `ESP32`.
3. **Si la terminal inferior se queda trabada diciendo `Connecting........`:**
   * El usuario debe mantener presionado el botón **`BOOT`** de la `ESP32` durante `2` segundos hasta que aparezca `Writing at 0x00010000...`, y luego soltarlo.
4. La terminal mostrará una línea parecida a esta durante la subida:
   ```text
   MAC: xx:xx:xx:xx:xx:xx
   ```
5. **El usuario debe copiar y anotar esa dirección `MAC` exacta** (seis pares de letras/números separados por dos puntos), porque es la MAC real de la `ESP32` del rover.
6. La terminal debe terminar en verde con **`[SUCCESS]`**.
7. El usuario debe desconectar el cable USB de la `ESP32` del rover y pegarle una etiqueta que diga **`ROVER`**.

*(Alternativa para ver la MAC si pasó muy rápido: el usuario debe abrir el ícono de la hormiga de PlatformIO en la barra izquierda → `Miscellaneous` → `PlatformIO Core CLI`, y ejecutar `python -m esptool --port COM3 read-mac`, cambiando `COM3` por su puerto).*

---

## 5. Paso a paso para actualizar la `ESP32-GS` (Estación Terrena)

### Paso 1: Abrir el proyecto de Ground Station en VS Code
1. El usuario debe conectar **únicamente la `ESP32-GS`** (la que lleva la antena monopolo de `2.4 GHz`, MAC `8C:94:DF:70:64:34`) por USB.
2. En **Visual Studio Code**, el usuario debe ir a **`File` → `Open Folder...`** y seleccionar **exactamente**:
   `C:\rovert-o-system-main\firmware\esp32_ground_station`
3. El usuario debe esperar a que la barra azul inferior muestre **`env:esp32dev`**.

### Paso 2: Escribir la MAC real del rover en `Config.h`
1. En el panel izquierdo, el usuario debe abrir `src` y dar clic en **`Config.h`**.
2. El archivo muestra estas líneas:
   ```cpp
   static const uint8_t ROVER_MAC[6] = {
       0x28, 0x05, 0xA5, 0x30, 0xE7, 0xA8
   };
   ```
3. El usuario debe reemplazar esos seis bytes por la **MAC que anotó en el Paso 3 de la Sección 4** (por ejemplo, si la MAC del rover fue `58:2A:BD:71:E5:E4`, el bloque queda `{ 0x58, 0x2A, 0xBD, 0x71, 0xE5, 0xE4 }`).
4. El usuario debe guardar el archivo presionando **`Ctrl + S`**.

### Paso 3: Subir el programa a la `ESP32-GS`
1. En la barra azul inferior, el usuario debe hacer clic en la flecha **`→`** (**PlatformIO: Upload**).
2. *(Si se queda en `Connecting........`, el usuario debe mantener presionado el botón **`BOOT`** hasta que empiece a escribir).*
3. La terminal debe terminar en verde con **`[SUCCESS]`**.
4. El usuario debe pegarle una etiqueta que diga **`GS - LAPTOP`** a esta placa.

---

## 6. Conexión física en el rover (cables entre Raspberry Pi y `ESP32-Onboard`)

La `ESP32 WROOM` del rover se conecta a la `Raspberry Pi 4` con **tres cables** (ambas tarjetas trabajan a `3.3 V`):

| Pin en `Raspberry Pi 4` | Dirección | Pin en `ESP32-Onboard (WROOM)` |
|---|---|---|
| **Pin físico 8** (`GPIO 14 / TX`) | $\longrightarrow$ | **`GPIO 34`** (`RX`) |
| **Pin físico 10** (`GPIO 15 / RX`) | $\longleftarrow$ | **`GPIO 23`** (`TX`) |
| **Pin físico 6** (`GND`) | $\longleftrightarrow$ | **`GND`** |

* **La `ESP32-GS` no lleva cables en sus pines:** La placa de Estación Terrena solo se conecta por USB a la laptop que corre la interfaz gráfica.

---

## 7. Cómo comprobar que todo quedó funcionando

1. **El orden de encendido importa:** El operador debe encender **primero la `ESP32-GS`** conectada a la laptop y abrir la interfaz gráfica (`main.py`), y **después encender la `ESP32-Onboard`** del rover.
2. **La prueba de confirmación:** En `2` a `3` segundos, el campo **`Acción en proceso`** de la interfaz gráfica debe cambiar de `Detenido` a **`En espera (Standby)`**.
3. **Si aparece `Enlace perdido (Standby)`:** El operador debe presionar el botón **`EN`** (reset) de la `ESP32-Onboard` con la `ESP32-GS` ya encendida, o presionar **`Iniciar misión`** en la interfaz gráfica.
4. **Si no cambia nada después de `5` segundos:**
   * El equipo debe verificar que la MAC escrita en `firmware/esp32_ground_station/src/Config.h` sea exactamente la MAC de la `ESP32-Onboard`.
   * El equipo debe verificar que ambos `Config.h` tengan `ESPNOW_WIFI_CHANNEL = 1`.
   * El equipo debe verificar que la antena monopolo esté bien atornillada y que ambas placas estén encendidas.
