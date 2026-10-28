# Guía de Inicio y Configuración: Rovert-O System

Bienvenido al repositorio central de **Rovert-O**. Este documento contiene todas las instrucciones paso a paso para configurar tu entorno de trabajo en Windows, descargar el código, compilar el proyecto y subir tus cambios a GitHub sin complicaciones.

---

## Índice
1. [Instalaciones Iniciales](#1-instalaciones-iniciales)
2. [Configuración de Extensiones en VS Code](#2-configuración-de-extensiones-en-vs-code)
3. [Descargar el Repositorio (Clonar)](#3-descargar-el-repositorio-clonar)
4. [Probar el Hardware (Programa Blink en ESP32)](#4-probar-el-hardware-programa-blink-en-esp32)
5. [Solución de Problemas Comunes en Windows](#5-solución-de-problemas-comunes-en-windows)
6. [Cómo Guardar y Subir tus Cambios a GitHub](#6-cómo-guardar-y-subir-tus-cambios-a-github)

---

## 1. Instalaciones Iniciales

Antes de comenzar, necesitas instalar tres programas básicos en tu computadora:

### A. Git para Windows
Git es la herramienta que permite sincronizar el código con GitHub.
1. Descarga el instalador desde la página oficial: [https://git-scm.com/download/win](https://git-scm.com/download/win).
2. Abre el archivo descargado.
3. Puedes dejar todas las opciones predeterminadas dando clic en **Next** hasta finalizar la instalación (**Install**).

### B. Visual Studio Code (VS Code)
Es el editor de código donde trabajaremos.
1. Descárgalo desde: [https://code.visualstudio.com/Download](https://code.visualstudio.com/Download).
2. Ejecuta el instalador y sigue las instrucciones en pantalla.

### C. Controladores del ESP32 (Drivers USB)
Para que Windows reconozca tu placa ESP32 al conectarla por cable, necesitas instalar el controlador del chip de comunicación:
* Si tu placa usa el chip **CP2102**: Descarga el [Driver Silicon Labs CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) (versión *Universal Windows Driver*), descomprime el archivo ZIP y ejecuta el instalador.
* Si tu placa usa el chip **CH340 / CH341**: Descarga el [Driver WCH CH341SER](http://www.wch-ic.com/downloads/CH341SER_EXE.html) y presiona **INSTALL**.

---

## 2. Configuración de Extensiones en VS Code

Abre **Visual Studio Code** e instala las siguientes extensiones:

1. En la barra lateral izquierda de VS Code, haz clic en el icono de **Extensiones** (cuatro bloques pequeños juntos, o presiona `Ctrl + Shift + X`).
2. En la barra de búsqueda superior, escribe e instala cada una de las siguientes:
   * **PlatformIO IDE**: Permite programar, compilar y flashear microcontroladores como el ESP32. Haz clic en **Install** y espera unos minutos a que descargue sus herramientas internas.
   * **CMake Tools**: Da soporte para proyectos basados en CMake (utilizado en la parte de control y ROS 2).
   * **C/C++** (de Microsoft): Proporciona autocompletado y resaltado de sintaxis para código C y C++.

> **Nota:** Al terminar de instalar **PlatformIO IDE**, reinicia VS Code para asegurarte de que cargue correctamente.

---

## 3. Descargar el Repositorio (Clonar)

Una vez que hayas aceptado la invitación como colaborador del repositorio:

1. Abre **VS Code**.
2. Presiona en tu teclado las teclas: `Ctrl + Shift + P` para abrir la paleta de comandos.
3. Escribe:
   ```text
   Git: Clone
   ```
   y presiona `Enter`.
4. Pega el enlace oficial del repositorio:
   ```text
   https://github.com/gomezluis2907/rovert-o-system.git
   ```
5. Presiona `Enter`.
6. Selecciona una carpeta en tu computadora donde quieras guardarlo (por ejemplo, en `Documentos`).
7. Si te aparece una ventana emergente pidiendo iniciar sesión en GitHub, haz clic en **Sign in with your browser** y autoriza el acceso.
8. Al terminar la descarga, aparecerá un mensaje en la esquina inferior derecha. Haz clic en **Open** (Abrir).

---

## 4. Probar el Hardware (Programa Blink en ESP32)

> **REGLA IMPORTANTE PARA PLATFORMIO:**
> PlatformIO solo detecta el proyecto si abres directamente la carpeta del microcontrolador, no la carpeta principal de todo el robot.

### Paso 1: Abrir la carpeta del ESP32
1. En VS Code, ve al menú superior: **File** $\rightarrow$ **Open Folder...** (Archivo $\rightarrow$ Abrir carpeta).
2. Navega dentro del repositorio descargado y abre exactamente esta ruta:
   ```text
   rovert-o-system/firmware/esp32_onboard
   ```
3. Haz clic en **Seleccionar carpeta**.

### Paso 2: Conectar la placa
1. Conecta tu ESP32 a la computadora con un cable micro-USB o USB-C (asegúrate de que sea un cable de datos y no solo de carga).
2. Espera unos segundos a que Windows reconozca el dispositivo en el Administrador de dispositivos (aparecerá como un puerto `COM`, por ejemplo `COM3` o `COM4`).

### Paso 3: Compilar y Subir el Código
Mira la **barra de estado inferior** (la barra azul o morada en la parte inferior de VS Code):
* **Palomita ($\checkmark$):** Compila el código para verificar que no haya errores de sintaxis.
* **Flecha a la derecha ($\rightarrow$):** Compila y sube el firmware a tu ESP32.
* **Enchufe:** Abre el monitor serie para ver mensajes enviados por `Serial.print`.

Haz clic en la **Flecha a la derecha ($\rightarrow$)**:
1. PlatformIO compilará `src/main.cpp`.
2. Buscará automáticamente el puerto `COM` correspondiente y comenzará a subir el código.
3. Al terminar, verás un mensaje en color verde: `[SUCCESS]`.
4. El LED azul de tu ESP32 comenzará a parpadear cada medio segundo.

---

## 5. Solución de Problemas Comunes en Windows

* **Error: `Connecting........_____.....` y se detiene:**
  * Muchas placas ESP32 necesitan que actives manualmente el modo de carga. 
  * En cuanto veas aparecer los puntos `Connecting........` en la consola inferior de VS Code, **mantén presionado el botón físico "BOOT"** en la placa hasta que empiece a mostrar el porcentaje de escritura (`Writing at 0x...`), luego suéltalo.
* **Error: `Could not open port COM...`:**
  * Revisa que el cable USB esté bien conectado y que hayas instalado los controladores CP2102 o CH340 indicados en la [Sección 1](#c-controladores-del-esp32-drivers-usb).
  * Asegúrate de cerrar cualquier otro programa que esté usando el puerto serie (como Arduino IDE u otra ventana de monitor serie).

---

## 6. Cómo Guardar y Subir tus Cambios a GitHub

Puedes guardar y compartir tus cambios directamente desde VS Code sin escribir comandos en la terminal:

1. **Antes de empezar a trabajar (Descargar lo nuevo):**
   * En la barra lateral izquierda de VS Code, haz clic en el icono de **Control de código fuente** (tres círculos conectados por líneas, o presiona `Ctrl + Shift + G`).
   * Haz clic en los tres puntos (`...`) en la esquina superior del panel y selecciona **Pull** (o dale al icono del círculo con flechas para sincronizar).
2. **Modifica tu código:**
   * Haz tus cambios o pruebas en los archivos correspondientes dentro de `firmware/esp32_onboard/src/`.
3. **Guardar tus cambios (Commit):**
   * Ve de nuevo a la pestaña de **Control de código fuente** en la izquierda.
   * Verás una lista de los archivos que modificaste en la sección *Changes*.
   * Pasa el ratón sobre la palabra *Changes* y haz clic en el símbolo de más (**`+`**) para preparar todos los archivos (*Stage All Changes*).
   * En la caja de texto que dice *"Message"*, escribe brevemente qué cambiaste (por ejemplo: `feat: agregar lectura de sensor de distancia`).
   * Haz clic en el botón azul **Commit**.
4. **Subir a GitHub (Push):**
   * Haz clic en el botón azul que dice **Sync Changes** (Sincronizar cambios) o en los tres puntos (`...`) $\rightarrow$ **Push**.
   * Tus cambios se subirán inmediatamente al repositorio en GitHub.
