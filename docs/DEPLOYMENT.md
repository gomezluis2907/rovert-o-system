# Guía Paso a Paso: Despliegue en el Rover Real (`roverto_ws`)

Esta guía explica exactamente qué ejecutar en la **Raspberry Pi 4** (arriba del rover) y en la **Laptop de Estación Terrena**.

---

## Parte 1: Preparación Única en la Raspberry Pi 4 (Solo se hace una vez)

### 1.1 Dar permisos a los puertos seriales (UART y USB)
Ejecuta esto en la Raspberry Pi y luego **reinicia la Pi**:

```bash
sudo usermod -aG dialout $USER
```

### 1.2 Habilitar el puerto UART en los pines GPIO 14 (TX) y GPIO 15 (RX)
1. Abre el archivo de configuración de arranque de la Pi:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
2. Agrega estas dos líneas al final del archivo y guarda (`Ctrl+O`, `Enter`, `Ctrl+X`):
   ```ini
   enable_uart=1
   dtoverlay=disable-bt
   ```
3. Desactiva la consola serial de Linux en `cmdline.txt`:
   ```bash
   sudo sed -i 's/console=serial0,115200 //g' /boot/firmware/cmdline.txt
   ```
4. Reinicia la Raspberry Pi:
   ```bash
   sudo reboot
   ```

### 1.3 Descargar el driver completo del LiDAR LD19 y compilar `roverto_ws`
Con internet en la Raspberry Pi, ejecuta:

```bash
sudo apt update
sudo apt install -y python3-serial ros-jazzy-slam-toolbox ros-jazzy-xacro

cd ~/rovert-o-system/roverto_ws/src
rm -rf ldlidar_stl_ros2
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

cd ~/rovert-o-system/roverto_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## Parte 2: Conexiones Físicas en el Rover

Antes de encender el software, verifica estos 2 cables en la **Raspberry Pi 4**:

1. **LiDAR LD19 $\rightarrow$ USB de la Raspberry Pi:**
   * Conectado por adaptador USB-TTL. Aparecerá como `/dev/ttyUSB0`.
2. **ESP32-Onboard $\rightarrow$ GPIO UART de la Raspberry Pi (`/dev/ttyAMA0`):**
   * `TX` del ESP32 $\rightarrow$ **GPIO 15 (Pin físico 10, RX)** de la Pi.
   * `RX` del ESP32 $\rightarrow$ **GPIO 14 (Pin físico 8, TX)** de la Pi.
   * `GND` del ESP32 $\rightarrow$ **GND (Pin físico 6)** de la Pi.

---

## Parte 3: Encender el Rover (En la Raspberry Pi 4)

Conéctate a la Raspberry Pi (o abre una terminal en ella) y ejecuta:

```bash
cd ~/rovert-o-system/roverto_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch roverto_bringup real_rover.launch.xml \
  launch_lidar_driver:=true \
  lidar_port:=/dev/ttyUSB0 \
  serial_port:=/dev/ttyAMA0
```

### ¿Qué levanta este único comando?
1. `robot_state_publisher`: Publica las dimensiones físicas del rover (URDF).
2. `ldlidar_stl_ros2_node`: Lee el LiDAR LD19 a `10 Hz` y publica `/scan`.
3. `lidar_guard_node`: Analiza `/scan` y decide si el camino está libre o hay obstáculo (`/obstacle_state`).
4. `esp32_bridge_node`:
   * Recibe **Packet 1** (ticks de encoders) desde el ESP32 y publica `/odom` + TF.
   * Envía **Packet 2** (estado de obstáculo) hacia el ESP32.
   * Envía **Packet 3** (fragmentos del mapa comprimido) hacia el ESP32.
5. `slam_toolbox`: Construye el mapa 2D (`/map`) a `0.10 m/celda` cada `3.0 s`.
6. `map_compressor_node`: Recorta y comprime `/map` en fragmentos listos para radio.

---

## Parte 4: Pruebas Aisladas en la Pi (Si falta hardware)

* **Probar SOLO el LiDAR y SLAM (sin ESP32 ni motores):**
  ```bash
  cd ~/rovert-o-system/roverto_ws
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  ros2 launch roverto_bringup lidar_test.launch.xml port_name:=/dev/ttyUSB0 use_rviz:=false
  ```
* **Probar el launch real sin tener el ESP32 conectado aún (`mock_serial`):**
  ```bash
  ros2 launch roverto_bringup real_rover.launch.xml mock_serial:=true launch_lidar_driver:=false
  ```

---

## Parte 5: Operar desde la Laptop (Estación Terrena)

1. Conecta el **ESP32-GS** por cable USB a tu Laptop (aparecerá como `/dev/ttyUSB0`).
2. Abre una terminal en tu Laptop y ejecuta la interfaz gráfica:
   ```bash
   cd ~/rovert-o-system
   GS_PORT=/dev/ttyUSB0 python3 ground_station_gui/GUI/main.py
   ```
3. Escribe la distancia en metros (ej. `5.0`) y presiona **Enviar**:
   * La orden viaja `Laptop -> USB -> ESP32-GS -> ESP-NOW -> ESP32-Onboard`.
   * El rover avanza y envía el mapa de regreso `Pi -> UART -> ESP32-Onboard -> ESP-NOW -> ESP32-GS -> USB -> GUI`.
