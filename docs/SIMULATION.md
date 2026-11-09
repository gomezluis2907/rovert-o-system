# Guía Paso a Paso: Simulación (`roverto_ws`)

Sigue estos pasos exactamente en orden. Cada **Terminal** es una ventana de terminal distinta.

---

## 0. Compilar el Workspace (Solo la primera vez o si cambias código)

Abre una terminal y ejecuta línea por línea:

```bash
cd ~/rovert-o-system/roverto_ws
touch src/ldlidar_stl_ros2/COLCON_IGNORE
rm -rf build/ldlidar_stl_ros2 install/ldlidar_stl_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> **Nota:** `COLCON_IGNORE` desactiva el driver físico del LiDAR porque en simulación Gazebo genera su propio láser virtual.

---

## 1. Iniciar la Simulación (Gazebo + SLAM + RViz2)

En la **Terminal 1**, ejecuta:

```bash
cd ~/rovert-o-system/roverto_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch roverto_bringup sim_mission.launch.xml
```

* Espera **15 segundos** hasta que se abran **Gazebo** (ventana 3D con el rover) y **RViz2** (visor del mapa).
* **No cierres esta terminal.**

*(Si tu PC va lenta y no quieres ver la ventana 3D de Gazebo, usa en su lugar: `ros2 launch roverto_bringup sim_mission.launch.xml headless:=true`).*

---

## 2. Recibir el Mapa (Elige Opción A u Opción B)

### Opción A: Guardar el mapa como imagen PNG (Sin GUI)
Abre una **Terminal 2** y ejecuta:

```bash
cd ~/rovert-o-system/roverto_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 ~/rovert-o-system/ground_station_gui/gs_map_decoder.py --ros -o /tmp/mapa.png
```
* **No cierres esta terminal.** Cuando llegue el mapa, se guardará en `/tmp/mapa.png`.

### Opción B: Ver el mapa en vivo dentro de la Interfaz Gráfica (GUI)
Si quieres usar la ventana de la Estación Terrena (`main.py`) conectada a la simulación:

1. En la **Terminal 2** (puente virtual ROS 2 $\leftrightarrow$ Serial):
   ```bash
   cd ~/rovert-o-system/roverto_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   python3 /tmp/sim_gui_bridge.py
   ```
2. En una **Terminal 3** (Interfaz Gráfica):
   ```bash
   cd ~/rovert-o-system
   GS_PORT=/tmp/virtual_esp32_gs python3 ground_station_gui/GUI/main.py
   ```

---

## 3. Ordenar al Rover que Avance (5 metros)

* **Si usaste la Opción B (GUI):** Escribe `5.0` en el campo **Distancia [m]** de la ventana y haz clic en **Enviar**.
* **Si usaste la Opción A (Sin GUI):** Abre una **Terminal 3** y ejecuta:

```bash
cd ~/rovert-o-system/roverto_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub -1 /gs_command std_msgs/msg/Float32 "{data: 5.0}"
```

### ¿Qué debe pasar ahora?
1. El rover avanza a `0.04 m/s` (4 cm/s), esquiva obstáculos solo y actualiza el mapa en **RViz2** (resolución `0.10 m/celda`).
2. Cada 10 segundos (y al llegar a los 5 metros), `map_compressor_node` comprime y envía el mapa.
3. Si usaste la **Opción A**, abre la imagen generada con:
   ```bash
   xdg-open /tmp/mapa.png
   ```

---

## 4. Cómo Apagar Todo Limpiamente

1. Presiona `Ctrl + C` en cada terminal abierta.
2. Si Gazebo se queda abierto en segundo plano, mátalo con:
   ```bash
   pkill -f "[g]z sim"
   ```
