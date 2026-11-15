# ROVERT-O Workspace Architecture Guide

The `roverto_ws` folder is the ROS 2 workspace for the ROVERT-O robot. This guide explains how every package and node works in simple language.

---

## 1. The Big Picture

The ROVERT-O system uses three computers that work as a team. The Ground Station laptop shows the map and sends mission distance goals. The Onboard ESP32 microcontroller drives the four wheels and handles the wireless ESP-NOW radio link. The Raspberry Pi 4 runs ROS 2, reads the LD19 laser scanner, tracks how far the rover has moved, avoids obstacles, and builds the 2D map.

---

## 2. The Six Packages in `roverto_ws/src`

### 2.1 `roverto_description` (The Physical Blueprint)
The `roverto_description` package holds the physical measurements and 3D model of the rover. The file `config/rover_calibration.yaml` is the single source of truth for all measurements, speeds, and safety distances. The file `urdf/roverto.urdf.xacro` builds the 3D robot model with four wheels, a main chassis box, and a centered front shelf for the LiDAR sensor.

### 2.2 `roverto_gazebo` (The Simulation World)
The `roverto_gazebo` package runs the 3D physics simulator on a computer. The file `worlds/classroom.sdf` creates an 8-meter by 8-meter square classroom with tables and chairs. The file `worlds/tunnel.sdf` provides an alternative 10-meter tunnel world. The file `config/gazebo_bridge.yaml` connects Gazebo topics to ROS 2 topics.

### 2.3 `ldlidar_stl_ros2` (The Physical LiDAR Driver)
The `ldlidar_stl_ros2` package is the hardware driver for the physical LD19 spinning laser sensor. This driver reads serial data from the USB port at 230400 baud and publishes 10 laser scans per second to the `/scan` topic.

### 2.4 `roverto_navigation` (The Map Maker)
The `roverto_navigation` package configures `slam_toolbox` to build a 2D overhead map of the room. The configuration file `config/mapper_params_online_async.yaml` sets the map resolution to 0.10 meters per cell and updates the `/map` topic every 3.0 seconds.

### 2.5 `roverto_autonomy` (The Custom Python Brain)
The `roverto_autonomy` package contains the custom Python nodes that control perception, odometry, mission tracking, map compression, and serial communication:
- The `laser_odom_node.py` file compares consecutive laser scans from `/scan` to calculate how far the rover moved and rotated without wheel encoders. It publishes the position to `/odom` and broadcasts the `odom -> base_footprint` transform.
- The `lidar_guard_node.py` file checks `/scan` for nearby obstacles within 0.45 meters in front or 0.30 meters on the sides. It publishes a safety code (`0=CLEAR`, `1=AVOID_LEFT`, `2=AVOID_RIGHT`, `3=STOP`, `4=HOLD_HEADING`) to `/obstacle_state`.
- The `sim_esp32_emulator_node.py` file tracks the target mission distance (such as advancing 5.0 meters) using `/odom`. It publishes `/cmd_vel` in simulation and `/rover_status` (`0=STANDBY`, `1=EXECUTING`, `2=AVOIDING`, `3=GOAL_REACHED`) on both simulation and physical hardware.
- The `map_compressor_node.py` file watches `/map` and `/odom`. It crops, quantizes (2 bits per cell), and compresses the map with `zlib` every 0.5 meters of travel (at most once every 12.5 seconds, or 0.08 Hz) and when the goal is reached.
- The `esp32_bridge_node.py` file communicates with the onboard ESP32 over the Raspberry Pi UART port (`/dev/ttyAMA0` at 115200 baud). It receives Ground Station commands (Packet 4) from the ESP32 and sends drive states (Packet 2) and compressed map chunks (Packet 3) to the ESP32.

### 2.6 `roverto_bringup` (The Master Launch Files)
The `roverto_bringup` package provides three launch files to start the system:
- The `sim_mission.launch.xml` file starts the Gazebo classroom simulation, SLAM, obstacle guard, mission tracker, map compressor, and RViz2.
- The `real_rover.launch.xml` file starts the physical rover stack on the Raspberry Pi 4 with the LD19 LiDAR driver, Laser Odometry, mission tracker, UART bridge, SLAM, and map compressor.
- The `lidar_test.launch.xml` file runs a bench test for the physical LD19 LiDAR and SLAM without motors or the ESP32.

---

## 3. The Front-Mounted LiDAR Shelf and Rear Cropping

The LD19 LiDAR sits on a centered shelf at the front of the rover (`lidar_xyz: [0.125, 0.0, 0.08]` meters relative to `base_link`). Because the main chassis box sits directly behind the shelf, the rear laser beams between 135 degrees and 225 degrees hit the rover body. The launch configuration and autonomy nodes crop out the rear arc beyond 135 degrees and ignore hits closer than 0.12 meters. This cropping prevents the rover from mistaking its own chassis for a wall or obstacle.

---

## 4. The Binary UART Protocol (Raspberry Pi <-> Onboard ESP32)

The Raspberry Pi and Onboard ESP32 communicate over 115200 baud serial using compact frames protected by an 8-bit XOR checksum:
- **Packet 2 (`0xAA 0x66`, 4 bytes, Pi -> ESP32):** This packet sends the active driving or obstacle command (`0=CLEAR`, `1=AVOID_LEFT`, `2=AVOID_RIGHT`, `3=STOP`, `4=HOLD_HEADING`). The bridge sends `3=STOP` automatically whenever the mission tracker is in `STANDBY` or `GOAL_REACHED`.
- **Packet 3 (`0xAA 0x77`, up to 205 bytes, Pi -> ESP32):** This packet carries a compressed map chunk for the ESP32 to relay over ESP-NOW to the Ground Station.
- **Packet 4 (`0xAA 0x88`, 8 bytes, ESP32 -> Pi):** This packet forwards a Ground Station mission command (`cmd_id=1` with `distance_m` to advance, or `cmd_id=0` to stop) from the ESP32 up to the Raspberry Pi.
- **Unknown Packet IDs:** The parser skips any extra packet IDs added by the Electronics team so custom sensor telemetry never breaks navigation sync.
