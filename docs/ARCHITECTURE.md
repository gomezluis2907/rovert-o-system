# Rovert-O System Architecture (Master Spec)

## Platform
| Item | Value |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Harmonic (`ros_gz`) |
| Onboard compute | Raspberry Pi 4B (LiDAR processing, `slam_toolbox`, reactive safety overrides; **no Nav2**) |
| Microcontroller | ESP32 (motor actuation, wheel encoders, ESP-NOW telemetry) |
| Sensor | LD19 2D LiDAR |

Design constraints: lightweight stack. No `robot_localization`, no `cmd_vel_mux`.

## TF Tree
```
map -> odom -> base_footprint -> base_link -> laser_frame
```
- `map -> odom`: `slam_toolbox`
- `odom -> base_footprint`: DiffDrive (sim) / `laser_odom_node` 2D scan-to-scan ICP (hardware)
- `base_footprint -> base_link -> laser_frame` (+ wheel links): `robot_state_publisher` from the xacro (with `laser_frame` mounted on the centered front shelf and rear `135°..225°` rays cropped)

## Single Source of Calibration
`roverto_description/config/rover_calibration.yaml` is the only place for physical constants
(wheel radius/offset, slip factor, wheel separation, LiDAR pose, `ticks_per_meter`,
tick clamp, safety distances, speeds). The xacro loads it via `xacro.load_yaml`.

## Binary Protocol (ESP-NOW & UART 115200 baud)
115200 baud, Little-Endian, no padding (`struct` format strings start with `<`).
`xor_checksum` = XOR of every preceding byte in the packet (header bytes included).
Full field-by-field specification and bitmasks are defined in [ICD.md](ICD.md).

### External Messages (ESP-NOW / USB / UART Passthrough)

| ID | Name | Direction | Rate | Bytes | Format | Summary |
|---|---|---|---|---|---|---|
| `0x01` | `COMMAND` | GS -> Rover | Event | `13 B` | `<BBBBffB` | `0xAA, 0x01, seq, cmd_id, distance_m, angle_deg, xor` (`0=STOP, 1=MOVE, 2=START_MISSION, 3=STOP_MISSION`) |
| `0x02` | `NAV` | Rover -> GS | `10 Hz` | `36 B` | `<BBBIfffffffB` | `0xAA, 0x02, seq, ts_ms, yaw, pitch, roll, ax, ay, az, velocity, xor` |
| `0x03` | `POWER` | Rover -> GS | `1 Hz` | `17 B` | `<BBBIffBB` | `0xAA, 0x03, seq, ts_ms, voltage_v, current_a, power_flags, xor` |
| `0x04` | `THERMAL` | Rover -> GS | `0.1 Hz` | `21 B` | `<BBBIfffBB` | `0xAA, 0x04, seq, ts_ms, temp_bat_c, temp_pcb_c, temp_lidar_c, thermal_flags, xor` |
| `0x05` | `HEALTH` | Rover -> GS | `0.5 Hz` | `15 B` | `<BBBIBBBBHBB` | `0xAA, 0x05, seq, ts_ms, rover_status, obstacle_state, rpi_cpu_temp_c, rpi_cpu_pct, rpi_ram_mb, health_flags, xor` |
| `0x06` | `MAP` | Rover -> GS | `<= 0.08 Hz` | `8..205 B` | `<BBBBBB` + payload + `B` | `0xAA, 0x06, map_seq_id, chunk_id, total_chunks, payload_len (<=198), payload, xor` |

### Internal UART Messages (Raspberry Pi <-> ESP32-Onboard)

#### `0x55` (`ENCODERS`): ESP32 -> Pi, 12 bytes, `<BBiiBB`
| Field | Type | Notes |
|---|---|---|
| sync | uint8 | `0xAA` |
| id | uint8 | `0x55` |
| left_ticks | int32 | cumulative |
| right_ticks | int32 | cumulative |
| rover_status | uint8 | 0=STANDBY, 1=EXECUTING, 2=AVOIDING, 3=GOAL_REACHED |
| xor_checksum | uint8 | |

#### `0x66` (`DRIVE_STATE`): Pi -> ESP32, 4 bytes, `<BBBB`
| Field | Type | Notes |
|---|---|---|
| sync | uint8 | `0xAA` |
| id | uint8 | `0x66` |
| obstacle_state | uint8 | 0=CLEAR, 1=AVOID_LEFT, 2=AVOID_RIGHT, 3=STOP, 4=HOLD_HEADING |
| xor_checksum | uint8 | |

#### `0x90` (`PI_TELEMETRY`): Pi -> ESP32, 12 bytes, `<BBfBBHBB` (10 Hz)
| Field | Type | Notes |
|---|---|---|
| sync | uint8 | `0xAA` |
| id | uint8 | `0x90` |
| velocity_mps | float32 | linear speed from `/odom` (`m/s`) |
| cpu_temp_c | uint8 | RPi SoC temperature (`°C`, limit `75 °C`) |
| cpu_pct | uint8 | RPi CPU load (`0..100 %`, limit `70 %`) |
| ram_mb | uint16 | RPi RAM used (`MB`, limit `2800 MB`) |
| pi_flags | uint8 | bit0=UART active, bit1=LiDAR `/scan` active |
| xor_checksum | uint8 | |

## Workspace Packages (`roverto_ws/src`)
| Package | Type | Role |
|---|---|---|
| `roverto_description` | ament_cmake | URDF/Xacro + `config/rover_calibration.yaml` |
| `roverto_gazebo` | ament_cmake | Worlds, `ros_gz_bridge` config, sim launch |
| `roverto_navigation` | ament_cmake | `slam_toolbox` config + launch |
| `roverto_bringup` | ament_cmake | Top-level launch (`sim_mission`, `real_rover`, `lidar_test`) |
| `roverto_autonomy` | ament_python | `laser_odom_node`, `lidar_guard_node`, `sim_esp32_emulator_node`, `esp32_bridge_node`, `map_compressor_node` |

## Simulation Topic Contract
| ROS topic | Type | Direction | Producer |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | GZ -> ROS | Gazebo |
| `/scan` | `sensor_msgs/LaserScan` | GZ -> ROS | `gpu_lidar` (frame `laser_frame`) |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS -> GZ | DiffDrive |
| `/odom` | `nav_msgs/Odometry` | GZ -> ROS | DiffDrive (`odom` -> `base_footprint`) |
| `/tf` | `tf2_msgs/TFMessage` | GZ -> ROS | DiffDrive |
| `/joint_states` | `sensor_msgs/JointState` | GZ -> ROS | JointStatePublisher |
