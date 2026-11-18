"""ESP32 UART bridge (see docs/ICD.md):
  ESP32 -> Pi: 0x55 ENCODERS (12 B), 0x01 COMMAND (13 B, or legacy 0x88 8 B)
  Pi -> ESP32: 0x66 DRIVE_STATE (4 B), 0x90 PI_TELEMETRY (10 B), 0x06 MAP (<=205 B)
"""
from collections import deque
import math
import os
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ByteMultiArray, Float32, Float32MultiArray, UInt8
from tf2_ros import TransformBroadcaster

from roverto_autonomy.calibration import load_calibration
from roverto_autonomy.protocol import (
    CMD_MOVE, msg_data_to_bytes, pack_packet2, pack_pi_telemetry,
    Packet1Parser, unpack_packet3)

try:
    import serial  # pyserial (apt: python3-serial)
except ImportError:  # handled at runtime: node stays alive and reports the problem
    serial = None


class MockSerial:
    """In-memory loopback port. inject() feeds bytes the node will read (ESP32 -> Pi);
    everything the node writes (Pi -> ESP32) is recorded with a timestamp in .writes."""

    def __init__(self):
        self._rx = bytearray()
        self.writes = []  # list of (monotonic_time, bytes)
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def inject(self, data: bytes):
        self._rx.extend(data)

    def read(self, n: int = 1) -> bytes:
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def write(self, data: bytes) -> int:
        self.writes.append((time.monotonic(), bytes(data)))
        return len(data)

    def close(self):
        self.is_open = False

    @property
    def written(self) -> bytes:
        return b''.join(w for _, w in self.writes)


class DiffDriveOdometry:
    """Midpoint (2nd-order Runge-Kutta) differential-drive integration."""

    def __init__(self, ticks_per_meter: float, wheel_separation: float):
        self.tpm = ticks_per_meter
        self.sep = wheel_separation
        self.x = self.y = self.yaw = 0.0

    def update(self, dl_ticks: int, dr_ticks: int):
        """Integrate tick deltas. Returns (distance, dtheta) for this step."""
        dl = dl_ticks / self.tpm
        dr = dr_ticks / self.tpm
        d = 0.5 * (dl + dr)
        dth = (dr - dl) / self.sep
        mid = self.yaw + 0.5 * dth
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))
        return d, dth


def _read_cpu_temp_c() -> int:
    """Read Raspberry Pi / Linux SoC temperature in Celsius (0 if unavailable)."""
    path = '/sys/class/thermal/thermal_zone0/temp'
    try:
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                return max(0, min(255, int(round(int(f.read().strip()) / 1000.0))))
    except Exception:
        pass
    return 0


def _read_proc_stat():
    """Return (idle_ticks, total_ticks) from /proc/stat or None."""
    try:
        with open('/proc/stat', 'r', encoding='utf-8') as f:
            parts = f.readline().split()
        if parts and parts[0] == 'cpu':
            vals = [int(x) for x in parts[1:]]
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            return idle, sum(vals)
    except Exception:
        pass
    return None


def _read_ram_mb() -> int:
    """Return used system RAM in MB from /proc/meminfo (0 if unavailable)."""
    try:
        total_kb = avail_kb = None
        with open('/proc/meminfo', 'r', encoding='utf-8') as f:
            for line in f:
                if line.startswith('MemTotal:'):
                    total_kb = int(line.split()[1])
                elif line.startswith('MemAvailable:'):
                    avail_kb = int(line.split()[1])
                if total_kb is not None and avail_kb is not None:
                    return max(0, min(65535, int(round((total_kb - avail_kb) / 1024.0))))
    except Exception:
        pass
    return 0


class Esp32BridgeNode(Node):
    """See docs/ICD.md for packet formats."""

    CHUNK_INTERVAL = 0.05  # s, minimum spacing between 0x06 MAP writes (20 chunks/s)

    def __init__(self, **node_kwargs):
        super().__init__('esp32_bridge_node', **node_kwargs)
        cal = load_calibration()
        self.port = self.declare_parameter('serial_port', '/dev/ttyAMA0').value
        self.baud = self.declare_parameter('baud_rate', 115200).value
        self.tpm = self.declare_parameter('ticks_per_meter', float(cal['ticks_per_meter'])).value
        self.sep = self.declare_parameter('wheel_separation', float(cal['wheel_separation'])).value
        self.max_ticks = self.declare_parameter(
            'max_ticks_per_cycle', int(cal['max_ticks_per_cycle'])).value
        self.mock = self.declare_parameter('mock_serial', False).value
        # Disabled by default because laser_odom_node publishes /odom and odom -> base_footprint TF
        self.publish_odom = self.declare_parameter('publish_odom', False).value
        self.publish_tf = self.declare_parameter('publish_tf', False).value

        self.odom_calc = DiffDriveOdometry(self.tpm, self.sep)
        self.parser = Packet1Parser()
        self.prev_ticks = None
        self.last_stamp = None
        self.obstacle_state = 0
        self.rover_status = None
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.pi_velocity_mps = 0.0
        self.cpu_temp_c = _read_cpu_temp_c()
        self.cpu_pct = 0
        self.ram_mb = _read_ram_mb()
        self._prev_cpu = _read_proc_stat()
        self.last_scan_mono = 0.0
        self.map_queue = deque(maxlen=4000)
        self.ser = None
        self._open_serial()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(UInt8, '/rover_status', 10)
        self.cmd_pub = self.create_publisher(Float32, '/gs_command', 10)
        self.mission_cmd_pub = self.create_publisher(Float32MultiArray, '/mission_command', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(UInt8, '/obstacle_state', self.on_obstacle, 10)
        self.create_subscription(UInt8, '/rover_status', self.on_rover_status, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            ByteMultiArray, '/compressed_map_chunk', self.on_chunk,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.01, self.on_read)                  # 100 Hz non-blocking read
        self.create_timer(0.2, self.on_keepalive)              # 5 Hz 0x66 DRIVE_STATE keepalive
        self.create_timer(0.1, self.on_pi_telemetry)           # 10 Hz 0x90 PI_TELEMETRY
        self.create_timer(2.0, self.on_sample_pi_health)       # 0.5 Hz RPi CPU temp & load
        self.create_timer(self.CHUNK_INTERVAL, self.on_chunk_timer)  # paced 0x06 MAP chunks
        self.create_timer(2.0, self._reconnect_check)
        self.get_logger().info(
            f'ESP32 bridge: {"MOCK" if self.mock else self.port}@{self.baud}, '
            f'ticks/m={self.tpm}, sep={self.sep} m, clamp={self.max_ticks} ticks')

    # ---- serial plumbing ---------------------------------------------------------------
    def _open_serial(self):
        if self.mock:
            self.ser = MockSerial()
            return
        if serial is None:
            self.get_logger().error('pyserial missing: install python3-serial. UART disabled.')
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1)
            self.get_logger().info(f'Opened {self.port}')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f'Cannot open {self.port}: {e} (will retry)')

    def _reconnect_check(self):
        if self.ser is None and not self.mock and serial is not None:
            self._open_serial()

    def _write(self, data: bytes):
        if self.ser is None:
            return
        try:
            self.ser.write(data)
        except Exception as e:  # serial.SerialException, timeouts, unplugged device
            self.get_logger().error(f'UART write failed: {e}')
            try:
                self.ser.close()
            finally:
                self.ser = None

    # ---- ESP32 -> ROS ------------------------------------------------------------------
    def on_read(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b''
        except Exception as e:
            self.get_logger().error(f'UART read failed: {e}')
            self.ser = None
            return
        for left, right, status in self.parser.feed(data):
            self.handle_packet1(left, right, status)
        for cmd_id, dist, angle in self.parser.pop_commands():
            # Publish full ICD 0x01 COMMAND [cmd_id, distance_m, angle_deg] on /mission_command
            mcmd = Float32MultiArray()
            mcmd.data = [float(cmd_id), float(dist), float(angle)]
            self.mission_cmd_pub.publish(mcmd)
            # Also publish legacy /gs_command (Float32) when advancing or stopping
            if cmd_id == CMD_MOVE and dist > 0.0:
                self.cmd_pub.publish(Float32(data=float(dist)))
            elif cmd_id != CMD_MOVE:
                self.cmd_pub.publish(Float32(data=0.0))

    def handle_packet1(self, left: int, right: int, status: int):
        now = self.get_clock().now()
        self.status_pub.publish(UInt8(data=status))
        if self.prev_ticks is None:
            self.prev_ticks = (left, right)
            self.last_stamp = now
            self._publish_odom(now, 0.0, 0.0)
            return
        dl, dr = left - self.prev_ticks[0], right - self.prev_ticks[1]
        self.prev_ticks = (left, right)
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now
        if abs(dl) > self.max_ticks or abs(dr) > self.max_ticks:
            self.get_logger().warn('ESP32 tick jump/reset detected — clamping delta to 0')
            self._publish_odom(now, 0.0, 0.0)
            return
        d, dth = self.odom_calc.update(dl, dr)
        v, w = (d / dt, dth / dt) if dt > 1e-6 else (0.0, 0.0)
        if self.publish_odom:
            self.pi_velocity_mps = float(v)
        self._publish_odom(now, v, w)

    def _publish_odom(self, now, v: float, w: float):
        stamp = now.to_msg()
        x, y, yaw = self.odom_calc.x, self.odom_calc.y, self.odom_calc.yaw
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        cov = [0.0] * 36
        for i, val in ((0, 0.01), (7, 0.01), (14, 1e6), (21, 1e6), (28, 1e6), (35, 0.03)):
            cov[i] = val
        odom.pose.covariance = cov
        tcov = list(cov)
        tcov[0], tcov[7], tcov[35] = 0.01, 0.01, 0.03
        odom.twist.covariance = tcov
        if self.publish_odom:
            self.odom_pub.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ---- ROS -> ESP32 ------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        self.pi_velocity_mps = float(msg.twist.twist.linear.x)

    def on_scan(self, _msg: LaserScan):
        self.last_scan_mono = time.monotonic()

    def on_cmd_vel(self, msg: Twist):
        prev = self._effective_drive_state()
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_angular_z = float(msg.angular.z)
        curr = self._effective_drive_state()
        if curr != prev:
            self._write(pack_packet2(curr))

    def _effective_drive_state(self) -> int:
        """Send STOP (3) when STANDBY (0) or GOAL_REACHED (3); forward in-place turn (1/2)
        when mission tracker rotates for angle_deg while path is CLEAR."""
        if self.rover_status in (0, 3):
            return 3
        if self.obstacle_state == 0 and abs(self.cmd_linear_x) < 1e-4:
            if self.cmd_angular_z > 0.05:
                return 1  # turn left (CCW)
            if self.cmd_angular_z < -0.05:
                return 2  # turn right (CW)
        return self.obstacle_state

    def on_rover_status(self, msg: UInt8):
        prev = self._effective_drive_state()
        self.rover_status = msg.data
        curr = self._effective_drive_state()
        if curr != prev:
            self._write(pack_packet2(curr))

    def on_obstacle(self, msg: UInt8):
        prev = self._effective_drive_state()
        self.obstacle_state = msg.data
        curr = self._effective_drive_state()
        if curr != prev:
            self._write(pack_packet2(curr))

    def on_keepalive(self):
        self._write(pack_packet2(self._effective_drive_state()))

    def on_sample_pi_health(self):
        self.cpu_temp_c = _read_cpu_temp_c()
        self.ram_mb = _read_ram_mb()
        curr = _read_proc_stat()
        if curr is not None and self._prev_cpu is not None:
            d_idle = curr[0] - self._prev_cpu[0]
            d_tot = curr[1] - self._prev_cpu[1]
            if d_tot > 0:
                self.cpu_pct = max(0, min(100, int(round(100.0 * (1.0 - d_idle / d_tot)))))
        self._prev_cpu = curr

    def on_pi_telemetry(self):
        lidar_ok = (time.monotonic() - self.last_scan_mono) <= 1.0 if self.last_scan_mono > 0 else False
        pi_flags = 0x01 | (0x02 if lidar_ok else 0x00)
        self._write(pack_pi_telemetry(
            self.pi_velocity_mps, self.cpu_temp_c, self.cpu_pct, self.ram_mb, pi_flags))

    def on_chunk(self, msg: ByteMultiArray):
        frame = msg_data_to_bytes(msg.data)
        if unpack_packet3(frame) is None:
            self.get_logger().warn(f'Dropping invalid Packet 3 frame ({len(frame)} B)')
            return
        if len(self.map_queue) == self.map_queue.maxlen:
            self.get_logger().warn('Map chunk queue full; dropping oldest frame')
        self.map_queue.append(frame)

    def on_chunk_timer(self):
        if self.map_queue:
            self._write(self.map_queue.popleft())


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
