"""2D scan-to-scan Laser Odometry node: estimates odom -> base_footprint from /scan using ICP."""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from roverto_autonomy.calibration import load_calibration
from roverto_autonomy.protocol import CMD_START_MISSION

REAR_CROP_RAD = 135.0 * (math.pi / 180.0)  # Ignore rear arc (>135 deg) obstructed by the chassis


def icp_2d(src: np.ndarray, dst: np.ndarray, max_iter: int = 15, max_dist: float = 0.35):
    """Align 2D scan src to dst (Nx2) via point-to-line ICP. Returns (dx, dy, dyaw) of robot motion."""
    pts = src.copy()
    # Estimate local wall normals on dst from neighboring scan points
    tang = np.zeros_like(dst)
    tang[1:-1] = dst[2:] - dst[:-2]
    tang[0] = dst[1] - dst[0]
    tang[-1] = dst[-1] - dst[-2]
    norm_len = np.hypot(tang[:, 0], tang[:, 1])
    valid_n = (norm_len > 1e-4) & (norm_len < 0.40)
    normals = np.zeros_like(dst)
    normals[valid_n, 0] = -tang[valid_n, 1] / norm_len[valid_n]
    normals[valid_n, 1] = tang[valid_n, 0] / norm_len[valid_n]

    total_th = 0.0
    total_t = np.zeros(2)
    reg = 1e-5 * np.eye(3)  # Damping keeps ill-conditioned corridors stable
    for _ in range(max_iter):
        diff = pts[:, None, :] - dst[None, :, :]
        d2 = np.einsum('ijk,ijk->ij', diff, diff)
        idx = np.argmin(d2, axis=1)
        mask = (d2[np.arange(len(pts)), idx] < (max_dist * max_dist)) & valid_n[idx]
        if np.count_nonzero(mask) < 6:
            break
        p, q, n = pts[mask], dst[idx[mask]], normals[idx[mask]]
        # Point-to-line Gauss-Newton step on (q - p) . n = 0
        j = np.column_stack((n[:, 0], n[:, 1], p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]))
        r = np.sum((q - p) * n, axis=1)
        dx, dy, dth = np.linalg.solve(j.T @ j + reg, j.T @ r)
        ca, sa = math.cos(dth), math.sin(dth)
        rot = np.array([[ca, -sa], [sa, ca]])
        t = np.array([dx, dy])
        pts = (rot @ pts.T).T + t
        total_th += dth
        total_t = rot @ total_t + t
        if abs(dth) < 1e-5 and math.hypot(dx, dy) < 1e-5:
            break
    total_th = math.atan2(math.sin(total_th), math.cos(total_th))
    return float(total_t[0]), float(total_t[1]), total_th


class LaserOdomNode(Node):
    """Publishes /odom and odom -> base_footprint TF directly from 10 Hz /scan messages."""

    def __init__(self):
        super().__init__('laser_odom_node')
        cal = load_calibration()
        lidar_xyz = cal.get('lidar_xyz', [0.125, 0.0, 0.08])
        self.lx = float(lidar_xyz[0])
        self.ly = float(lidar_xyz[1])
        self.min_range = self.declare_parameter('min_valid_range', 0.12).value
        self.max_step_xy = self.declare_parameter('max_step_xy', 0.15).value
        self.max_step_yaw = self.declare_parameter('max_step_yaw', 0.35).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/mission_command', self.on_mission_cmd, 10)

        self.prev_pts = None
        self.prev_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.get_logger().info(f'Laser odometry active (lidar offset x={self.lx:.3f} m)')

    def on_mission_cmd(self, msg: Float32MultiArray):
        if len(msg.data) >= 1 and int(round(msg.data[0])) == CMD_START_MISSION:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.prev_pts = None
            self.prev_stamp = None
            self._publish(self.get_clock().now(), 0.0, 0.0)
            self.get_logger().info('INICIAR MISION: laser odometry reset to (0, 0, 0)')

    def _scan_to_points(self, msg: LaserScan) -> np.ndarray:
        """Convert valid forward/side LaserScan rays into Nx2 points in base_footprint frame."""
        n = len(msg.ranges)
        if n == 0:
            return np.empty((0, 2))
        raw = msg.angle_min + np.arange(n) * msg.angle_increment
        angles = np.arctan2(np.sin(raw), np.cos(raw))
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        r_min = max(float(msg.range_min), self.min_range)
        valid = (np.isfinite(ranges) & (ranges >= r_min) & (ranges <= msg.range_max)
                 & (np.abs(angles) <= REAR_CROP_RAD))
        angles, ranges = angles[valid], ranges[valid]
        # Downsample to <=150 rays so ICP stays under 2 ms on Raspberry Pi 4
        if angles.size > 150:
            step = max(1, angles.size // 150)
            angles, ranges = angles[::step], ranges[::step]
        px = self.lx + ranges * np.cos(angles)
        py = self.ly + ranges * np.sin(angles)
        return np.column_stack((px, py))

    def on_scan(self, msg: LaserScan):
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if now.nanoseconds == 0:
            now = self.get_clock().now()
        pts = self._scan_to_points(msg)
        if self.prev_pts is None or len(pts) < 10 or len(self.prev_pts) < 10:
            self.prev_pts = pts if len(pts) >= 10 else self.prev_pts
            self.prev_stamp = now
            self._publish(now, 0.0, 0.0)
            return

        dx, dy, dyaw = icp_2d(pts, self.prev_pts)
        self.prev_pts = pts
        dt = (now - self.prev_stamp).nanoseconds * 1e-9 if self.prev_stamp else 0.1
        self.prev_stamp = now

        # Reject implausible scan-matching jumps
        if math.hypot(dx, dy) > self.max_step_xy or abs(dyaw) > self.max_step_yaw:
            dx, dy, dyaw = 0.0, 0.0, 0.0

        mid = self.yaw + 0.5 * dyaw
        self.x += dx * math.cos(mid) - dy * math.sin(mid)
        self.y += dx * math.sin(mid) + dy * math.cos(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

        v = math.copysign(math.hypot(dx, dy), dx) / dt if dt > 1e-4 else 0.0
        w = dyaw / dt if dt > 1e-4 else 0.0
        self._publish(now, v, w)

    def _publish(self, now, v: float, w: float):
        stamp = now.to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LaserOdomNode()
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
