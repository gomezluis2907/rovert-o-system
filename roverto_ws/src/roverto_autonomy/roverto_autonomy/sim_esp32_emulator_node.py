"""Mission state machine: 'rotate angle_deg and advance distance_m' with reactive avoidance."""
import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt8

from roverto_autonomy.calibration import load_calibration
from roverto_autonomy.protocol import (
    CMD_MOVE, CMD_START_MISSION, CMD_STOP, CMD_STOP_MISSION)

STANDBY, EXECUTING, AVOIDING, GOAL_REACHED = 0, 1, 2, 3
CLEAR, AVOID_LEFT, AVOID_RIGHT, STOP, HOLD_HEADING = 0, 1, 2, 3, 4


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    """Extract yaw from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimEsp32Emulator(Node):
    """Executes the Rovert-O mission state machine (see docs/ICD.md and docs/ARCHITECTURE.md)."""

    def __init__(self):
        super().__init__('sim_esp32_emulator_node')
        cal = load_calibration()
        self.linear_speed = self.declare_parameter(
            'linear_speed', float(cal['linear_speed'])).value
        self.angular_speed = self.declare_parameter(
            'angular_speed', float(cal['angular_speed'])).value
        self.k_yaw = self.declare_parameter('k_yaw', 1.5).value
        self.yaw_tolerance = self.declare_parameter('yaw_tolerance_rad', 0.04).value
        self.pass_distance = self.declare_parameter('pass_distance', 0.35).value
        self.goal_hold_sec = self.declare_parameter('goal_hold_sec', 3.0).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(UInt8, '/rover_status', 10)
        self.create_subscription(Float32, '/gs_command', self.on_command, 10)
        self.create_subscription(Float32MultiArray, '/mission_command', self.on_mission_command, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(UInt8, '/obstacle_state', self.on_obstacle, 10)
        self.create_timer(0.05, self.on_timer)  # 20 Hz

        self.pose: Optional[tuple] = None  # (x, y, yaw)
        self.obstacle = CLEAR
        self.status = STANDBY
        self.target = 0.0
        self.start_pose = (0.0, 0.0, 0.0)
        self.dist_along = 0.0
        self.turning_to_heading = False
        self.pass_pending = False       # an avoid-turn happened; a forward pass is owed
        self.pass_start: Optional[tuple] = None
        self.goal_time = None
        self._last_mission_cmd_mono = 0.0
        self.get_logger().info(
            'Mission tracker ready. Accepts /mission_command (ICD 0x01) and /gs_command.')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_obstacle(self, msg: UInt8):
        self.obstacle = msg.data

    def _reset_to_standby(self, log_msg: str):
        self.status = STANDBY
        self.target = 0.0
        self.dist_along = 0.0
        self.turning_to_heading = False
        self.pass_pending = False
        self.pass_start = None
        self.goal_time = None
        self._publish(0.0, 0.0)
        self.get_logger().info(log_msg)

    def _apply_command(self, cmd_id: int, distance_m: float, angle_deg: float):
        if cmd_id == CMD_STOP:
            self._reset_to_standby('DETENER command received -> STANDBY (mission active)')
            return
        if cmd_id == CMD_START_MISSION:
            self._reset_to_standby('INICIAR MISION received -> reset state to STANDBY')
            return
        if cmd_id == CMD_STOP_MISSION:
            self.target = 0.0
            self.turning_to_heading = False
            self.pass_pending = False
            self.pass_start = None
            self.status = GOAL_REACHED
            self.goal_time = self.get_clock().now()
            self._publish(0.0, 0.0)
            self.get_logger().info('DETENER MISION received -> sending final map and stopping')
            return
        if cmd_id != CMD_MOVE or (distance_m <= 0.0 and abs(angle_deg) < 1e-3):
            self._reset_to_standby('STOP command received -> STANDBY')
            return
        if self.pose is None:
            self.get_logger().warn('No /odom yet; ignoring command')
            return

        target_yaw = wrap_angle(self.pose[2] + math.radians(float(angle_deg)))
        self.start_pose = (self.pose[0], self.pose[1], target_yaw)
        self.target = max(0.0, float(distance_m))
        self.dist_along = 0.0
        self.turning_to_heading = abs(angle_deg) >= 1.0
        self.pass_pending = False
        self.pass_start = None
        self.goal_time = None
        self.status = EXECUTING
        self.get_logger().info(
            f'Move command: turn {angle_deg:+.1f} deg, advance {self.target:.2f} m '
            f'(from x={self.pose[0]:.2f} y={self.pose[1]:.2f} yaw={math.degrees(self.pose[2]):.1f} deg)')

    def on_mission_command(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        self._last_mission_cmd_mono = time.monotonic()
        cmd_id = int(round(msg.data[0]))
        dist = float(msg.data[1])
        angle = float(msg.data[2])
        self._apply_command(cmd_id, dist, angle)

    def on_command(self, msg: Float32):
        # Ignore duplicate legacy message emitted by esp32_bridge_node in the same tick
        if (time.monotonic() - self._last_mission_cmd_mono) < 0.05:
            return
        if msg.data <= 0.0:
            self._apply_command(CMD_STOP, 0.0, 0.0)
        else:
            self._apply_command(CMD_MOVE, float(msg.data), 0.0)

    def _publish(self, lin: float, ang: float):
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.cmd_pub.publish(cmd)
        self.status_pub.publish(UInt8(data=self.status))

    def on_timer(self):
        now = self.get_clock().now()
        if self.status == GOAL_REACHED:
            self._publish(0.0, 0.0)
            if self.goal_time is not None and (now - self.goal_time).nanoseconds * 1e-9 \
                    >= self.goal_hold_sec:
                self.status = STANDBY
            return
        if self.status == STANDBY or self.pose is None:
            self._publish(0.0, 0.0)
            return

        x, y, yaw = self.pose
        x0, y0, yaw0 = self.start_pose

        # Phase 1: In-place angular rotation when angle_deg was commanded
        if self.turning_to_heading:
            err = wrap_angle(yaw0 - yaw)
            if abs(err) <= self.yaw_tolerance:
                self.turning_to_heading = False
                self.start_pose = (x, y, yaw0)
                if self.target <= 0.0:
                    self.status = GOAL_REACHED
                    self.goal_time = now
                    self.get_logger().info(
                        f'GOAL_REACHED: completed turn to yaw={math.degrees(yaw):.1f} deg')
                    self._publish(0.0, 0.0)
                    return
            else:
                self.status = EXECUTING
                ang = self.angular_speed if err > 0.0 else -self.angular_speed
                self._publish(0.0, ang)
                return

        # Phase 2: Longitudinal advance along yaw0 with reactive obstacle avoidance
        self.dist_along = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)

        if self.dist_along >= self.target:
            self.status = GOAL_REACHED
            self.goal_time = now
            self.get_logger().info(f'GOAL_REACHED: progress {self.dist_along:.2f} m')
            self._publish(0.0, 0.0)
            return

        obs = self.obstacle
        lin, ang = 0.0, 0.0
        if obs == STOP:
            self.status = EXECUTING
        elif obs in (AVOID_LEFT, AVOID_RIGHT):
            self.status = AVOIDING
            self.pass_pending = True
            self.pass_start = None
            ang = self.angular_speed if obs == AVOID_LEFT else -self.angular_speed
        else:
            self.status = EXECUTING
            if self.pass_pending:
                if self.pass_start is None:
                    self.pass_start = (x, y)
                travelled = math.hypot(x - self.pass_start[0], y - self.pass_start[1])
                if travelled >= self.pass_distance:
                    self.pass_pending = False
                    self.pass_start = None
                lin = self.linear_speed  # straight along the deflected heading
            elif obs == HOLD_HEADING:
                lin = self.linear_speed
            else:  # CLEAR: realign with the target heading
                lin = self.linear_speed
                ang = max(-self.angular_speed,
                          min(self.angular_speed, self.k_yaw * wrap_angle(yaw0 - yaw)))
        self._publish(lin, ang)


def main(args=None):
    rclpy.init(args=args)
    node = SimEsp32Emulator()
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
