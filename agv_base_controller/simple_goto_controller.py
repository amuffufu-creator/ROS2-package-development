#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


# -------------------------
# Utils
# -------------------------
def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


# Simple exponential low-pass filter
class LowPass:
    def __init__(self, alpha: float, init: float = 0.0):
        self.alpha = float(alpha)
        self.state = float(init)

    def filt(self, x: float) -> float:
        self.state = self.alpha * x + (1.0 - self.alpha) * self.state
        return self.state


# Slew limiter (rate limiter)
class SlewLimiter:
    def __init__(self, rate_per_s: float, initial: float = 0.0):
        self.rate = abs(float(rate_per_s))  # units per second
        self.state = float(initial)
        self.last_t = time.time()

    def update(self, target: float) -> float:
        now = time.time()
        dt = max(1e-6, now - self.last_t)
        max_delta = self.rate * dt
        delta = target - self.state
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        self.state += delta
        self.last_t = now
        return self.state


# -------------------------
# Go-To-Pose Controller Node
# -------------------------
class GoToPoseController(Node):
    ROTATE_TO_AIM = 1
    MOVE_TO_GOAL = 2
    FINAL_ROTATE = 3
    DONE = 4

    def __init__(self):
        super().__init__('go_to_pose_controller')

        # ---- required params ----
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_theta_deg', 0.0)

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_theta = deg2rad(float(self.get_parameter('target_theta_deg').value))

        # ---- tunable params (expose as ROS params if you want later) ----
        # Gains
        self.kp_linear = 0.7            # v = kp_linear * dist
        self.kp_rotate = 1.6            # omega in rotate states
        self.kp_track = 1.0             # omega in move state (gentle heading correction)

        # Limits
        self.max_linear = 1.2           # m/s
        self.max_angular = 0.9          # rad/s

        # Tolerances
        self.dist_tolerance = 0.05      # m
        self.aim_tol = deg2rad(4.0)     # rad: finish State 1
        self.theta_tolerance = deg2rad(5.0)  # rad: finish State 3

        # Hysteresis / safety switching
        self.reaim_threshold = deg2rad(20.0)     # if heading error too big while moving -> back to rotate
        self.pos_reacquire_factor = 1.2          # if drift away in final rotate -> go back to move

        # Anti-stiction / kick-start
        self.min_linear = 0.15         # m/s (only in MOVE)
        self.min_rot = 0.4             # rad/s (only in rotate states)

        # Motion shaping
        self.boost_time = 0.15
        self.boost_linear = 0.30

        # LPF for angle errors (reduce jitter from encoder odom)
        self.aim_lpf = LowPass(alpha=0.25, init=0.0)
        self.final_lpf = LowPass(alpha=0.25, init=0.0)

        # Slew limiters
        self.linear_slew = SlewLimiter(rate_per_s=1.0, initial=0.0)
        self.angular_slew = SlewLimiter(rate_per_s=2.0, initial=0.0)

        # State
        self.state = self.ROTATE_TO_AIM
        self._last_state = None
        self.goal_reached_logged = False

        # Motion start detection (for boost)
        self.prev_cmd = Twist()
        self.last_motion_time = 0.0

        # Robot pose (from /odom)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.have_odom = False

        # ROS interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"GoToPose started. Target=({self.target_x:.3f}, {self.target_y:.3f}, "
            f"{self.get_parameter('target_theta_deg').value:.1f} deg)"
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation

        # yaw from quaternion (assuming planar motion)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.current_theta = math.atan2(siny, cosy)

        self.have_odom = True

    def _log_state_change(self, dist: float, err_aim: float, err_final: float):
        if self.state != self._last_state:
            name = {
                self.ROTATE_TO_AIM: "ROTATE_TO_AIM",
                self.MOVE_TO_GOAL: "MOVE_TO_GOAL",
                self.FINAL_ROTATE: "FINAL_ROTATE",
                self.DONE: "DONE",
            }.get(self.state, str(self.state))

            self.get_logger().info(
                f"[STATE] -> {name} | dist={dist:.3f} | err_aim={err_aim:.3f} | err_final={err_final:.3f}"
            )
            self._last_state = self.state

    def control_loop(self):
        if not self.have_odom:
            # don't command anything until /odom is alive
            self.cmd_vel_pub.publish(Twist())
            return

        # --- compute geometry to goal ---
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.hypot(dx, dy)

        # Bearing to goal (aim)
        aim = math.atan2(dy, dx) if dist > 1e-6 else self.current_theta
        raw_err_aim = normalize_angle(aim - self.current_theta)
        err_aim = self.aim_lpf.filt(raw_err_aim)

        # Final heading error
        raw_err_final = normalize_angle(self.target_theta - self.current_theta)
        err_final = self.final_lpf.filt(raw_err_final)

        # State transitions (logic first, command later)
        if self.state == self.ROTATE_TO_AIM:
            # If already at position, skip to final rotate
            if dist < self.dist_tolerance:
                self.state = self.FINAL_ROTATE
            elif abs(err_aim) < self.aim_tol:
                self.state = self.MOVE_TO_GOAL

        elif self.state == self.MOVE_TO_GOAL:
            if dist < self.dist_tolerance:
                self.state = self.FINAL_ROTATE
            elif abs(err_aim) > self.reaim_threshold:
                # heading too wrong -> stop moving, re-aim
                self.state = self.ROTATE_TO_AIM

        elif self.state == self.FINAL_ROTATE:
            # If we drifted away, go back to move
            if dist > self.dist_tolerance * self.pos_reacquire_factor:
                self.state = self.MOVE_TO_GOAL
            elif abs(err_final) < self.theta_tolerance:
                self.state = self.DONE

        # DONE condition per your requirement: dist < dist_tol AND |theta_err_final| < theta_tol
        if dist < self.dist_tolerance and abs(err_final) < self.theta_tolerance:
            self.state = self.DONE

        self._log_state_change(dist, err_aim, err_final)

        # --- compute desired cmd for each state ---
        desired_lin = 0.0
        desired_rot = 0.0

        if self.state == self.ROTATE_TO_AIM:
            desired_lin = 0.0
            desired_rot = self.kp_rotate * err_aim

            # enforce min rotation to overcome stiction
            if abs(desired_rot) < self.min_rot:
                desired_rot = math.copysign(self.min_rot, desired_rot)

            desired_rot = max(-self.max_angular, min(self.max_angular, desired_rot))

        elif self.state == self.MOVE_TO_GOAL:
            # linear by distance
            desired_lin = self.kp_linear * dist
            if desired_lin < self.min_linear:
                desired_lin = self.min_linear
            desired_lin = min(desired_lin, self.max_linear)

            # gentle heading correction while moving
            desired_rot = self.kp_track * err_aim
            desired_rot = max(-self.max_angular, min(self.max_angular, desired_rot))

            # optional: limit turning while moving (avoid spinning)
            # (conservative clamp: angular not too big relative to linear)
            turn_to_linear_factor = 0.3
            max_allowed_rot = abs(desired_lin) * turn_to_linear_factor
            if max_allowed_rot < 0.1:
                max_allowed_rot = 0.1
            desired_rot = max(-max_allowed_rot, min(max_allowed_rot, desired_rot))

            # kick-start boost only for linear
            now = time.time()
            starting_motion = (
                abs(self.prev_cmd.linear.x) < 1e-3 and abs(self.prev_cmd.angular.z) < 1e-3 and
                (abs(desired_lin) > 1e-3 or abs(desired_rot) > 1e-3)
            )
            if starting_motion:
                self.last_motion_time = now
            if 0 <= (now - self.last_motion_time) < self.boost_time and desired_lin != 0.0:
                desired_lin = math.copysign(max(abs(desired_lin), self.boost_linear), desired_lin)

        elif self.state == self.FINAL_ROTATE:
            desired_lin = 0.0
            desired_rot = self.kp_rotate * err_final

            if abs(desired_rot) < self.min_rot:
                desired_rot = math.copysign(self.min_rot, desired_rot)

            desired_rot = max(-self.max_angular, min(self.max_angular, desired_rot))

        elif self.state == self.DONE:
            desired_lin = 0.0
            desired_rot = 0.0
            if not self.goal_reached_logged:
                self.get_logger().info("GOAL REACHED (pose): dist & theta within tolerances")
                self.goal_reached_logged = True

        else:
            desired_lin = 0.0
            desired_rot = 0.0

        # --- slew limiting and publish ---
        cmd = Twist()
        cmd.linear.x = self.linear_slew.update(desired_lin)
        cmd.angular.z = self.angular_slew.update(desired_rot)

        self.prev_cmd = cmd
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()