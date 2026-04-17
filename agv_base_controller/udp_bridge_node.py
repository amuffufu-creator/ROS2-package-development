#!/usr/bin/env python3
import math
import socket
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))  # x,y,z,w


class UdpBridgeNode(Node):
    def __init__(self):
        super().__init__('udp_bridge_node')
        self.get_logger().info('UDP Bridge Node Started (odom + debug)')

        # ---------- Params ----------
        self.declare_parameter('esp32_ip', '192.168.100.25') #192.168.100.25
        self.declare_parameter('esp32_port', 8889)  # ESP32 listens here
        self.declare_parameter('pc_port', 8888)     # PC listens here

        # Must match ESP32 geometry
        self.declare_parameter('wheel_base', 0.285)       # meters
        self.declare_parameter('wheel_radius', 0.0319)    # meters
        self.declare_parameter('ticks_per_rev', 330.0)    # ticks/rev

        self.declare_parameter('publish_tf', True)

        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = int(self.get_parameter('esp32_port').value)
        self.pc_port = int(self.get_parameter('pc_port').value)

        self.B = float(self.get_parameter('wheel_base').value)
        self.R = float(self.get_parameter('wheel_radius').value)
        self.N = float(self.get_parameter('ticks_per_rev').value)

        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.ticks_to_m = (2.0 * math.pi * self.R) / self.N
        self.mps_to_tps = self.N / (2.0 * math.pi * self.R)

        self.esp32_addr = (self.esp32_ip, self.esp32_port)

        # ---------- UDP ----------
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(('0.0.0.0', self.pc_port))
        self.rx.setblocking(False)

        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info(f"Listening on UDP {self.pc_port} for ESP32 packets")
        self.get_logger().info(f"Sending commands to {self.esp32_ip}:{self.esp32_port}")

        # handshake: ensures ESP32 learns PC IP even if controller not running yet
        try:
            self.tx.sendto(b"s,0,0\n", self.esp32_addr)
            self.get_logger().info("Handshake sent: s,0,0")
        except Exception as e:
            self.get_logger().warn(f"Handshake failed: {e}")

        # ---------- ROS ----------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # ---------- State ----------
        self.last_cmd = Twist()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_ticks_r = None
        self.last_ticks_l = None
        self.last_time = None

        # debug print throttle
        self._last_dbg_print_ns = 0

        # timers
        self.create_timer(0.02, self.send_cmd_timer)   # 50 Hz send commands
        self.create_timer(0.01, self.recv_timer)       # 100 Hz receive

    # ------------------ cmd_vel -> wheel ticks/s ------------------
    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd = msg

    def send_cmd_timer(self):
        v = float(self.last_cmd.linear.x)
        w = float(self.last_cmd.angular.z)

        v_r = v + 0.5 * self.B * w
        v_l = v - 0.5 * self.B * w

        tps_r = v_r * self.mps_to_tps
        tps_l = v_l * self.mps_to_tps

        cmd = f"s,{tps_r:.3f},{tps_l:.3f}\n".encode('ascii')
        try:
            self.tx.sendto(cmd, self.esp32_addr)
        except Exception as e:
            self.get_logger().warn(f"UDP send failed: {e}")

    # ------------------ receive packets ------------------
    def recv_timer(self):
        while True:
            ready, _, _ = select.select([self.rx], [], [], 0.0)
            if not ready:
                break
            data, _addr = self.rx.recvfrom(512)
            line = data.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            if line.startswith('o,'):
                self.process_o_packet(line)
            elif line.startswith('d,'):
                self.process_d_packet(line)

    # ------------------ odom packet: o,ticksA,ticksB (A=Right, B=Left by your firmware comment) ------------------
    def process_o_packet(self, line: str):
        parts = line.split(',')
        if len(parts) < 3:
            return
        try:
            # IMPORTANT: assumes ESP32 sends o,<rightTicks>,<leftTicks>
            ticks_r = int(parts[1])
            ticks_l = int(parts[2])
        except Exception:
            return

        now = self.get_clock().now()

        if self.last_ticks_r is None:
            self.last_ticks_r = ticks_r
            self.last_ticks_l = ticks_l
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 1e-6:
            return

        dtr = ticks_r - self.last_ticks_r
        dtl = ticks_l - self.last_ticks_l

        self.last_ticks_r = ticks_r
        self.last_ticks_l = ticks_l
        self.last_time = now

        ds_r = dtr * self.ticks_to_m
        ds_l = dtl * self.ticks_to_m

        ds = 0.5 * (ds_r + ds_l)
        dth = (ds_r - ds_l) / self.B

        th_mid = self.th + 0.5 * dth
        self.x += ds * math.cos(th_mid)
        self.y += ds * math.sin(th_mid)
        self.th = wrap(self.th + dth)

        v_est = ds / dt
        w_est = dth / dt

        self.publish_odom(now, v_est, w_est)

    def publish_odom(self, now, v_est: float, w_est: float):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)

        qx, qy, qz, qw = yaw_to_quat(self.th)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(v_est)
        odom.twist.twist.angular.z = float(w_est)

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ------------------ debug packet: d,ms,tA,tB,dA,dB,tgtA,tgtB,pwmA,pwmB,x,y,th ------------------
    def process_d_packet(self, line: str):
        # print at most 2 Hz
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_dbg_print_ns < int(0.5 * 1e9):
            return
        self._last_dbg_print_ns = now_ns

        parts = line.split(',')
        if len(parts) < 13:
            return

        try:
            ms   = int(parts[1])
            tA   = int(parts[2]); tB = int(parts[3])
            dA   = int(parts[4]); dB = int(parts[5])
            tgtA = float(parts[6]); tgtB = float(parts[7])
            pwmA = float(parts[8]); pwmB = float(parts[9])
            x    = float(parts[10]); y = float(parts[11]); th = float(parts[12])
        except Exception:
            return

        self.get_logger().info(
            f"[ESP DBG] ms={ms} | A={tA} B={tB} | dA={dA} dB={dB} | "
            f"tgtA={tgtA:.1f} tgtB={tgtB:.1f} | pwmA={pwmA:.0f} pwmB={pwmB:.0f} | "
            f"odom(x,y,th)=({x:.3f},{y:.3f},{th:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = UdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()