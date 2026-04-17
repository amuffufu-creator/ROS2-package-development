#!/usr/bin/env python3
import csv, time, math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)

class OdomLogger(Node):
    def __init__(self, out_csv="run.csv"):
        super().__init__("odom_logger")
        self.out_csv = out_csv
        self.t0 = time.time()
        self.f = open(self.out_csv, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "x", "y", "theta", "v", "w"])
        self.create_subscription(Odometry, "/odom", self.cb, 50)
        self.get_logger().info(f"Logging /odom to {self.out_csv}. Ctrl+C to stop.")

    def cb(self, msg: Odometry):
        t = time.time() - self.t0
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.w.writerow([f"{t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{th:.4f}", f"{v:.4f}", f"{w:.4f}"])

    def destroy_node(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = OdomLogger("run.csv")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()