import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuUdpBridge(Node):
    def __init__(self):
        super().__init__('imu_udp_bridge')
        self.declare_parameter('imu_port', 8890)
        self.declare_parameter('imu_frame', 'imu_link')

        port = int(self.get_parameter('imu_port').value)
        self.frame = str(self.get_parameter('imu_frame').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.01)

        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.tick)  # 100Hz check loop

        self.drop_bad = 0
        self.drop_parse = 0
        self.rx_cnt = 0
        self.last_log_ns = 0

        self.get_logger().info(
            f"Listening UDP IMU on :{port} -> publishing /imu/data (frame_id={self.frame})"
        )

    def tick(self):
        try:
            data, addr = self.sock.recvfrom(512)
        except socket.timeout:
            return

        self.rx_cnt += 1
        line = data.decode(errors='ignore').strip()

        # Expect EXACTLY 12 fields:
        # i,ms,qx,qy,qz,qw,gx,gy,gz,ax,ay,az
        parts = line.split(',')
        if len(parts) != 12 or parts[0] != 'i':
            self.drop_bad += 1
            self._throttle_log(f"Drop bad format from {addr}: '{line}' (len={len(parts)})")
            return

        try:
            qx, qy, qz, qw = map(float, parts[2:6])
            gx, gy, gz     = map(float, parts[6:9])
            ax, ay, az     = map(float, parts[9:12])
        except ValueError:
            self.drop_parse += 1
            self._throttle_log(f"Drop parse error from {addr}: '{line}'")
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self.pub.publish(msg)

        # Throttle stats log ~1Hz
        self._throttle_log(f"RX={self.rx_cnt} drop_bad={self.drop_bad} drop_parse={self.drop_parse}")

    def _throttle_log(self, msg: str, period_ns: int = 1_000_000_000):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= period_ns:
            self.get_logger().info(msg)
            self.last_log_ns = now_ns

def main():
    rclpy.init()
    node = ImuUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()