"""
LiDAR 고장 주입 노드 (E2xx 시험용).

    Gazebo scan -> 브리지 -> /scan_raw -> [scan_fault_injector] -> /scan -> SLAM, Nav2, sensor_monitor

mode:
    normal  : 그대로 전달
    occlude : occlude_center_deg 중심 occlude_width_deg 구간을 +inf(측정 없음)로 (센서 창 가림)
    dropout : dropout_rate_hz 이하로만 전달 (0이면 완전히 끊김)

실행 중 변경: ros2 param set /scan_fault_injector mode occlude
"""

import math

import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.scan_check import occlude
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

KNOWN_MODES = ('normal', 'occlude', 'dropout')


class ScanFaultInjector(Node):
    """/scan_raw를 mode에 따라 변형해서 /scan으로 보낸다."""

    def __init__(self):
        super().__init__('scan_fault_injector')
        self.declare_parameter('mode', 'normal')
        self.declare_parameter('occlude_center_deg', 0.0)    # 0 = 로봇 정면
        self.declare_parameter('occlude_width_deg', 180.0)
        self.declare_parameter('dropout_rate_hz', 1.0)
        self.warned_mode = None
        self.last_sent_t = None

        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.mode_pub = self.create_publisher(String, '~/mode', 10)
        self.create_subscription(LaserScan, 'scan_raw', self.on_scan, 10)
        self.create_timer(1.0, lambda: self.mode_pub.publish(String(data=self.current_mode())))
        self.get_logger().info(f'scan_fault_injector 시작, mode={self.current_mode()}')

    def current_mode(self):
        mode = self.get_parameter('mode').value
        if mode in KNOWN_MODES:
            self.warned_mode = None
            return mode
        if mode != self.warned_mode:
            self.get_logger().warn(
                f"알 수 없는 mode '{mode}' -> normal로 동작. 가능: {', '.join(KNOWN_MODES)}")
            self.warned_mode = mode
        return 'normal'

    def on_scan(self, msg):
        mode = self.current_mode()
        if mode == 'occlude':
            msg.ranges = occlude(
                msg.ranges, msg.angle_min, msg.angle_increment,
                math.radians(self.get_parameter('occlude_center_deg').value),
                math.radians(self.get_parameter('occlude_width_deg').value))
        elif mode == 'dropout':
            rate = self.get_parameter('dropout_rate_hz').value
            t = self.get_clock().now().nanoseconds * 1e-9
            if rate <= 0.0:
                return
            if self.last_sent_t is not None and t - self.last_sent_t < 1.0 / rate:
                return
            self.last_sent_t = t
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFaultInjector()
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
