"""
LiDAR 감시 노드 (E201 가림, E202 끊김). 판정은 core/scan_check.py.

구독: /scan
발행: /diagnostics, /field_toolkit/alarms (nodes/alarm_reporter.py)
"""

from dataclasses import fields

import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.scan_check import ScanChecker, ScanThresholds
from robot_field_toolkit.nodes.alarm_reporter import AlarmReporter
from sensor_msgs.msg import LaserScan


class SensorMonitor(Node):
    """스캔 내용과 수신 주기로 LiDAR 이상을 알린다."""

    def __init__(self):
        super().__init__('sensor_monitor')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('alarm_codes_file', '')
        defaults = ScanThresholds()
        values = {}
        for f in fields(ScanThresholds):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(f.name).value
        self.checker = ScanChecker(ScanThresholds(**values))
        self.reporter = AlarmReporter(self, 'sensor', self.get_parameter('alarm_codes_file').value)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan, 10)
        self.create_timer(0.1, self.evaluate)
        self.get_logger().info('sensor_monitor 시작')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg):
        self.checker.update_scan(self.now_sec(), msg.ranges, msg.range_min, msg.range_max)

    def evaluate(self):
        t = self.now_sec()
        if t > 0.0:
            self.reporter.update(t, self.checker.evaluate(t))


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
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
