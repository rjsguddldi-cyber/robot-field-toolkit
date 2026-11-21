"""
동작 감시 노드 (E101~E104). 판정은 core/motion_check.py가 하고 이 노드는 껍데기.

구독: 명령(cmd_topic, 기본 /cmd_vel_checked = 인터락 통과 명령), /wheel_cmd, /odom
발행: /diagnostics, /field_toolkit/alarms (nodes/alarm_reporter.py)
"""

from dataclasses import fields

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.motion_check import MotionChecker, MotionThresholds
from robot_field_toolkit.nodes.alarm_reporter import AlarmReporter


class MotionMonitor(Node):
    """명령과 실제 움직임을 비교해서 구동 알람을 낸다."""

    def __init__(self):
        super().__init__('motion_monitor')
        # 명령 기준: 인터락이 막은 명령 때문에 E101이 뜨지 않도록 인터락 통과 명령을 본다.
        self.declare_parameter('cmd_topic', 'cmd_vel_checked')
        self.declare_parameter('wheel_cmd_topic', 'wheel_cmd')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('alarm_codes_file', '')
        self.declare_parameter('eval_rate', 10.0)

        self.checker = MotionChecker(self.thresholds_from_params(MotionThresholds))
        self.reporter = AlarmReporter(self, 'motion', self.get_parameter('alarm_codes_file').value)

        cmd_topic = self.get_parameter('cmd_topic').value
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(
            Twist, self.get_parameter('wheel_cmd_topic').value, self.on_wheel_cmd, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_timer(1.0 / self.get_parameter('eval_rate').value, self.evaluate)
        self.get_logger().info(f'motion_monitor 시작, 명령 기준: {cmd_topic}')

    def thresholds_from_params(self, cls):
        """기준값 dataclass의 칸 이름을 그대로 파라미터 이름으로 선언하고 읽는다."""
        defaults = cls()
        values = {}
        for f in fields(cls):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(f.name).value
        return cls(**values)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.checker.update_command(self.now_sec(), msg.linear.x, msg.angular.z)

    def on_wheel_cmd(self, msg):
        self.checker.update_wheel_cmd(self.now_sec())

    def on_odom(self, msg):
        tw = msg.twist.twist
        self.checker.update_odom(self.now_sec(), tw.linear.x, tw.angular.z)

    def evaluate(self):
        t = self.now_sec()
        if t > 0.0:  # /clock을 받기 전에는 0
            self.reporter.update(t, self.checker.evaluate(t))


def main(args=None):
    rclpy.init(args=args)
    node = MotionMonitor()
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
