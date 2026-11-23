"""
동작 감시 노드 (E101~E105). 판정은 core/motion_check.py, core/slip_check.py.

구독: 명령(cmd_topic, 기본 /cmd_vel_checked = 인터락 통과 명령), /wheel_cmd, /odom,
      /scan (E105 바퀴 헛돎 교차 확인), /initialpose (위치 재설정 = E105 해제)
발행: /diagnostics, /field_toolkit/alarms (nodes/alarm_reporter.py)
"""

from dataclasses import fields

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.common import yaw_from_quaternion
from robot_field_toolkit.core.motion_check import MotionChecker, MotionThresholds
from robot_field_toolkit.core.slip_check import SlipChecker, SlipThresholds
from robot_field_toolkit.nodes.alarm_reporter import AlarmReporter
from sensor_msgs.msg import LaserScan


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

        self.declare_parameter('scan_topic', 'scan')
        self.checker = MotionChecker(self.thresholds_from_params(MotionThresholds))
        self.slip = SlipChecker(self.thresholds_from_params(SlipThresholds))
        self.reporter = AlarmReporter(self, 'motion', self.get_parameter('alarm_codes_file').value)

        cmd_topic = self.get_parameter('cmd_topic').value
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(
            Twist, self.get_parameter('wheel_cmd_topic').value, self.on_wheel_cmd, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self.on_scan, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initialpose, 10)
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
        t = self.now_sec()
        tw = msg.twist.twist
        self.checker.update_odom(t, tw.linear.x, tw.angular.z)
        p = msg.pose.pose
        q = p.orientation
        self.slip.update_odom(t, p.position.x, p.position.y,
                              yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_scan(self, msg):
        self.slip.update_scan(self.now_sec(), msg.ranges, msg.range_min, msg.range_max)

    def on_initialpose(self, msg):
        self.slip.reset()
        self.get_logger().info('위치 재설정 수신: E105 기록 초기화')

    def evaluate(self):
        t = self.now_sec()
        if t > 0.0:  # /clock을 받기 전에는 0
            self.reporter.update(t, self.checker.evaluate(t) + self.slip.evaluate(t))


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
