"""
주행·팔 상호 인터락 노드 (E301~E303) + 속도 오버라이드. 판단은 core/interlock.py.

명령 경로:  명령원 -> /cmd_vel -> [safety_interlock] -> /cmd_vel_checked -> fault_injector
팔 경로:    요청원 -> /arm/pose_request -> [safety_interlock] -> /arm/pose_command -> arm_controller

속도 오버라이드(두산 티치펜던트 이동속도 %): ros2 param set /safety_interlock speed_override 30
"""

from dataclasses import fields
import json

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.interlock import InterlockThresholds, SafetyInterlock
from robot_field_toolkit.nodes.alarm_reporter import AlarmReporter
from std_msgs.msg import Bool, String


class SafetyInterlockNode(Node):
    """주행 명령과 팔 요청을 서로 막고, 통과하는 주행 명령에 속도 %를 곱한다."""

    def __init__(self):
        super().__init__('safety_interlock')
        self.declare_parameter('cmd_in', 'cmd_vel')
        self.declare_parameter('cmd_out', 'cmd_vel_checked')
        self.declare_parameter('speed_override', 100.0)
        self.declare_parameter('alarm_codes_file', '')
        # 이 알람이 켜져 있는 동안 팔 작업(stowed 외) 금지
        self.declare_parameter('inhibit_codes', ['E105'])
        self.nav_inhibit = False
        self.active_alarms = set()
        defaults = InterlockThresholds()
        values = {}
        for f in fields(InterlockThresholds):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(f.name).value
        self.il = SafetyInterlock(InterlockThresholds(**values))
        self.il.set_speed_override(self.get_parameter('speed_override').value)
        self.add_on_set_parameters_callback(self.on_set_params)

        self.reporter = AlarmReporter(
            self, 'interlock', self.get_parameter('alarm_codes_file').value)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_out').value, 10)
        self.arm_cmd_pub = self.create_publisher(String, 'arm/pose_command', 10)

        self.create_subscription(Twist, self.get_parameter('cmd_in').value, self.on_cmd, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(String, 'arm/state', self.on_arm_state, 10)
        self.create_subscription(String, 'arm/pose_request', self.on_arm_request, 10)
        self.create_subscription(Bool, 'field_toolkit/arm_work_inhibit', self.on_inhibit, 10)
        self.create_subscription(String, 'field_toolkit/alarms', self.on_alarm, 10)
        self.create_timer(0.1, self.evaluate)
        self.get_logger().info(f'safety_interlock 시작, 속도 {self.il.speed_override:.0f}%')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_set_params(self, params):
        for p in params:
            if p.name == 'speed_override':
                if not 0.0 <= float(p.value) <= 100.0:
                    return SetParametersResult(successful=False, reason='0~100 사이여야 함')
                self.il.set_speed_override(p.value)
                self.get_logger().info(f'속도 오버라이드 {float(p.value):.0f}%')
        return SetParametersResult(successful=True)

    def on_cmd(self, msg):
        v, w, blocked = self.il.filter_drive(self.now_sec(), msg.linear.x, msg.angular.z)
        out = Twist()
        out.linear.x = v
        out.angular.z = w
        self.cmd_pub.publish(out)

    def on_odom(self, msg):
        tw = msg.twist.twist
        self.il.update_odom(self.now_sec(), tw.linear.x, tw.angular.z)

    def on_arm_state(self, msg):
        try:
            s = json.loads(msg.data)
            self.il.update_arm_state(self.now_sec(), s['pose'], s['status'], s.get('elapsed', 0.0))
        except (ValueError, KeyError):
            self.get_logger().warn(f'arm/state 형식 오류: {msg.data}')

    def on_arm_request(self, msg):
        pose = msg.data.strip()
        ok, reason = self.il.request_arm(self.now_sec(), pose)
        if ok:
            self.arm_cmd_pub.publish(String(data=pose))
        else:
            self.get_logger().warn(f'팔 요청 거부: {pose} ({reason})')

    def on_inhibit(self, msg):
        self.nav_inhibit = bool(msg.data)
        self.update_inhibit()

    def on_alarm(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        if ev.get('state') == 'raised':
            self.active_alarms.add(ev.get('code'))
        else:
            self.active_alarms.discard(ev.get('code'))
        self.update_inhibit()

    def update_inhibit(self):
        codes = sorted(self.active_alarms & set(self.get_parameter('inhibit_codes').value))
        if self.nav_inhibit:
            codes.insert(0, 'E401')
        self.il.set_work_inhibit(bool(codes), ', '.join(codes))

    def evaluate(self):
        t = self.now_sec()
        if t > 0.0:
            self.reporter.update(t, self.il.evaluate(t))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyInterlockNode()
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
