"""
동작 감시 노드 (E101~E104). 판정은 core/motion_check.py가 하고 이 노드는 껍데기.

구독: /cmd_vel(상위 명령), /wheel_cmd(구동기 입력), /odom(실제 속도)
발행:
    /diagnostics            DiagnosticArray, 1Hz (WARN->WARN, ALARM/EMERGENCY->ERROR)
    /field_toolkit/alarms   String(JSON), 알람이 뜨거나 풀릴 때만
터미널: 알람이 뜨면 코드, 의미, 점검 순서 출력
"""

from dataclasses import fields
import json
import os

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.alarm_codes import load_alarm_codes
from robot_field_toolkit.core.motion_check import MotionChecker, MotionThresholds
from std_msgs.msg import String

DIAG_LEVEL = {
    'WARN': DiagnosticStatus.WARN,
    'ALARM': DiagnosticStatus.ERROR,
    'EMERGENCY': DiagnosticStatus.ERROR,
}


class MotionMonitor(Node):
    """명령과 실제 움직임을 비교해서 구동 알람을 낸다."""

    def __init__(self):
        super().__init__('motion_monitor')

        # 토픽 이름 (3층 인터락 이후 cmd_topic을 cmd_vel_checked로 바꿀 예정)
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('wheel_cmd_topic', 'wheel_cmd')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('alarm_codes_file', '')
        self.declare_parameter('eval_rate', 10.0)

        # 판정 기준값: MotionThresholds의 칸 이름을 그대로 파라미터 이름으로 쓴다.
        defaults = MotionThresholds()
        values = {}
        for f in fields(MotionThresholds):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(f.name).value
        self.checker = MotionChecker(MotionThresholds(**values))

        codes_file = self.get_parameter('alarm_codes_file').value or os.path.join(
            get_package_share_directory('robot_field_toolkit'), 'config', 'alarm_codes.yaml')
        self.alarm_codes = load_alarm_codes(codes_file)

        self.active = {}   # 코드 -> 지금 켜져 있는가
        self.latest = []   # 마지막 판정 결과

        self.create_subscription(
            Twist, self.get_parameter('cmd_topic').value, self.on_cmd, 10)
        self.create_subscription(
            Twist, self.get_parameter('wheel_cmd_topic').value, self.on_wheel_cmd, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.alarm_pub = self.create_publisher(String, '/field_toolkit/alarms', 10)

        self.create_timer(1.0 / self.get_parameter('eval_rate').value, self.evaluate)
        self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info(f'motion_monitor 시작, 코드표: {codes_file}')

    def now_sec(self):
        """노드 시계(use_sim_time이면 시뮬레이션 시간) 기준 초."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 입력: core에 넘기기만 한다 ----------
    def on_cmd(self, msg):
        self.checker.update_command(self.now_sec(), msg.linear.x, msg.angular.z)

    def on_wheel_cmd(self, msg):
        self.checker.update_wheel_cmd(self.now_sec())

    def on_odom(self, msg):
        tw = msg.twist.twist
        self.checker.update_odom(self.now_sec(), tw.linear.x, tw.angular.z)

    # ---------- 판정 ----------
    def evaluate(self):
        t = self.now_sec()
        if t == 0.0:
            return  # 아직 /clock을 못 받음
        self.latest = self.checker.evaluate(t)
        for j in self.latest:
            if j.active != self.active.get(j.code, False):
                self.active[j.code] = j.active
                self.on_change(t, j)

    def on_change(self, t, judgement):
        """알람이 새로 뜨거나 풀렸을 때: 이벤트 발행 + 터미널 출력."""
        info = self.alarm_codes[judgement.code]
        state = 'raised' if judgement.active else 'cleared'
        event = {
            'stamp': round(t, 3),
            'code': info.code,
            'name': info.name,
            'level': info.level,
            'state': state,
            'detail': judgement.detail,
        }
        self.alarm_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))

        log = self.get_logger()
        if judgement.active:
            text = f'\n{info.format_text()}\n  현재값: {judgement.detail}'
            (log.warn if info.level == 'WARN' else log.error)(text)
        else:
            log.info(f'[해제] {info.code} {info.name}')

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for j in self.latest:
            info = self.alarm_codes[j.code]
            st = DiagnosticStatus()
            st.name = f'field_toolkit/motion: {j.code} {info.name}'
            st.hardware_id = 'my_robot'
            if j.active:
                st.level = DIAG_LEVEL[info.level]
                st.message = f'{info.level}: {info.meaning}'
            else:
                st.level = DiagnosticStatus.OK
                st.message = 'OK'
            st.values = [
                KeyValue(key='detail', value=j.detail),
                KeyValue(key='first_check', value=info.checks[0]),
            ]
            arr.status.append(st)
        self.diag_pub.publish(arr)


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
