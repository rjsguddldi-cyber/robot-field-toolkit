"""
MoMa 사이클 시운전 러너. 계획·집계·리포트는 core/commissioning.py.

상태 흐름 (docs/commissioning_flow.md에 mermaid 그림):
    SET_SPEED -> NAVIGATE -> SETTLE -> CHECK_ARRIVAL -> ARM(approach, work, retreat, stowed)
    -> RECORD -> 다음 사이클 ... -> REPORT

- 속도: /safety_interlock speed_override 파라미터를 바꾼다 (30 -> 60 -> 100%)
- 이동: Nav2 NavigateToPose 액션
- 도착 확인: TF(map->base_footprint) 위치로 오차를 재고, nav_monitor의 팔 작업 금지 신호(E401)를 본다
- 팔: /arm/pose_request (safety_interlock을 거쳐 arm_controller로)
- 알람: /field_toolkit/alarms에서 사이클 중 raised된 코드를 기록
- 리포트: ~/field_toolkit_reports/commissioning_<시각>.md, .json
"""

from datetime import datetime
import json
import math
import os

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_field_toolkit.core.commissioning import (
    ARM_SEQUENCE, build_plan, check_points, CycleRecord, summarize, to_json, to_markdown)
from robot_field_toolkit.core.common import yaw_from_quaternion
from robot_field_toolkit.core.nav_check import pose_error
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class CommissioningRunner(Node):
    """시운전 사이클을 차례로 실행하고 리포트를 쓴다."""

    def __init__(self):
        super().__init__('commissioning_runner')
        self.declare_parameter('points_file', '')
        self.declare_parameter('speeds', [30.0, 60.0, 100.0])
        self.declare_parameter('cycles_per_speed', 3)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('arm_step_timeout', 15.0)
        self.declare_parameter('report_dir', '~/field_toolkit_reports')
        self.declare_parameter('note', '')
        p = self.get_parameter

        points_file = p('points_file').value or os.path.join(
            get_package_share_directory('robot_field_toolkit'), 'config', 'work_points.yaml')
        with open(points_file, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.points = cfg['points']
        check_points(self.points)  # 좌표가 비어 있으면 여기서 멈춘다
        self.plan = build_plan(cfg['order'], p('speeds').value, p('cycles_per_speed').value)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.param_cli = self.create_client(SetParameters, '/safety_interlock/set_parameters')
        self.arm_pub = self.create_publisher(String, 'arm/pose_request', 10)
        self.create_subscription(String, 'arm/state', self.on_arm_state, 10)
        self.create_subscription(String, 'field_toolkit/alarms', self.on_alarm, 10)
        self.create_subscription(Bool, 'field_toolkit/arm_work_inhibit', self.on_inhibit, 10)
        # 도착 위치는 TF로 읽는다 (/amcl_pose는 일정 거리를 움직여야 갱신돼 늦을 수 있음)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arm_state = {}
        self.inhibit = False
        self.alarms = []           # 사이클 중 raised 코드
        self.active_alarms = set()  # 지금 켜져 있는 알람
        self.records = []
        self.faults = set()
        self.started = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        self.idx = -1              # 현재 사이클 번호 (plan 인덱스)
        self.state = 'START'
        self.state_t = 0.0
        self.rec = None
        self.cur_speed = None
        self.goal_handle = None
        self.nav_result = None     # None / 'succeeded' / 'failed: ...'
        self.arm_step = 0
        self.arm_start_t = 0.0
        self.cycle_start_t = 0.0
        self.done = False

        self.create_timer(0.2, self.step)
        self.get_logger().info(f'시운전 시작: {len(self.plan)}사이클, 지점 {cfg["order"]}')

    # ---------- 입력 ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_arm_state(self, msg):
        try:
            self.arm_state = json.loads(msg.data)
        except ValueError:
            pass

    def on_alarm(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        if ev.get('state') == 'raised':
            self.active_alarms.add(ev['code'])
            if self.rec is not None and ev['code'] not in self.alarms:
                self.alarms.append(ev['code'])
        else:
            self.active_alarms.discard(ev.get('code'))

    def on_inhibit(self, msg):
        self.inhibit = bool(msg.data)

    def current_pose(self):
        """TF map->base_footprint. 없으면 None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        tr, q = tf.transform.translation, tf.transform.rotation
        return tr.x, tr.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    # ---------- 상태 머신 ----------
    def go(self, state):
        self.state = state
        self.state_t = self.now()

    def step(self):
        t = self.now()
        if t <= 0.0 or self.done:
            return
        handler = getattr(self, f'st_{self.state.lower()}')
        handler(t)

    def st_start(self, t):
        if not self.nav.wait_for_server(timeout_sec=0.0):
            if t - self.state_t > 5.0:
                self.get_logger().warn('Nav2(navigate_to_pose)를 기다리는 중...')
                self.state_t = t
            return
        self.next_cycle()

    def next_cycle(self):
        self.idx += 1
        if self.idx >= len(self.plan):
            self.go('REPORT')
            return
        pc = self.plan[self.idx]
        self.rec = CycleRecord(pc.speed, pc.cycle, pc.point)
        self.alarms = []
        self.get_logger().info(
            f'--- 사이클 {self.idx + 1}/{len(self.plan)}: 속도 {pc.speed:.0f}%, '
            f'{pc.cycle}회차, 지점 {pc.point}')
        self.go('SET_SPEED' if pc.speed != self.cur_speed else 'NAVIGATE_SEND')

    def st_set_speed(self, t):
        if not self.param_cli.service_is_ready():
            return
        speed = self.rec.speed
        req = SetParameters.Request()
        req.parameters = [Parameter(name='speed_override', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=speed))]
        self.param_cli.call_async(req)
        self.cur_speed = speed
        self.get_logger().info(f'속도 오버라이드 {speed:.0f}%')
        self.go('NAVIGATE_SEND')

    def st_navigate_send(self, t):
        pt = self.points[self.rec.point]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pt['x'])
        goal.pose.pose.position.y = float(pt['y'])
        goal.pose.pose.orientation.z = math.sin(float(pt['yaw']) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(pt['yaw']) / 2.0)
        self.nav_result = None
        self.cycle_start_t = t
        self.nav.send_goal_async(goal).add_done_callback(self.on_goal_response)
        self.go('NAVIGATE_WAIT')

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.nav_result = 'failed: Nav2가 목표 거부'
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        status = future.result().status
        self.nav_result = ('succeeded' if status == GoalStatus.STATUS_SUCCEEDED
                           else f'failed: 이동 실패(status {status})')

    def st_navigate_wait(self, t):
        if self.nav_result == 'succeeded':
            self.go('SETTLE')
        elif self.nav_result is not None:
            self.finish_cycle(t, self.nav_result.split(': ', 1)[1])
        elif t - self.state_t > self.get_parameter('nav_timeout').value:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.finish_cycle(t, '이동 시간 초과')

    def st_settle(self, t):
        if t - self.state_t >= self.get_parameter('settle_time').value:
            self.go('CHECK_ARRIVAL')

    def st_check_arrival(self, t):
        pt = self.points[self.rec.point]
        pose = self.current_pose()
        if pose is not None:
            d, a = pose_error((pt['x'], pt['y'], pt['yaw']), pose)
            self.rec.arrival_dist, self.rec.arrival_yaw = d, a
        if 'E105' in self.active_alarms:
            self.finish_cycle(t, 'E105 바퀴 헛돎: 위치를 믿을 수 없어 팔 작업 생략')
            return
        if self.inhibit:
            self.finish_cycle(t, 'E401 도착 오차 초과로 팔 작업 생략')
            return
        self.arm_step = 0
        self.arm_start_t = t
        self.request_arm(t)

    def request_arm(self, t):
        pose = ARM_SEQUENCE[self.arm_step]
        self.arm_pub.publish(String(data=pose))
        self.go('ARM_WAIT')

    def st_arm_wait(self, t):
        pose = ARM_SEQUENCE[self.arm_step]
        s = self.arm_state
        if s.get('pose') == pose and s.get('status') == 'reached' and t - self.state_t >= 0.5:
            self.arm_step += 1
            if self.arm_step >= len(ARM_SEQUENCE):
                self.rec.arm_time = t - self.arm_start_t
                self.finish_cycle(t, '')
            else:
                self.request_arm(t)
        elif t - self.state_t > self.get_parameter('arm_step_timeout').value:
            self.arm_pub.publish(String(data='stowed'))
            self.finish_cycle(t, f'팔 {pose} 도달 실패')

    def finish_cycle(self, t, fail_reason):
        r = self.rec
        r.success = not fail_reason
        r.fail_reason = fail_reason
        r.cycle_time = t - self.cycle_start_t
        r.alarms = list(self.alarms)
        self.records.append(r)
        self.get_logger().info(
            f'사이클 결과: {"성공" if r.success else "실패"} {fail_reason} '
            f'시간 {r.cycle_time:.1f}s, 알람 {r.alarms or "-"}')
        self.next_cycle()

    def st_report(self, t):
        self.set_speed_back()
        summary = summarize(self.records)
        meta = {
            'started': self.started,
            'points': {k: self.points[k] for k in self.points},
            'faults': self.get_parameter('note').value or '-',
            'note': self.get_parameter('note').value,
        }
        out_dir = os.path.expanduser(self.get_parameter('report_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        stem = os.path.join(out_dir, 'commissioning_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
        with open(stem + '.md', 'w', encoding='utf-8') as f:
            f.write(to_markdown(self.records, summary, meta))
        with open(stem + '.json', 'w', encoding='utf-8') as f:
            f.write(to_json(self.records, summary, meta))
        self.get_logger().info(f'리포트 저장: {stem}.md, {stem}.json')
        self.done = True

    def set_speed_back(self):
        if self.param_cli.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [Parameter(name='speed_override', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=100.0))]
            self.param_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CommissioningRunner()
    except ValueError as e:  # 작업 지점 좌표가 비어 있는 등 설정 오류
        rclpy.logging.get_logger('commissioning_runner').error(str(e))
        rclpy.shutdown()
        return
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
