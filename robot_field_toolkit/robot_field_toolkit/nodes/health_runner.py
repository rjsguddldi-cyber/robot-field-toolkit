"""
PHM 정기 건강 점검 러너 (3단계). 지표 계산은 core/health_indicators.py, 판정은 core/phm.py.

같은 MoMa 사이클(A -> 팔 4자세 -> B -> 팔 4자세 ...)을 반복하며 사이클마다:
    데이터 수집 -> 건강지표 -> 하중 상태별 기준선과 비교 -> 건강도·예지 경보 -> CSV 한 줄

- 하중 상태: /field_toolkit/load_state (String, 마지막 값 유지). 없으면 load_state 파라미터
- 기준선: ~/field_toolkit_reports/phm_baseline.json (하중 상태별, 재시작해도 유지)
- 기준선 재설정: ros2 service call /health_runner/reset_baseline std_srvs/srv/Trigger
- 판정 결과: /field_toolkit/phm_status (String, JSON) — 4단계 자동 대응이 받는다
- 채점용 ground truth: /ground_truth/odom (Gazebo 실제 위치). CSV 채점 열에만 쓰고 판정에는 안 씀
"""

import csv
from datetime import datetime
import json
import math
import os

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from robot_field_toolkit.core.commissioning import ARM_SEQUENCE, check_points
from robot_field_toolkit.core.common import yaw_from_quaternion
from robot_field_toolkit.core.health_indicators import (
    compute_indicators, CycleData, IndicatorSettings)
from robot_field_toolkit.core.nav_check import pose_error
from robot_field_toolkit.core.phm import PhmMonitor, PhmSettings
from robot_field_toolkit.core.phm_response import response_for
from robot_field_toolkit.core.scan_check import valid_ratio
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

INFO_COLS = ['cycle', 'sim_time_s', 'point', 'load_state', 'success', 'fail_reason',
             'cycle_time_s', 'alarms', 'nav_recoveries']
PHM_COLS = ['health', 'phm_status', 'learned', 'worst_indicator', 'alarm_indicators',
            'response', 'note']   # response: 이 사이클 판정 뒤 적용한 자동 대응
# 채점 전용: 실제 도착 오차, 그리고 도착 순간 로봇이 믿는 위치(TF)와 실제 위치의 거리
GT_COLS = ['gt_arrive_dist_cm', 'gt_arrive_yaw_deg', 'gt_loc_err_cm',
           'condition']   # 시험 조건 표시 (예: wheel_wear=0.03). 채점·분석용, 판정에는 안 씀


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class HealthRunner(Node):
    """MoMa 사이클을 반복하며 사이클마다 건강도를 판정하고 CSV를 쓴다."""

    def __init__(self):
        super().__init__('health_runner')
        for name, default in [
                ('points_file', ''), ('cycles', 20), ('settle_time', 2.0),
                ('nav_timeout', 120.0), ('arm_step_timeout', 15.0),
                ('report_dir', '~/field_toolkit_reports'), ('baseline_file', 'phm_baseline.json'),
                ('load_state', 'empty'), ('csv_name', ''), ('condition', ''),
                ('k_baseline', 8), ('ma_window', 5),
                ('n_consecutive', 3), ('caution_d', 2.0), ('warning_d', 3.0),
                ('score_slope', 20.0), ('clip_d', 6.0), ('auto_response', True)]:
            self.declare_parameter(name, default)

        def p(name):
            return self.get_parameter(name).value

        points_file = p('points_file') or os.path.join(
            get_package_share_directory('robot_field_toolkit'), 'config', 'work_points.yaml')
        with open(points_file, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.points = cfg['points']
        check_points(self.points)
        self.order = cfg['order']

        out_dir = os.path.expanduser(p('report_dir'))
        os.makedirs(out_dir, exist_ok=True)
        self.baseline_path = os.path.join(out_dir, p('baseline_file'))
        saved = {}
        if os.path.exists(self.baseline_path):
            with open(self.baseline_path, encoding='utf-8') as f:
                saved = json.load(f)
        self.phm = PhmMonitor(PhmSettings(
            k_baseline=p('k_baseline'), ma_window=p('ma_window'),
            n_consecutive=p('n_consecutive'), caution_d=p('caution_d'),
            warning_d=p('warning_d'), score_slope=p('score_slope'), clip_d=p('clip_d')), saved)
        self.ind_settings = IndicatorSettings()
        self.load_state = p('load_state')

        self.condition = p('condition')   # 시험 조건 표시 (CSV 열만, 판정에 안 씀)
        # csv_name을 주면 그 파일에 이어 쓴다 (예: 마모 단계마다 러너를 다시 켜도 한 파일)
        name = p('csv_name') or 'phm_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
        self.csv_path = os.path.join(out_dir, name)
        self.csv_new = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
        self.csv_file = open(self.csv_path, 'a', newline='', encoding='utf-8')
        self.csv = None   # 첫 사이클 지표 키를 보고 머리줄을 쓴다

        # ---- 통신 ----
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.arm_pub = self.create_publisher(String, 'arm/pose_request', 10)
        self.status_pub = self.create_publisher(String, 'field_toolkit/phm_status', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'field_toolkit/load_state', self.on_load, latched)
        self.create_subscription(String, 'field_toolkit/condition', self.on_condition, latched)
        self.create_subscription(String, 'arm/state', self.on_arm_state, 10)
        self.create_subscription(String, 'field_toolkit/alarms', self.on_alarm, 10)
        self.create_subscription(Bool, 'field_toolkit/arm_work_inhibit', self.on_inhibit, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel_checked', self.on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 20)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 10)
        self.create_service(Trigger, '~/reset_baseline', self.on_reset)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 4단계 자동 대응: 경보 수준에 따라 safety_interlock 속도 제한, 팔 정밀 작업 보류
        self.param_cli = self.create_client(SetParameters, '/safety_interlock/set_parameters')
        self.response = response_for('normal')
        self.applied_speed = None
        self.note = ''

        self.arm_state = {}
        self.inhibit = False
        self.alarms = []
        self.active_alarms = set()
        self.odom_pose = None
        self.gt_pose = None
        self.data = None          # 지금 사이클의 CycleData
        self.driving = False      # 주행 구간(①②③④ 수집)인지

        self.idx = 0
        self.state = 'START'
        self.state_t = 0.0
        self.goal_handle = None
        self.nav_result = None
        self.recoveries = 0
        self.done = False
        self.create_timer(0.2, self.step)
        self.get_logger().info(
            f'건강 점검 시작: {p("cycles")}사이클, 지점 {self.order}, 하중 {self.load_state}, '
            f'CSV {self.csv_path}')

    # ---------- 입력 ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_load(self, msg):
        if msg.data and msg.data != self.load_state:
            self.get_logger().info(f'하중 상태 변경: {self.load_state} -> {msg.data} (다음 사이클부터)')
            self.load_state = msg.data

    def on_condition(self, msg):
        self.condition = msg.data

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
            if self.data is not None and ev['code'] not in self.alarms:
                self.alarms.append(ev['code'])
        else:
            self.active_alarms.discard(ev.get('code'))

    def on_inhibit(self, msg):
        self.inhibit = bool(msg.data)

    def stamp(self, topic, header):
        if self.data is not None:
            self.data.topic_stamps.setdefault(topic, []).append(
                (self.now(), stamp_sec(header) if header is not None else None))

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.odom_pose = (pos.x, pos.y)
        self.stamp('odom', msg.header)
        if self.driving:
            tw = msg.twist.twist
            self.data.odom_twist.append((self.now(), tw.linear.x, tw.angular.z))

    def on_imu(self, msg):
        self.stamp('imu', msg.header)
        if self.driving:
            self.data.imu_gyro.append((self.now(), msg.angular_velocity.z))

    def on_cmd(self, msg):
        if self.driving:
            self.data.cmd.append((self.now(), msg.linear.x, msg.angular.z))

    def on_amcl(self, msg):
        if self.driving:
            c = msg.pose.covariance
            self.data.amcl_cov.append((self.now(), c[0], c[7], c[35]))

    def on_scan(self, msg):
        self.stamp('scan', msg.header)
        if self.data is not None:
            self.data.scan_valid.append(
                (self.now(), valid_ratio(msg.ranges, msg.range_min, msg.range_max)))

    def on_joints(self, msg):
        self.stamp('joint_states', msg.header)

    def on_gt(self, msg):
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.gt_pose = (pos.x, pos.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_reset(self, request, response):
        self.phm.reset(self.load_state)
        self.save_baseline()
        response.success = True
        response.message = f'하중 상태 {self.load_state} 기준선 재설정, 다시 학습'
        self.get_logger().warn(response.message)
        return response

    def tf_pose(self):
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
        getattr(self, f'st_{self.state.lower()}')(t)

    def st_start(self, t):
        if not self.nav.wait_for_server(timeout_sec=0.0) or self.tf_pose() is None:
            if t - self.state_t > 5.0:
                self.get_logger().warn('Nav2와 TF(map->base_footprint)를 기다리는 중...')
                self.state_t = t
            return
        self.start_cycle(t)

    def start_cycle(self, t):
        if self.idx >= self.get_parameter('cycles').value:
            self.finish_all()
            return
        self.point = self.order[self.idx % len(self.order)]
        self.cycle_load = self.load_state   # 사이클 도중 바뀌어도 시작 때 값으로 기록
        self.cycle_condition = self.condition
        self.cycle_start_t = t
        self.alarms = []
        self.recoveries = 0
        self.gt_arrival = None
        pt = self.points[self.point]
        self.data = CycleData(goal=(float(pt['x']), float(pt['y']), float(pt['yaw'])))
        self.data.odom_start = self.odom_pose
        self.data.tf_start = self.tf_pose()[:2]
        self.driving = True
        self.get_logger().info(
            f'--- 사이클 {self.idx + 1}/{self.get_parameter("cycles").value}: '
            f'지점 {self.point}, 하중 {self.cycle_load}')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.data.goal[0]
        goal.pose.pose.position.y = self.data.goal[1]
        goal.pose.pose.orientation.z = math.sin(self.data.goal[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.data.goal[2] / 2.0)
        self.nav_result = None
        self.goal_handle = None
        self.nav.send_goal_async(goal, feedback_callback=self.on_feedback).add_done_callback(
            self.on_goal_response)
        self.go('NAVIGATE_WAIT')

    def on_feedback(self, fb):
        self.recoveries = max(self.recoveries, fb.feedback.number_of_recoveries)

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
            self.end_drive()
            self.finish_cycle(t, self.nav_result.split(': ', 1)[1])
        elif t - self.state_t > self.get_parameter('nav_timeout').value:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.end_drive()
            self.finish_cycle(t, '이동 시간 초과')

    def end_drive(self):
        """주행 구간 끝: 끝 위치 기록."""
        self.driving = False
        self.data.odom_end = self.odom_pose
        pose = self.tf_pose()
        self.data.tf_end = pose[:2] if pose else None
        return pose

    def st_settle(self, t):
        if t - self.state_t >= self.get_parameter('settle_time').value:
            self.st_check_arrival(t)

    def st_check_arrival(self, t):
        self.data.arrival_pose = self.end_drive()
        if self.gt_pose is not None:   # 채점용. 판정에는 넣지 않는다
            d, a = pose_error(self.data.goal, self.gt_pose)
            pose = self.data.arrival_pose
            loc = math.dist(pose[:2], self.gt_pose[:2]) * 100.0 if pose else None
            self.gt_arrival = (d * 100.0, math.degrees(a), loc)
        if 'E105' in self.active_alarms:
            self.finish_cycle(t, 'E105 바퀴 헛돎: 팔 작업 생략')
            return
        if self.inhibit:
            self.finish_cycle(t, 'E401 도착 오차 초과로 팔 작업 생략')
            return
        if self.response.hold_arm:   # PHM 경고: 사고 전에 정밀 작업을 멈춘다 (실패가 아니라 대응)
            self.note = 'PHM 경고: 팔 정밀 작업 보류'
            self.finish_cycle(t, '')
            return
        self.arm_step = 0
        self.request_arm()

    def request_arm(self):
        self.arm_pub.publish(String(data=ARM_SEQUENCE[self.arm_step]))
        self.go('ARM_WAIT')

    def st_arm_wait(self, t):
        pose = ARM_SEQUENCE[self.arm_step]
        s = self.arm_state
        if s.get('pose') == pose and s.get('status') == 'reached' and t - self.state_t >= 0.5:
            self.data.arm_times[pose] = t - self.state_t
            self.arm_step += 1
            if self.arm_step >= len(ARM_SEQUENCE):
                self.finish_cycle(t, '')
            else:
                self.request_arm()
        elif t - self.state_t > self.get_parameter('arm_step_timeout').value:
            self.arm_pub.publish(String(data='stowed'))
            self.finish_cycle(t, f'팔 {pose} 도달 실패')

    # ---------- 기록 ----------
    def finish_cycle(self, t, fail_reason):
        success = not fail_reason
        ind = compute_indicators(self.data, self.ind_settings)
        v = self.phm.update(self.cycle_load, ind, success)
        self.save_baseline()   # 기준선 + 이동평균 이력 (러너를 다시 켜도 이어지게)
        self.apply_response(v.status)

        info = {
            'cycle': self.idx + 1, 'sim_time_s': round(self.cycle_start_t, 1),
            'point': self.point, 'load_state': self.cycle_load, 'success': int(success),
            'fail_reason': fail_reason, 'cycle_time_s': round(t - self.cycle_start_t, 2),
            'alarms': ' '.join(self.alarms), 'nav_recoveries': self.recoveries}
        phm = {
            'health': None if v.health is None else round(v.health, 1),
            'phm_status': v.status, 'learned': f'{v.learned}/{v.k}',
            'worst_indicator': v.worst or '', 'alarm_indicators': ' '.join(v.alarm_keys),
            'response': self.response.label(), 'note': self.note}
        self.note = ''
        gt = dict(zip(GT_COLS, (*(self.gt_arrival or (None, None, None)), self.cycle_condition)))
        row = {**info, **{k: (round(x, 4) if isinstance(x, float) else x)
                          for k, x in ind.items()}, **phm, **gt}
        if self.csv is None:
            fields = INFO_COLS + list(ind) + PHM_COLS + GT_COLS
            if not self.csv_new:
                self.upgrade_csv_header(fields)
            self.csv = csv.DictWriter(self.csv_file, fields)
            if self.csv_new:
                self.csv.writeheader()
        self.csv.writerow(row)
        self.csv_file.flush()

        self.status_pub.publish(String(data=json.dumps({
            'cycle': self.idx + 1, 'load_state': v.load_state, 'status': v.status,
            'label': v.label(), 'health': phm['health'], 'worst': v.worst,
            'alarm_indicators': list(v.alarm_keys), 'd_ma': v.d_ma or {},
            'response': {'speed': self.response.speed, 'hold_arm': self.response.hold_arm,
                         'advice': self.response.advice}},
            ensure_ascii=False)))
        health_txt = '-' if v.health is None else f'{v.health:.0f}'
        alarm_txt = f' 경보: {",".join(v.alarm_keys)}' if v.alarm_keys else ''
        text = (f'사이클 {self.idx + 1} {"성공" if success else "실패 " + fail_reason} '
                f'{t - self.cycle_start_t:.1f}s | 건강도 {health_txt} [{v.label()}] '
                f'가장 나쁜 지표 {v.worst or "-"}{alarm_txt}')
        # rclpy 로거는 호출 위치마다 심각도를 기억해서, 한 줄에서 info와 warn을 번갈아 부르면
        # ValueError로 죽는다 (11/23 첫 주의 경보 때 발생) → 호출 위치를 나눈다
        if v.status in ('caution', 'warning'):
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

        self.data = None
        self.idx += 1
        self.start_cycle(t)

    def apply_response(self, status):
        """경보 수준 -> 자동 대응. 속도가 바뀔 때만 safety_interlock에 보낸다."""
        if not self.get_parameter('auto_response').value:
            return
        new = response_for(status)
        if new.hold_arm and not self.response.hold_arm:
            self.get_logger().warn(f'[자동 대응] {new.advice}')
        self.response = new
        if new.speed != self.applied_speed and self.param_cli.service_is_ready():
            req = SetParameters.Request()
            req.parameters = [Parameter(name='speed_override', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=new.speed))]
            self.param_cli.call_async(req)
            self.get_logger().info(f'[자동 대응] 속도 오버라이드 {new.speed:.0f}%')
            self.applied_speed = new.speed

    def upgrade_csv_header(self, fields):
        """이어 쓸 CSV의 열이 지금과 다르면(열 추가 등) 새 열 순서로 다시 쓴다. 없는 값은 빈칸."""
        self.csv_file.close()
        with open(self.csv_path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            old_fields, rows = reader.fieldnames, list(reader)
        if old_fields != fields:
            with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fields + [c for c in old_fields if c not in fields])
                w.writeheader()
                w.writerows(rows)
            self.get_logger().info(f'CSV 열 갱신: {len(rows)}줄 다시 씀')
        self.csv_file = open(self.csv_path, 'a', newline='', encoding='utf-8')

    def save_baseline(self):
        with open(self.baseline_path, 'w', encoding='utf-8') as f:
            json.dump(self.phm.to_dict(), f, ensure_ascii=False, indent=1)

    def finish_all(self):
        self.csv_file.close()
        self.get_logger().info(f'건강 점검 끝. CSV: {self.csv_path}')
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HealthRunner()
    except ValueError as e:  # 작업 지점 좌표가 비어 있는 등 설정 오류
        rclpy.logging.get_logger('health_runner').error(str(e))
        rclpy.shutdown()
        return
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.csv_file.closed:
            node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
