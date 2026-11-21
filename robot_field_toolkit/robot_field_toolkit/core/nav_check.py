"""
주행 감시 판정 (E401 도착 오차, E402 위치 상실, E403 경로 차단). rclpy 없이 순수 파이썬.

입력(노드가 넣어 줌):
    set_goal          목표 좌표 (/plan 마지막 점)
    update_status     NavigateToPose 목표 상태 (action status: 2 실행 중, 4 성공, 6 실패 ...)
    update_feedback   남은 거리, 복구 횟수 (action feedback)
    update_pose       현재 위치 (TF map->base_footprint)
    update_covariance AMCL 공분산 (/amcl_pose)
"""

from dataclasses import dataclass
import math

from robot_field_toolkit.core.common import Debounce, Judgement, normalize_angle

CODES = ('E401', 'E402', 'E403')

STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


def pose_error(goal, actual):
    """(거리 오차 [m], 각도 오차 [rad]). goal, actual = (x, y, yaw)."""
    dist = math.hypot(goal[0] - actual[0], goal[1] - actual[1])
    return dist, abs(normalize_angle(goal[2] - actual[2]))


@dataclass
class NavThresholds:
    """주행 감시 기준값. 의미는 config/nav_monitor.yaml 주석 참고."""

    arrive_dist_tol: float = 0.10    # E401: 도착 후 거리 오차 허용 [m]
    arrive_yaw_tol: float = 0.15     # E401: 도착 후 각도 오차 허용 [rad] (약 8.6도)
    settle_time: float = 1.0         # E401: 도착(성공) 후 이만큼 기다렸다가 잰다 [s]
    cov_xy_max: float = 0.35         # E402: 위치 표준편차 허용 [m]
    cov_yaw_max: float = 0.45        # E402: 방향 표준편차 허용 [rad] (약 26도)
    lost_time: float = 2.0           # E402: 지속 시간 [s]
    recovery_max: int = 3            # E403: 한 목표에서 복구 동작이 이 횟수 이상이면
    progress_window: float = 15.0    # E403: 이 시간 동안
    progress_min: float = 0.2        # E403: 남은 거리가 이만큼도 안 줄면 [m]
    clear_time: float = 1.0          # E402 해제 지연 [s]


class NavChecker:
    """목표 도착 정확도, 위치 추정 품질, 경로 진척을 판정한다."""

    def __init__(self, thresholds=None):
        self.th = thresholds or NavThresholds()
        self.goal = None           # (x, y, yaw)
        self.goal_id = None
        self.status = None
        self.status_t = None
        self.amcl = None           # 현재 위치 (x, y, yaw), TF 기준
        self.amcl_std = None       # (xy_std, yaw_std)
        self.recoveries = 0
        self.dist_remaining = None
        self._progress_ref = None  # (t, 남은 거리)
        self.arrival_error = None  # (dist, yaw) 도착 후 잰 값
        self.e401 = False
        self.e403 = False
        self.e403_reason = ''
        self._e402 = Debounce(self.th.lost_time, self.th.clear_time)

    # ---------- 입력 ----------
    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)

    def update_status(self, t, goal_id, status):
        """새 목표가 시작되면 E401/E403 상태를 초기화한다."""
        if goal_id != self.goal_id:
            self.goal_id = goal_id
            self.recoveries = 0
            self.dist_remaining = None
            self._progress_ref = None
            self.arrival_error = None
            self.e401 = False
            self.e403 = False
            self.e403_reason = ''
        if status != self.status:
            self.status = status
            self.status_t = t

    def update_feedback(self, t, distance_remaining, recoveries):
        self.dist_remaining = distance_remaining
        self.recoveries = recoveries
        if self._progress_ref is None or \
                self._progress_ref[1] - distance_remaining >= self.th.progress_min:
            self._progress_ref = (t, distance_remaining)

    def update_pose(self, x, y, yaw):
        """
        현재 위치 (map 기준). 노드는 TF map->base_footprint를 넣는다.

        /amcl_pose는 로봇이 일정 거리(0.25m)·각도를 움직였을 때만 나와서
        멈춘 순간의 위치보다 늦을 수 있으므로 도착 오차에는 쓰지 않는다.
        """
        self.amcl = (x, y, yaw)

    def update_covariance(self, cov_xx, cov_yy, cov_yawyaw):
        """AMCL 공분산 (/amcl_pose). E402용."""
        self.amcl_std = (math.sqrt(max(cov_xx, cov_yy, 0.0)), math.sqrt(max(cov_yawyaw, 0.0)))

    # ---------- 판정 ----------
    def work_inhibit(self):
        """팔 작업 금지 신호 (E401이 켜져 있으면 True)."""
        return self.e401

    def evaluate(self, t):
        th = self.th
        # E401: 성공 후 settle_time이 지나면 한 번 재고, 다음 목표까지 유지
        if (self.status == STATUS_SUCCEEDED and self.arrival_error is None
                and self.goal is not None and self.amcl is not None
                and t - self.status_t >= th.settle_time):
            self.arrival_error = pose_error(self.goal, self.amcl)
            d, a = self.arrival_error
            self.e401 = d > th.arrive_dist_tol or a > th.arrive_yaw_tol
        if self.arrival_error is None:
            e401_txt = '도착 전'
        else:
            d, a = self.arrival_error
            e401_txt = (f'거리 {d * 100:.1f}cm, 각도 {math.degrees(a):.1f}도'
                        f' (기준 {th.arrive_dist_tol * 100:.0f}cm, '
                        f'{math.degrees(th.arrive_yaw_tol):.1f}도)')

        # E402: AMCL 불확실성
        lost = self.amcl_std is not None and (
            self.amcl_std[0] > th.cov_xy_max or self.amcl_std[1] > th.cov_yaw_max)
        e402_txt = ('AMCL 없음' if self.amcl_std is None else
                    f'위치 표준편차 {self.amcl_std[0]:.2f}m, 방향 {self.amcl_std[1]:.2f}rad')

        # E403: 실행 중 복구 반복 또는 진척 없음. 다음 목표까지 유지(실패해도 남김)
        if self.status == STATUS_EXECUTING and not self.e403:
            if self.recoveries >= th.recovery_max:
                self.e403, self.e403_reason = True, f'복구 {self.recoveries}회'
            elif self._progress_ref is not None and \
                    t - self._progress_ref[0] >= th.progress_window:
                self.e403 = True
                self.e403_reason = (f'{th.progress_window:.0f}s 동안 남은 거리'
                                    f' {th.progress_min:.1f}m도 안 줄어듦')
        if self.status == STATUS_SUCCEEDED:
            self.e403 = False
        e403_txt = self.e403_reason or (
            f'복구 {self.recoveries}회, 남은 거리 '
            + ('-' if self.dist_remaining is None else f'{self.dist_remaining:.2f}m'))

        return [
            Judgement('E401', self.e401, e401_txt),
            Judgement('E402', self._e402.update(t, lost), e402_txt),
            Judgement('E403', self.e403, e403_txt),
        ]
