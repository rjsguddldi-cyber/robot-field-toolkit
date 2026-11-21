"""
구동 동작 판정 (E101~E104). rclpy 없이 순수 파이썬.

시간(t, 초)은 모두 인자로 받는다. ROS 노드는 시뮬레이션 시간을 넣고,
테스트는 가짜 시간을 넣는다. 그래서 컨트롤러가 ROS가 아니어도 재사용할 수 있다.

    checker = MotionChecker()
    checker.update_command(t, v, w)   # 상위 명령 (/cmd_vel)
    checker.update_wheel_cmd(t)       # 구동기 입력 수신 (/wheel_cmd)
    checker.update_odom(t, v, w)      # 실제 속도 (/odom twist)
    results = checker.evaluate(t)     # [Judgement(code, active, detail), ...]
"""

from collections import deque
from dataclasses import dataclass

CODES = ('E101', 'E102', 'E103', 'E104')


@dataclass
class MotionThresholds:
    """판정 기준값. 숫자의 의미는 config/motion_monitor.yaml 주석 참고."""

    cmd_v_min: float = 0.05        # 이보다 큰 |v| 명령이면 '움직이라는 명령' [m/s]
    cmd_w_min: float = 0.1         # 이보다 큰 |w| 명령이면 '움직이라는 명령' [rad/s]
    stall_ratio: float = 0.2       # E101: 실제/명령 < 20%
    stall_time: float = 1.0        # E101: 지속 시간 [s]
    weak_ratio: float = 0.7        # E102: 20% <= 실제/명령 < 70%
    weak_time: float = 2.0         # E102: 지속 시간 [s]
    loss_window: float = 2.0       # E103: 수신 수를 세는 구간 [s]
    loss_ratio: float = 0.7        # E103: wheel_cmd 수 < cmd 수의 70%
    loss_min_cmds: int = 5         # E103: 구간 안 명령이 이보다 적으면 판정 안 함
    straight_w_max: float = 0.05   # E104: |w| 명령이 이보다 작으면 '직진 명령' [rad/s]
    drift_w: float = 0.15          # E104: 직진인데 실제 |w|가 이보다 큼 [rad/s]
    drift_time: float = 1.5        # E104: 지속 시간 [s]
    clear_time: float = 1.0        # 해제: 조건이 이만큼 계속 사라져야 해제 [s]
    odom_timeout: float = 0.5      # odom이 이보다 오래 안 오면 판정 보류 [s]


@dataclass
class Judgement:
    """코드 하나의 판정 결과."""

    code: str
    active: bool
    detail: str = ''


class _Debounce:
    """조건이 on_time 동안 계속 참이면 켜고, off_time 동안 계속 거짓이면 끈다."""

    def __init__(self, on_time, off_time):
        self.on_time = on_time
        self.off_time = off_time
        self.active = False
        self._since = None  # 현재 상태와 반대인 조건이 시작된 시각

    def update(self, t, condition):
        if condition == self.active:
            self._since = None
            return self.active
        if self._since is None:
            self._since = t
        hold = self.off_time if self.active else self.on_time
        if t - self._since >= hold:
            self.active = condition
            self._since = None
        return self.active

    def reset(self):
        self.active = False
        self._since = None


def _axis_ratio(cmd, actual):
    """명령 방향으로 실제로 낸 비율. 반대로 가면 0으로 본다."""
    return max(0.0, actual * (1.0 if cmd > 0 else -1.0) / abs(cmd))


class MotionChecker:
    """상위 명령, 구동기 입력, 실제 속도를 비교해서 E101~E104를 판정한다."""

    def __init__(self, thresholds=None):
        self.th = thresholds or MotionThresholds()
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.odom_t = None
        self._cmd_times = deque()
        self._wheel_times = deque()
        clear = self.th.clear_time
        self._deb = {
            'E101': _Debounce(self.th.stall_time, clear),
            'E102': _Debounce(self.th.weak_time, clear),
            'E103': _Debounce(0.0, clear),  # 구간(window) 자체가 지속 시간 역할
            'E104': _Debounce(self.th.drift_time, clear),
        }

    # ---------- 입력 ----------
    def update_command(self, t, v, w):
        """상위 명령 수신. 마지막 명령을 유지한다 (Gazebo DiffDrive도 마지막 명령 유지)."""
        self.cmd_v = v
        self.cmd_w = w
        self._cmd_times.append(t)

    def update_wheel_cmd(self, t):
        """구동기 입력 수신 시각만 센다 (E103용)."""
        self._wheel_times.append(t)

    def update_odom(self, t, v, w):
        """실제 속도 수신."""
        self.odom_v = v
        self.odom_w = w
        self.odom_t = t

    # ---------- 판정 ----------
    def command_active(self):
        """'움직이라는 명령'인가."""
        return abs(self.cmd_v) > self.th.cmd_v_min or abs(self.cmd_w) > self.th.cmd_w_min

    def speed_ratio(self):
        """
        실제/명령 비율.

        움직이라는 축(v, w)마다 비율을 구해서 큰 쪽을 쓴다. 한 축이라도 제대로
        움직이면 '구동은 되는 중'으로 보아 오알람을 줄이려는 선택이다.
        움직이라는 명령이 아니면 None.
        """
        ratios = []
        if abs(self.cmd_v) > self.th.cmd_v_min:
            ratios.append(_axis_ratio(self.cmd_v, self.odom_v))
        if abs(self.cmd_w) > self.th.cmd_w_min:
            ratios.append(_axis_ratio(self.cmd_w, self.odom_w))
        return max(ratios) if ratios else None

    def _count_in_window(self, times, t):
        while times and times[0] < t - self.th.loss_window:
            times.popleft()
        return len(times)

    def evaluate(self, t):
        """시각 t의 판정 결과 목록 (E101~E104 순서)."""
        th = self.th
        details = dict.fromkeys(CODES, '')

        # E103은 odom과 상관없이 수신 수만 비교한다.
        n_cmd = self._count_in_window(self._cmd_times, t)
        n_wheel = self._count_in_window(self._wheel_times, t)
        loss = n_cmd >= th.loss_min_cmds and n_wheel < th.loss_ratio * n_cmd
        details['E103'] = f'cmd {n_cmd}개, wheel_cmd {n_wheel}개 (최근 {th.loss_window:.1f}s)'

        odom_ok = self.odom_t is not None and t - self.odom_t <= th.odom_timeout
        if not odom_ok:
            # 실제 속도를 모르면 E101/E102/E104는 판단하지 않는다 (오알람 방지).
            for code in ('E101', 'E102', 'E104'):
                self._deb[code].reset()
                details[code] = 'odom 없음: 판정 보류'
            stall = weak = drift = False
        else:
            ratio = self.speed_ratio()
            stall = ratio is not None and ratio < th.stall_ratio
            weak = ratio is not None and th.stall_ratio <= ratio < th.weak_ratio
            straight = abs(self.cmd_w) < th.straight_w_max and abs(self.cmd_v) > th.cmd_v_min
            drift = straight and abs(self.odom_w) > th.drift_w
            ratio_txt = '정지 명령' if ratio is None else f'실제/명령 {ratio * 100:.0f}%'
            details['E101'] = details['E102'] = ratio_txt
            details['E104'] = f'명령 w {self.cmd_w:+.2f}, 실제 w {self.odom_w:+.2f} rad/s'

        conditions = {'E101': stall, 'E102': weak, 'E103': loss, 'E104': drift}
        return [
            Judgement(code, self._deb[code].update(t, conditions[code]), details[code])
            for code in CODES
        ]
