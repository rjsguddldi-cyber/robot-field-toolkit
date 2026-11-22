"""
PHM 3단계: 하중 조건별 기준선, 건강도 0~100, 예지 경보. rclpy 없이 순수 파이썬, 통계만.

흐름 (사이클 1회마다 update):
    지표 dict (core/health_indicators.compute_indicators)
      -> 그 하중 상태의 기준선이 없으면: 성공 사이클만 모아 K개가 되면 기준선 완성 ("학습 중 k/K")
      -> 있으면: 이탈 정도 d -> 지표 점수 -> 건강도
                 최근 M사이클 이동평균의 d가 기준을 N사이클 연속 넘으면 주의/경고

이탈 정도 d = (값 - 기준선 평균) / max(기준선 표준편차, 최소 표준편차), 나쁜 방향만 센다.
지표 점수 = 100 - slope * max(0, d - 1), 0~100.  건강도 = (점수 평균 + 최저 점수) / 2.
ground truth(Gazebo 실제 위치)는 여기 들어오지 않는다. 채점은 노드가 CSV에만 쓴다.
"""

from collections import deque
from dataclasses import dataclass
import statistics

# 건강도에 쓰는 지표: 키 -> (나쁜 방향, 최소 표준편차). 방향: high=커지면 나쁨, low=작아지면, both=양쪽.
HEALTH_INDICATORS = {
    'rot_ratio': ('both', 0.01),
    'rot_resid_rms': ('high', 0.01),       # rad/s
    'trans_resid_pct': ('both', 1.0),      # %
    'track_ratio': ('both', 0.02),
    'amcl_xy_mean': ('high', 0.02),        # m
    'arrive_dist_cm': ('high', 2.0),       # cm
    'arrive_yaw_deg': ('high', 1.0),       # 도
    'arm_time_total': ('high', 0.3),       # s
    'lidar_valid_mean': ('low', 2.0),      # %
    'scan_jitter_ms': ('high', 5.0),       # ms
}

LEARNING, NORMAL, CAUTION, WARNING = 'learning', 'normal', 'caution', 'warning'
STATUS_KO = {LEARNING: '기준선 학습 중', NORMAL: '정상', CAUTION: '주의', WARNING: '경고'}


@dataclass
class PhmSettings:
    """PHM 판정 설정. 의미는 config/health_runner.yaml 주석 참고."""

    k_baseline: int = 8        # K: 기준선을 배우는 성공 사이클 수 (하중 상태마다)
    ma_window: int = 5         # M: 이동평균 창 [사이클]
    n_consecutive: int = 3     # N: 이동평균이 기준을 몇 사이클 연속 넘어야 경보
    caution_d: float = 2.0     # 주의: 평소 흔들림의 2배
    warning_d: float = 3.0     # 경고: 평소 흔들림의 3배
    score_slope: float = 20.0  # 건강도 감점: d 1 초과부터 1당 이만큼
    clip_d: float = 6.0        # 이동평균에 넣기 전 사이클 1회 이탈을 ±이 값으로 자름 (한 번 튄 값 대비)


# ---------- 계산식 ----------
def signed_z(x, mean, std, floor):
    """부호 있는 이탈 (x - 평균) / max(표준편차, 바닥값). 값이나 기준선이 없으면 None."""
    if x is None or mean is None:
        return None
    return (x - mean) / max(std or 0.0, floor)


def bad_side(z, side):
    """부호 있는 이탈 -> 나쁜 방향 이탈 d (0 이상)."""
    if z is None:
        return None
    if side == 'high':
        return max(0.0, z)
    if side == 'low':
        return max(0.0, -z)
    return abs(z)


def indicator_score(d, slope=20.0):
    """이탈 d -> 점수 0~100. d <= 1이면 100, d = 2 -> 80, 3 -> 60, 6 -> 0."""
    if d is None:
        return None
    return min(100.0, max(0.0, 100.0 - slope * max(0.0, d - 1.0)))


def health_score(scores):
    """(지표 점수 평균 + 최저 점수) / 2. 점수가 하나도 없으면 None."""
    vals = [s for s in scores if s is not None]
    if not vals:
        return None
    return (statistics.fmean(vals) + min(vals)) / 2.0


# ---------- 기준선 ----------
class LoadBaseline:
    """하중 상태 하나의 기준선. K개 성공 사이클을 모으면 평균·표준편차를 고정한다."""

    def __init__(self, k, samples=None):
        self.k = k
        self.samples = list(samples or [])   # [{지표 키: 값}]
        self.mean, self.std = {}, {}
        if self.complete:
            self._freeze()

    @property
    def complete(self):
        return len(self.samples) >= self.k

    def add(self, indicators):
        if self.complete:
            return
        self.samples.append({k: indicators.get(k) for k in HEALTH_INDICATORS})
        if self.complete:
            self._freeze()

    def _freeze(self):
        for key in HEALTH_INDICATORS:
            vals = [s[key] for s in self.samples if s.get(key) is not None]
            # 값이 2개 미만이면 표준편차를 못 구하므로 그 지표는 판정하지 않는다
            self.mean[key] = statistics.fmean(vals) if len(vals) >= 2 else None
            self.std[key] = statistics.stdev(vals) if len(vals) >= 2 else None

    def to_dict(self):
        return {'k': self.k, 'samples': self.samples, 'mean': self.mean, 'std': self.std}


@dataclass
class Verdict:
    """사이클 1회 판정 결과."""

    load_state: str
    status: str                    # learning / normal / caution / warning
    learned: int                   # 기준선 학습한 사이클 수
    k: int
    health: float = None           # 0~100 (학습 중이면 None)
    worst: str = None              # 이번 사이클 점수가 가장 낮은 지표
    scores: dict = None            # 지표 -> 점수
    d_ma: dict = None              # 지표 -> 이동평균 이탈 d
    alarm_keys: tuple = ()         # 경보를 만든 지표

    def label(self):
        if self.status == LEARNING:
            return f'{STATUS_KO[LEARNING]} ({self.learned}/{self.k})'
        return STATUS_KO[self.status]


class PhmMonitor:
    """하중 상태별 기준선을 들고 사이클마다 건강도와 예지 경보를 판정한다."""

    def __init__(self, settings=None, saved=None):
        self.s = settings or PhmSettings()
        self.baselines = {}
        self._z = {}      # 하중 -> 지표 -> 최근 M개 잘린 부호 이탈
        self._count = {}  # 하중 -> 지표 -> [주의 연속 수, 경고 연속 수]
        for load, b in (saved or {}).items():
            self.baselines[load] = LoadBaseline(self.s.k_baseline, b.get('samples'))
            # 운영 이력: 러너를 다시 켜도(예: 마모 단계 전환) 이동평균·연속 수가 이어지게
            hist = b.get('history', {})
            self._z[load] = {k: deque(v, maxlen=self.s.ma_window)
                             for k, v in hist.get('z', {}).items()}
            self._count[load] = {k: list(v) for k, v in hist.get('count', {}).items()}

    def baseline(self, load):
        if load not in self.baselines:
            self.baselines[load] = LoadBaseline(self.s.k_baseline)
        return self.baselines[load]

    def reset(self, load):
        """그 하중 상태의 기준선·이동평균을 지우고 다시 학습 (정비·재캘리브레이션 후)."""
        self.baselines.pop(load, None)
        self._z.pop(load, None)
        self._count.pop(load, None)

    def to_dict(self):
        out = {}
        for load, b in self.baselines.items():
            out[load] = b.to_dict()
            out[load]['history'] = {
                'z': {k: list(q) for k, q in self._z.get(load, {}).items()},
                'count': self._count.get(load, {})}
        return out

    def update(self, load, indicators, success):
        """사이클 1회 지표 -> Verdict. 하중이 다르면 기준선도 따로 (다른 하중끼리 비교 안 함)."""
        s = self.s
        base = self.baseline(load)
        if not base.complete:
            # 학습 중: 성공 사이클만 모은다. K개째를 채운 사이클도 학습에 썼으므로 판정은 다음부터
            if success:
                base.add(indicators)
            return Verdict(load, LEARNING, len(base.samples), s.k_baseline)

        zq = self._z.setdefault(load, {})
        cnt = self._count.setdefault(load, {})
        scores, d_ma, alarm_c, alarm_w = {}, {}, [], []
        for key, (side, floor) in HEALTH_INDICATORS.items():
            z = signed_z(indicators.get(key), base.mean.get(key), base.std.get(key), floor)
            scores[key] = indicator_score(bad_side(z, side), s.score_slope)
            q = zq.setdefault(key, deque(maxlen=s.ma_window))
            c = cnt.setdefault(key, [0, 0])
            if z is None:
                continue   # 이번 사이클 값이 없으면 연속 수를 바꾸지 않는다
            q.append(max(-s.clip_d, min(s.clip_d, z)))
            if len(q) < s.ma_window:
                continue   # 창이 다 차기 전에는 경보 판정 안 함
            d = bad_side(statistics.fmean(q), side)
            d_ma[key] = d
            c[0] = c[0] + 1 if d > s.caution_d else 0
            c[1] = c[1] + 1 if d > s.warning_d else 0
            if c[1] >= s.n_consecutive:
                alarm_w.append(key)
            elif c[0] >= s.n_consecutive:
                alarm_c.append(key)

        valid = {k: v for k, v in scores.items() if v is not None}
        worst = min(valid, key=valid.get) if valid else None
        if worst is not None and valid[worst] >= 100.0:
            worst = None   # 모두 100점이면 '가장 나쁜 지표'는 없음 (동점 중 첫 번째가 찍히지 않게)
        status = WARNING if alarm_w else CAUTION if alarm_c else NORMAL
        return Verdict(load, status, len(base.samples), s.k_baseline,
                       health=health_score(scores.values()), worst=worst, scores=scores,
                       d_ma=d_ma, alarm_keys=tuple(alarm_w or alarm_c))
