"""
MoMa 사이클 시운전: 계획, 기록, 집계, 리포트. rclpy 없이 순수 파이썬.

사이클 1회 = 작업 지점 하나 방문:
    이동 -> 도착 확인(E401) -> approach -> work -> retreat -> stowed
지점은 순서대로 돌아가며 방문하고(A, B, A, ...), 속도 단계(30 -> 60 -> 100%)마다 N회.
"""

from dataclasses import asdict, dataclass, field
import json
import math

ARM_SEQUENCE = ('approach', 'work', 'retreat', 'stowed')


@dataclass
class PlannedCycle:
    """계획된 사이클 하나."""

    speed: float     # 속도 오버라이드 [%]
    cycle: int       # 그 속도에서 몇 번째 (1부터)
    point: str       # 작업 지점 이름


def build_plan(point_names, speeds=(30, 60, 100), cycles_per_speed=3):
    """속도 단계마다 cycles_per_speed회, 지점은 돌아가며."""
    if not point_names:
        raise ValueError('작업 지점이 없음')
    plan = []
    k = 0
    for speed in speeds:
        for c in range(1, cycles_per_speed + 1):
            plan.append(PlannedCycle(float(speed), c, point_names[k % len(point_names)]))
            k += 1
    return plan


def check_points(points):
    """작업 지점 좌표가 다 채워졌는지. 빈 값이 있으면 ValueError."""
    for name, p in points.items():
        for key in ('x', 'y', 'yaw'):
            if p.get(key) is None:
                raise ValueError(f'작업 지점 {name}의 {key}가 비어 있음 (rviz2 Publish Point로 찍어서 넣기)')


@dataclass
class CycleRecord:
    """사이클 하나의 결과."""

    speed: float
    cycle: int
    point: str
    success: bool = False
    cycle_time: float = 0.0          # 이동 시작 ~ 팔 stowed 완료 [s]
    arrival_dist: float = None       # 도착 거리 오차 [m]
    arrival_yaw: float = None        # 도착 각도 오차 [rad]
    arm_time: float = 0.0            # 팔 approach ~ stowed [s]
    alarms: list = field(default_factory=list)   # 사이클 중 발생한 알람 코드
    fail_reason: str = ''


def _mean(xs):
    return sum(xs) / len(xs) if xs else None


def summarize(records):
    """속도 단계별 요약: 성공률, 평균·최대 도착 오차, 평균 사이클 타임."""
    out = []
    for speed in sorted({r.speed for r in records}):
        rs = [r for r in records if r.speed == speed]
        dists = [r.arrival_dist for r in rs if r.arrival_dist is not None]
        yaws = [r.arrival_yaw for r in rs if r.arrival_yaw is not None]
        ok = [r for r in rs if r.success]
        out.append({
            'speed': speed,
            'cycles': len(rs),
            'success': len(ok),
            'success_rate': len(ok) / len(rs),
            'dist_mean': _mean(dists),
            'dist_max': max(dists) if dists else None,
            'yaw_mean': _mean(yaws),
            'yaw_max': max(yaws) if yaws else None,
            'cycle_time_mean': _mean([r.cycle_time for r in ok]),
        })
    return out


def _cm(v):
    return '-' if v is None else f'{v * 100:.1f}'


def _deg(v):
    return '-' if v is None else f'{math.degrees(v):.1f}'


def _s(v):
    return '-' if v is None else f'{v:.1f}'


def to_markdown(records, summary, meta):
    """Markdown 리포트."""
    lines = [
        '# MoMa 사이클 시운전 리포트',
        '',
        '> **Gazebo 시뮬레이션 기준** 결과입니다. 실제 로봇 수치가 아닙니다.',
        '',
        f"- 시작: {meta.get('started', '-')}",
        f"- 작업 지점: {meta.get('points', '-')}",
        f"- 고장 주입: {meta.get('faults', '-')}",
        f"- 메모: {meta.get('note', '')}",
        '',
        '## 사이클별 결과',
        '',
        '| 속도% | 회차 | 지점 | 성공 | 사이클 타임(s) | 도착 오차(cm) | 도착 오차(도) | 팔 동작(s) | 발생 알람 | 실패 이유 |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for r in records:
        lines.append(
            f'| {r.speed:.0f} | {r.cycle} | {r.point} | {"O" if r.success else "X"} '
            f'| {_s(r.cycle_time)} | {_cm(r.arrival_dist)} | {_deg(r.arrival_yaw)} '
            f'| {_s(r.arm_time)} | {", ".join(r.alarms) or "-"} | {r.fail_reason or "-"} |')
    lines += [
        '',
        '## 속도 단계별 요약',
        '',
        '| 속도% | 성공/회수 | 성공률 | 평균 오차(cm) | 최대 오차(cm) | 평균 각도(도) | 최대 각도(도) | 평균 사이클 타임(s) |',
        '|---|---|---|---|---|---|---|---|',
    ]
    for s in summary:
        lines.append(
            f"| {s['speed']:.0f} | {s['success']}/{s['cycles']} | {s['success_rate'] * 100:.0f}% "
            f"| {_cm(s['dist_mean'])} | {_cm(s['dist_max'])} | {_deg(s['yaw_mean'])} "
            f"| {_deg(s['yaw_max'])} | {_s(s['cycle_time_mean'])} |")
    return '\n'.join(lines) + '\n'


def to_json(records, summary, meta):
    """JSON 리포트 문자열."""
    return json.dumps({
        'basis': 'Gazebo simulation',
        'meta': meta,
        'cycles': [asdict(r) for r in records],
        'summary': summary,
    }, ensure_ascii=False, indent=2)
