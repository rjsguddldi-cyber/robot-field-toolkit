"""
PHM 건강지표 (사이클 1회 단위). rclpy 없이 순수 파이썬, 통계만 사용.

사이클 1회 = 한 작업 지점으로 이동 시작 → 도착 확인 → 팔 approach/work/retreat/stowed 끝.
노드(3단계 러너)가 사이클 동안 모은 (시각, 값) 목록을 CycleData에 넣고
compute_indicators()를 부르면 CSV 한 줄로 쓸 수 있는 dict가 나온다.
시각은 모두 시뮬레이션 시간 [s].

연쇄:  odom 잔차(①②) → 위치 불확실성(④) → 도착 오차(⑤) → 팔 작업 위험(⑤⑥) → 사이클 타임·정지
"""

import bisect
from dataclasses import dataclass, field
import math
import statistics

from robot_field_toolkit.core.nav_check import pose_error

# 지표 설명표: 키 -> (번호, 이름, 단위, 하중 영향). 하중 영향이 있는 지표는
# 3단계에서 하중 상태별 기준선으로 비교한다.
INDICATORS = {
    'rot_resid_mean': ('①', '회전 잔차 평균 (IMU − odom 각속도)', 'rad/s', False),
    'rot_resid_rms': ('①', '회전 잔차 RMS', 'rad/s', False),
    'rot_ratio': ('①', '회전 비율 (IMU/odom, 최소제곱 기울기)', '비율', False),
    'trans_resid_pct': ('②', '이동 잔차 (odom 직선 이동량 vs TF)', '%', True),
    'track_ratio': ('③', '구동 추종률 (odom/명령 선속도)', '비율', True),
    'amcl_xy_mean': ('④', 'AMCL 위치 표준편차 평균', 'm', False),
    'amcl_xy_max': ('④', 'AMCL 위치 표준편차 최대', 'm', False),
    'amcl_yaw_mean': ('④', 'AMCL 방향 표준편차 평균', 'rad', False),
    'arrive_dist_cm': ('⑤', '도착 거리 오차 (TF)', 'cm', False),
    'arrive_yaw_deg': ('⑤', '도착 각도 오차 (TF)', '도', False),
    'arm_time_total': ('⑥', '팔 도달 시간 합계', 's', True),
    'lidar_valid_mean': ('⑦', 'LiDAR 유효 비율 평균', '%', False),
    'lidar_valid_min': ('⑦', 'LiDAR 유효 비율 최소', '%', False),
}
ARM_POSES = ('approach', 'work', 'retreat', 'stowed')
TIMING_TOPICS = ('scan', 'odom', 'imu', 'joint_states')


@dataclass
class IndicatorSettings:
    """지표 계산 설정 (판정 기준이 아니라 '어떤 샘플을 쓸지')."""

    rot_w_min: float = 0.1        # ①: |odom 각속도|가 이보다 큰(회전 중) 샘플만 [rad/s]
    pair_max_dt: float = 0.05     # ①: odom과 IMU 샘플 시각 차이가 이보다 크면 짝짓지 않음 [s]
    trans_min_dist: float = 0.2   # ②: TF 직선 이동량이 이보다 짧으면 계산 안 함 [m]
    track_v_min: float = 0.05     # ③: |명령 선속도|가 이보다 큰 샘플만 [m/s]


@dataclass
class CycleData:
    """사이클 1회 동안 모은 데이터. 비어 있는 항목의 지표는 None."""

    odom_twist: list = field(default_factory=list)   # (t, v, w)  /odom twist, 주행 구간
    imu_gyro: list = field(default_factory=list)     # (t, wz)    /imu angular_velocity.z, 주행 구간
    cmd: list = field(default_factory=list)          # (t, v, w)  /cmd_vel_checked, 주행 구간
    odom_start: tuple = None                         # (x, y)     주행 시작 odom 위치
    odom_end: tuple = None                           # (x, y)     도착 odom 위치
    tf_start: tuple = None                           # (x, y)     주행 시작 TF map->base
    tf_end: tuple = None                             # (x, y)     도착 TF map->base
    amcl_cov: list = field(default_factory=list)     # (t, cov_xx, cov_yy, cov_yawyaw)
    goal: tuple = None                               # (x, y, yaw) 목표
    arrival_pose: tuple = None                       # (x, y, yaw) 성공 1초 뒤 TF
    arm_times: dict = field(default_factory=dict)    # 자세 이름 -> 도달 시간 [s]
    scan_valid: list = field(default_factory=list)   # (t, 유효 비율 0~1)
    topic_stamps: dict = field(default_factory=dict)  # 토픽 -> [(받은 시각, 헤더 시각 또는 None)]


# ---------- ① 회전 잔차 ----------
def rotation_residual(odom_twist, imu_gyro, settings=None):
    """
    회전 중 샘플에서 IMU와 odom 각속도 비교. (평균 차이, RMS 차이, 비율, 샘플 수).

    odom 샘플마다 시각이 가장 가까운 IMU 샘플과 짝짓는다.
    비율 = Σ(imu·odom) / Σ(odom²)  (원점을 지나는 최소제곱 기울기)
    """
    st = settings or IndicatorSettings()
    if not odom_twist or not imu_gyro:
        return None, None, None, 0
    imu_t = [s[0] for s in imu_gyro]
    diffs, sxy, sxx = [], 0.0, 0.0
    for t, _, w_odom in odom_twist:
        if abs(w_odom) <= st.rot_w_min:
            continue
        i = bisect.bisect_left(imu_t, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(imu_t)), key=lambda j: abs(imu_t[j] - t))
        if abs(imu_t[best] - t) > st.pair_max_dt:
            continue
        w_imu = imu_gyro[best][1]
        diffs.append(w_imu - w_odom)
        sxy += w_imu * w_odom
        sxx += w_odom * w_odom
    if not diffs:
        return None, None, None, 0
    rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
    return statistics.fmean(diffs), rms, sxy / sxx, len(diffs)


# ---------- ② 이동 잔차 ----------
def translation_residual(odom_start, odom_end, tf_start, tf_end, settings=None):
    """
    직선 이동량 비교 [%] = (odom 이동량 − TF 이동량) / TF 이동량 × 100.

    누적 경로 길이 대신 시작점·끝점 직선 거리를 쓴다 (AMCL이 위치를 고칠 때마다
    경로 길이가 덧붙어 부풀지 않게). +면 odom이 실제보다 많이 갔다고 믿는 것.
    """
    st = settings or IndicatorSettings()
    if None in (odom_start, odom_end, tf_start, tf_end):
        return None
    d_odom = math.dist(odom_start[:2], odom_end[:2])
    d_tf = math.dist(tf_start[:2], tf_end[:2])
    if d_tf < st.trans_min_dist:
        return None
    return (d_odom - d_tf) / d_tf * 100.0


# ---------- ③ 구동 추종률 ----------
def tracking_ratio(odom_twist, cmd, settings=None):
    """
    구동 추종률 = odom 선속도 / 명령 선속도의 평균. 명령은 마지막 값을 유지한다고 본다.

    주의: odom이 바퀴 기반이라 바퀴 마모(반지름 감소)는 여기서 안 보인다.
    """
    st = settings or IndicatorSettings()
    if not odom_twist or not cmd:
        return None
    cmd_t = [c[0] for c in cmd]
    ratios = []
    for t, v_odom, _ in odom_twist:
        i = bisect.bisect_right(cmd_t, t) - 1
        if i < 0:
            continue
        v_cmd = cmd[i][1]
        if abs(v_cmd) > st.track_v_min:
            ratios.append(v_odom / v_cmd)
    return statistics.fmean(ratios) if ratios else None


# ---------- ④ AMCL 불확실성 ----------
def amcl_uncertainty(amcl_cov):
    """(위치 표준편차 평균, 최대, 방향 표준편차 평균). 위치는 √max(cov_xx, cov_yy)."""
    if not amcl_cov:
        return None, None, None
    xy = [math.sqrt(max(cxx, cyy, 0.0)) for _, cxx, cyy, _ in amcl_cov]
    yaw = [math.sqrt(max(cyaw, 0.0)) for *_, cyaw in amcl_cov]
    return statistics.fmean(xy), max(xy), statistics.fmean(yaw)


# ---------- ⑤ 도착 오차 ----------
def arrival_error(goal, arrival_pose):
    """(거리 오차 [cm], 각도 오차 [도]). E401과 같은 계산."""
    if goal is None or arrival_pose is None:
        return None, None
    d, a = pose_error(goal, arrival_pose)
    return d * 100.0, math.degrees(a)


# ---------- ⑦ LiDAR 유효 비율 ----------
def lidar_valid(scan_valid):
    """(평균, 최소) [%]."""
    if not scan_valid:
        return None, None
    vals = [v for _, v in scan_valid]
    return statistics.fmean(vals) * 100.0, min(vals) * 100.0


# ---------- ⑧ 통신·타이밍 ----------
def topic_timing(stamps):
    """
    (수신 간격 평균 [ms], 지터 = 간격 표준편차 [ms], 헤더 지연 평균 [ms]).

    stamps: [(받은 시각, 헤더 시각 또는 None)]. 헤더가 없는 메시지는 지연을 None.
    """
    if len(stamps) < 2:
        return None, None, None
    recv = [r for r, _ in stamps]
    gaps = [(b - a) * 1000.0 for a, b in zip(recv, recv[1:])]
    jitter = statistics.pstdev(gaps) if len(gaps) > 1 else 0.0
    delays = [(r - h) * 1000.0 for r, h in stamps if h is not None]
    return statistics.fmean(gaps), jitter, (statistics.fmean(delays) if delays else None)


# ---------- 묶어서 ----------
def compute_indicators(data, settings=None):
    """사이클 데이터 → 지표 dict (CSV 한 줄). 계산할 수 없는 값은 None."""
    st = settings or IndicatorSettings()
    out = {}
    mean_d, rms_d, ratio, n_rot = rotation_residual(data.odom_twist, data.imu_gyro, st)
    out.update(rot_resid_mean=mean_d, rot_resid_rms=rms_d, rot_ratio=ratio, rot_samples=n_rot)
    out['trans_resid_pct'] = translation_residual(
        data.odom_start, data.odom_end, data.tf_start, data.tf_end, st)
    out['track_ratio'] = tracking_ratio(data.odom_twist, data.cmd, st)
    out['amcl_xy_mean'], out['amcl_xy_max'], out['amcl_yaw_mean'] = amcl_uncertainty(data.amcl_cov)
    out['arrive_dist_cm'], out['arrive_yaw_deg'] = arrival_error(data.goal, data.arrival_pose)
    for pose in ARM_POSES:
        out[f'arm_time_{pose}'] = data.arm_times.get(pose)
    arm = [data.arm_times[p] for p in ARM_POSES if data.arm_times.get(p) is not None]
    out['arm_time_total'] = sum(arm) if len(arm) == len(ARM_POSES) else None
    out['lidar_valid_mean'], out['lidar_valid_min'] = lidar_valid(data.scan_valid)
    for topic in TIMING_TOPICS:
        gap, jit, delay = topic_timing(data.topic_stamps.get(topic, []))
        out[f'{topic}_interval_ms'] = gap
        out[f'{topic}_jitter_ms'] = jit
        out[f'{topic}_delay_ms'] = delay
    return out
