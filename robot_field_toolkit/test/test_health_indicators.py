"""core/health_indicators.py 테스트. 가짜 사이클 데이터로 각 지표가 정의대로 나오는지 확인."""

import math

import pytest
from robot_field_toolkit.core.health_indicators import (
    amcl_uncertainty, arrival_error, compute_indicators, CycleData, INDICATORS, lidar_valid,
    rotation_residual, topic_timing, tracking_ratio, translation_residual)


def series(t0, t1, hz, fn):
    """t0~t1 동안 hz로 (t, *fn(t)) 샘플 목록."""
    n = int(round((t1 - t0) * hz))
    return [(t0 + i / hz, *fn(t0 + i / hz)) for i in range(n)]


# ---------- ① 회전 잔차 ----------
def test_rotation_ratio_matches_measured_1_19():
    """11/22 실측처럼 IMU가 odom의 1.19배로 돌면 비율 1.19."""
    odom = series(0, 5, 50, lambda t: (0.0, 1.0))
    imu = series(0, 5, 100, lambda t: (1.19,))
    mean_d, rms_d, ratio, n = rotation_residual(odom, imu)
    assert ratio == pytest.approx(1.19)
    assert mean_d == pytest.approx(0.19) and rms_d == pytest.approx(0.19)
    assert n == 250


def test_rotation_ignores_straight_driving():
    """회전하지 않는 구간(|w| <= 0.1)은 쓰지 않는다."""
    odom = series(0, 5, 50, lambda t: (0.3, 0.02))
    imu = series(0, 5, 100, lambda t: (0.5,))
    assert rotation_residual(odom, imu) == (None, None, None, 0)


def test_rotation_signed_turns_use_regression_ratio():
    """왼쪽·오른쪽 회전이 섞여도 비율은 1.2, 평균 차이는 상쇄돼 0 근처."""
    odom = series(0, 4, 50, lambda t: (0.0, 1.0 if t < 2 else -1.0))
    imu = series(0, 4, 100, lambda t: ((1.2 if t < 2 else -1.2),))
    mean_d, rms_d, ratio, _ = rotation_residual(odom, imu)
    assert ratio == pytest.approx(1.2)
    assert abs(mean_d) < 0.02, '부호가 반대인 회전은 평균에서 상쇄'
    assert rms_d == pytest.approx(0.2, abs=0.01), 'RMS는 크기를 본다'


def test_rotation_skips_samples_without_nearby_imu():
    odom = series(0, 2, 50, lambda t: (0.0, 1.0))
    imu = series(10, 12, 100, lambda t: (1.0,))  # 시각이 안 겹침
    assert rotation_residual(odom, imu)[3] == 0


# ---------- ② 이동 잔차 ----------
def test_translation_residual_positive_when_odom_overestimates():
    """odom은 2.1m 갔다는데 TF는 2.0m → +5%."""
    r = translation_residual((0, 0), (2.1, 0), (1, 1), (1, 3))
    assert r == pytest.approx(5.0)


def test_translation_uses_straight_distance_not_frame():
    """좌표계(odom, map)가 달라도 직선 이동량끼리 비교."""
    r = translation_residual((5, 5), (5, 7), (0, 0), (2, 0))
    assert r == pytest.approx(0.0)


def test_translation_short_move_not_judged():
    assert translation_residual((0, 0), (0.1, 0), (0, 0), (0.1, 0)) is None


# ---------- ③ 구동 추종률 ----------
def test_tracking_ratio_with_held_command():
    """명령 0.2를 한 번만 보내도(유지) odom 0.16이면 0.8."""
    cmd = [(0.0, 0.2, 0.0)]
    odom = series(0.5, 3, 50, lambda t: (0.16, 0.0))
    assert tracking_ratio(odom, cmd) == pytest.approx(0.8)


def test_tracking_ignores_stop_command():
    cmd = [(0.0, 0.0, 0.0)]
    odom = series(0, 2, 50, lambda t: (0.0, 0.0))
    assert tracking_ratio(odom, cmd) is None


# ---------- ④ ⑤ ⑦ ----------
def test_amcl_uncertainty():
    cov = [(0, 0.04, 0.01, 0.01), (1, 0.09, 0.01, 0.04)]  # 표준편차 0.2, 0.3
    m, mx, yaw = amcl_uncertainty(cov)
    assert m == pytest.approx(0.25) and mx == pytest.approx(0.3)
    assert yaw == pytest.approx(0.15)


def test_arrival_error_cm_deg():
    d, a = arrival_error((1.0, 2.0, 0.0), (1.06, 2.08, math.radians(5)))
    assert d == pytest.approx(10.0) and a == pytest.approx(5.0)


def test_lidar_valid_percent():
    assert lidar_valid([(0, 1.0), (1, 0.5)]) == (pytest.approx(75.0), pytest.approx(50.0))


# ---------- ⑧ 통신·타이밍 ----------
def test_timing_regular_topic_has_zero_jitter():
    stamps = [(i * 0.1 + 0.002, i * 0.1) for i in range(20)]  # 10Hz, 헤더보다 2ms 늦게 받음
    gap, jit, delay = topic_timing(stamps)
    assert gap == pytest.approx(100.0) and jit == pytest.approx(0.0, abs=1e-6)
    assert delay == pytest.approx(2.0)


def test_timing_jitter_and_no_header():
    stamps = [(0.0, None), (0.1, None), (0.3, None), (0.4, None)]  # 간격 100, 200, 100ms
    gap, jit, delay = topic_timing(stamps)
    assert gap == pytest.approx(133.33, abs=0.01)
    assert jit == pytest.approx(47.14, abs=0.01)
    assert delay is None


# ---------- 묶어서 ----------
def test_compute_indicators_full_cycle():
    data = CycleData(
        odom_twist=series(0, 4, 50, lambda t: (0.2, 1.0 if t < 1 else 0.0)),
        imu_gyro=series(0, 4, 100, lambda t: ((1.19 if t < 1 else 0.0),)),
        cmd=[(0.0, 0.2, 0.0)],
        odom_start=(0, 0), odom_end=(0.6, 0), tf_start=(0, 0), tf_end=(0.6, 0),
        amcl_cov=[(0, 0.01, 0.01, 0.01)],
        goal=(1, 1, 0), arrival_pose=(1, 1.05, 0),
        arm_times={'approach': 1.7, 'work': 0.6, 'retreat': 1.0, 'stowed': 0.9},
        scan_valid=[(0, 1.0)],
        topic_stamps={'scan': [(i * 0.1, i * 0.1) for i in range(10)]},
    )
    out = compute_indicators(data)
    assert out['rot_ratio'] == pytest.approx(1.19)
    assert out['trans_resid_pct'] == pytest.approx(0.0)
    assert out['track_ratio'] == pytest.approx(1.0)
    assert out['arrive_dist_cm'] == pytest.approx(5.0)
    assert out['arm_time_total'] == pytest.approx(4.2)
    assert out['scan_interval_ms'] == pytest.approx(100.0)
    assert out['imu_interval_ms'] is None, '모으지 않은 토픽은 None'


def test_empty_cycle_gives_none_not_error():
    out = compute_indicators(CycleData())
    assert out['rot_ratio'] is None and out['arm_time_total'] is None
    assert out['lidar_valid_mean'] is None


def test_indicator_table_covers_output_keys():
    """설명표(INDICATORS)의 키는 모두 compute_indicators 출력에 있어야 한다."""
    out = compute_indicators(CycleData())
    for key in INDICATORS:
        assert key in out


def test_load_sensitive_indicators_marked():
    """하중 영향 지표(②③⑥)는 3단계에서 하중별 기준선으로 비교한다."""
    load = {k for k, v in INDICATORS.items() if v[3]}
    assert load == {'trans_resid_pct', 'track_ratio', 'arm_time_total'}
