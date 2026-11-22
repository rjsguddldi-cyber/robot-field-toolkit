"""core/phm.py 테스트: 하중별 기준선, 건강도 계산식, 예지 경보."""

import json
import random

import pytest
from robot_field_toolkit.core.phm import (
    bad_side, CAUTION, health_score, indicator_score, LEARNING, NORMAL, PhmMonitor,
    PhmSettings, signed_z, WARNING)

K = PhmSettings().k_baseline


def normal_cycle(rng, trans=0.0, arm=4.2):
    """정상 사이클 지표 (작은 흔들림 포함). trans: 이동 잔차 %, arm: 팔 시간 합계 s."""
    return {
        'rot_ratio': 1.19 + rng.gauss(0, 0.005),
        'rot_resid_rms': 0.19 + rng.gauss(0, 0.005),
        'trans_resid_pct': trans + rng.gauss(0, 0.5),
        'track_ratio': 0.98 + rng.gauss(0, 0.01),
        'amcl_xy_mean': 0.05 + rng.gauss(0, 0.005),
        'arrive_dist_cm': 4.0 + rng.gauss(0, 1.0),
        'arrive_yaw_deg': 2.0 + rng.gauss(0, 0.5),
        'arm_time_total': arm + rng.gauss(0, 0.1),
        'lidar_valid_mean': 99.0 + rng.gauss(0, 0.3),
        'scan_jitter_ms': 10.0 + rng.gauss(0, 2.0),
    }


def learn(mon, rng, load='empty', **kw):
    for _ in range(K):
        v = mon.update(load, normal_cycle(rng, **kw), True)
    return v


# ---------- 계산식 ----------
def test_score_formula_points():
    """점수: d <= 1 -> 100, 2 -> 80, 3 -> 60, 6 이상 -> 0."""
    assert indicator_score(0.5) == 100
    assert indicator_score(2) == pytest.approx(80)
    assert indicator_score(3) == pytest.approx(60)
    assert indicator_score(6) == 0 and indicator_score(9) == 0
    assert indicator_score(None) is None


def test_health_mixes_mean_and_min():
    """9개 100점 + 1개 0점 -> 평균 90, 최저 0 -> 45 (한 지표의 큰 문제가 묻히지 않음)."""
    assert health_score([100] * 9 + [0]) == pytest.approx(45)
    assert health_score([80] * 10) == pytest.approx(80)
    assert health_score([None, None]) is None


def test_bad_side_direction_and_sigma_floor():
    assert bad_side(-2.0, 'high') == 0.0, '작아지는 건 high 지표에서 나쁘지 않음'
    assert bad_side(-2.0, 'low') == 2.0
    assert bad_side(-2.0, 'both') == 2.0
    # 표준편차 0.001처럼 너무 조용한 기준선은 바닥값 2cm로 나눈다
    assert signed_z(5.0, 4.0, 0.001, 2.0) == pytest.approx(0.5)


# ---------- 학습 ----------
def test_learning_progress_and_success_only():
    mon = PhmMonitor()
    rng = random.Random(1)
    v = mon.update('empty', normal_cycle(rng), True)
    assert v.status == LEARNING and v.learned == 1 and v.label() == '기준선 학습 중 (1/8)'
    v = mon.update('empty', normal_cycle(rng), False)
    assert v.learned == 1, '실패 사이클은 기준선에 넣지 않음'
    for _ in range(K - 1):
        v = mon.update('empty', normal_cycle(rng), True)
    assert v.status == LEARNING and v.learned == K, 'K개째 사이클도 학습용, 판정은 다음부터'
    v = mon.update('empty', normal_cycle(rng), True)
    assert v.status == NORMAL and v.health is not None


def test_worst_is_none_when_all_perfect():
    mon = PhmMonitor()
    rng = random.Random(8)
    learn(mon, rng)
    v = mon.update('empty', dict(mon.baselines['empty'].mean), True)
    assert v.health == 100 and v.worst is None


# ---------- 추가 1: 하중 조건별 기준선 ----------
def test_load_change_only_gives_no_alarm():
    """빈 상태로 학습 후 짐을 실으면 (팔 시간·이동 잔차가 달라져도) 경보 없이 학습 중."""
    mon = PhmMonitor()
    rng = random.Random(2)
    learn(mon, rng, 'empty')
    for i in range(20):
        v = mon.update('loaded_2kg', normal_cycle(rng, trans=-4.0, arm=5.5), True)
        if i < K:
            assert v.status == LEARNING, f'{i}회차: 처음 보는 하중은 학습 중'
        else:
            assert v.status == NORMAL, f'{i}회차: 같은 하중끼리 비교하면 정상'
    # 다시 빈 상태로 돌아와도 빈 상태 기준선으로 정상
    for _ in range(10):
        assert mon.update('empty', normal_cycle(rng), True).status == NORMAL


def test_same_load_worsening_gives_caution_then_warning():
    """같은 하중에서 이동 잔차가 점점 커지면(바퀴 마모처럼) 주의 -> 경고."""
    mon = PhmMonitor()
    rng = random.Random(3)
    learn(mon, rng, 'empty')
    statuses = []
    for i in range(30):
        v = mon.update('empty', normal_cycle(rng, trans=0.4 * i), True)
        statuses.append(v.status)
    first_c = statuses.index(CAUTION)
    first_w = statuses.index(WARNING)
    assert 0 < first_c < first_w
    assert NORMAL in statuses[:first_c], '나빠지기 시작한 직후엔 정상'
    assert v.alarm_keys == ('trans_resid_pct',) and v.worst == 'trans_resid_pct'


def test_no_false_alarm_under_normal_conditions():
    """정상 상태 200사이클 동안 경보 0회 (여러 난수 시드)."""
    for seed in range(5):
        mon = PhmMonitor()
        rng = random.Random(100 + seed)
        learn(mon, rng)
        for _ in range(200):
            assert mon.update('empty', normal_cycle(rng), True).status == NORMAL


def test_single_spike_gives_no_alarm():
    """한 사이클만 도착 오차 50cm로 튀어도 경보 없음 (이동평균 + 자르기 + N회 연속)."""
    mon = PhmMonitor()
    rng = random.Random(4)
    learn(mon, rng)
    for _ in range(6):
        mon.update('empty', normal_cycle(rng), True)
    spike = normal_cycle(rng)
    spike['arrive_dist_cm'] = 50.0
    v = mon.update('empty', spike, True)
    assert v.worst == 'arrive_dist_cm' and v.scores['arrive_dist_cm'] == 0, '점수엔 바로 보임'
    for _ in range(10):
        assert mon.update('empty', normal_cycle(rng), True).status == NORMAL


def test_missing_value_skipped_not_alarmed():
    """도착 실패로 도착 오차가 None이어도 판정이 깨지지 않는다."""
    mon = PhmMonitor()
    rng = random.Random(5)
    learn(mon, rng)
    c = normal_cycle(rng)
    c['arrive_dist_cm'] = None
    v = mon.update('empty', c, False)
    assert v.status == NORMAL and v.scores['arrive_dist_cm'] is None


# ---------- 재설정, 저장 ----------
def test_reset_relearns_only_that_load():
    mon = PhmMonitor()
    rng = random.Random(6)
    learn(mon, rng, 'empty')
    learn(mon, rng, 'loaded_2kg', arm=5.5)
    mon.reset('empty')
    assert mon.update('empty', normal_cycle(rng), True).status == LEARNING
    assert mon.update('loaded_2kg', normal_cycle(rng, arm=5.5), True).status == NORMAL


def test_baseline_survives_save_and_load():
    mon = PhmMonitor()
    rng = random.Random(7)
    learn(mon, rng)
    saved = json.loads(json.dumps(mon.to_dict()))
    mon2 = PhmMonitor(saved=saved)
    assert mon2.update('empty', normal_cycle(rng), True).status == NORMAL
    assert mon2.baselines['empty'].mean == pytest.approx(mon.baselines['empty'].mean)
