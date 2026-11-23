"""core/phm_report.py 테스트 (가짜 CSV 줄)."""

import pytest
from robot_field_toolkit.core.phm_report import build_report, cycles_to_limit, trend

BASE = {'mean': {'rot_ratio': 1.15, 'trans_resid_pct': 2.6, 'arrive_dist_cm': 5.0},
        'std': {'rot_ratio': 0.005, 'trans_resid_pct': 0.2, 'arrive_dist_cm': 0.1}}


def test_trend_slope():
    slope, icpt = trend([(1, 1.0), (2, 2.0), (3, 3.0)])
    assert slope == pytest.approx(1.0) and icpt == pytest.approx(0.0)


def test_cycles_to_limit_up_and_down():
    pts = [(1, 2.0), (2, 3.0), (3, 4.0)]          # 사이클당 +1
    assert cycles_to_limit(pts, 7.0, True) == pytest.approx(3.0)
    assert cycles_to_limit(pts, 3.5, True) == 0, '이미 넘음'
    assert cycles_to_limit(pts, 0.0, False) is None, '아래 기준인데 올라가는 중'
    down = [(1, 1.15), (2, 1.14), (3, 1.13)]
    assert cycles_to_limit(down, 1.10, False) == pytest.approx(3.0)


def rows_for(wears):
    rows = []
    for w in wears:
        for _ in range(4):
            st = 'normal' if w < 0.03 else ('caution' if w < 0.04 else 'warning')
            rows.append({'condition': f'wheel_wear={w:.2f}', 'phm_status': st, 'health': '80',
                         'alarms': 'E402' if w == 0.02 else '',
                         'alarm_indicators': 'rot_ratio' if st != 'normal' else '',
                         'rot_ratio': str(1.15 * (1 - w)), 'trans_resid_pct': str(2.6 + 100 * w),
                         'arrive_dist_cm': '5.0', 'gt_arrive_dist_cm': '5', 'gt_loc_err_cm': '7'})
    return rows


def test_report_says_phm_first_when_old_alarms_never_fire():
    md = build_report(rows_for([0.0, 0.01, 0.02, 0.03, 0.04]), BASE)
    assert '| PHM 주의 | 13 |' in md
    assert 'E105 바퀴 헛돎 | 발생 안 함' in md
    assert '한 번도 뜨지 않았습니다' in md
    assert '재캘리브레이션' in md, '경고면 재캘리브레이션 권고'
    assert 'Gazebo 시뮬레이션 기준' in md
    assert '| wheel_wear=0.02 | 4 |' in md
    assert '| 정상 | 0 | 0 | 4 | 0 |' in md


def test_report_counts_lead_cycles_when_old_alarm_fires():
    rows = rows_for([0.0, 0.03, 0.04])
    rows[-1]['alarms'] = 'E401'
    md = build_report(rows, BASE)
    assert 'PHM이 기존 알람보다 7사이클 먼저' in md


def test_report_quiet_when_nothing_deviates_and_uses_load_speed():
    rows = [{'condition': 'payload_kg=0.5', 'phm_status': 'normal', 'health': '100',
             'alarms': '', 'alarm_indicators': '', 'rot_ratio': '1.15',
             'trans_resid_pct': '2.6', 'arrive_dist_cm': '5.0'} for _ in range(4)]
    md = build_report(rows, BASE, load='loaded_0.5kg')
    assert '벗어난 지표 없음' in md
    assert '속도 70%' in md, '적재 상태면 하중별 속도 프로파일'


def test_report_when_old_alarm_comes_first():
    rows = rows_for([0.0, 0.03])
    rows[1]['alarms'] = 'E105'
    md = build_report(rows, BASE)
    assert '기존 알람이 2번째 사이클에서' in md and '-' not in md.split('먼저')[0][-5:]
