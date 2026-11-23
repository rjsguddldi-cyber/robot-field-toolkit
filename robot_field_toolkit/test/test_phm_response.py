"""core/phm_response.py 테스트."""

from robot_field_toolkit.core.phm_response import response_for


def test_normal_and_learning_full_speed():
    for s in ('normal', 'learning'):
        r = response_for(s)
        assert r.speed == 100 and not r.hold_arm


def test_caution_limits_speed_only():
    r = response_for('caution')
    assert r.speed == 70 and not r.hold_arm


def test_warning_holds_arm_and_recommends_recalibration():
    r = response_for('warning')
    assert r.speed == 50 and r.hold_arm and '재캘리브레이션' in r.advice
    assert '팔 작업 보류' in r.label()


def test_unknown_status_is_conservative():
    assert response_for('???').hold_arm


def test_loaded_speed_profile():
    r = response_for('normal', 'loaded_0.5kg')
    assert r.speed == 70 and not r.hold_arm and '적재' in r.advice
    assert response_for('warning', 'loaded_0.5kg').speed == 50, 'PHM 대응이 더 낮으면 그쪽'
    assert response_for('normal', 'empty').speed == 100
