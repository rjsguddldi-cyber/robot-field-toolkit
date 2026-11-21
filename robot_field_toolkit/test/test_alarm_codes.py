"""config/alarm_codes.yaml 형식 검사."""

from pathlib import Path

import pytest
from robot_field_toolkit.core.alarm_codes import load_alarm_codes, parse_alarm_codes
from robot_field_toolkit.core.interlock import CODES as INTERLOCK_CODES
from robot_field_toolkit.core.motion_check import CODES as MOTION_CODES
from robot_field_toolkit.core.nav_check import CODES as NAV_CODES
from robot_field_toolkit.core.scan_check import CODES as SCAN_CODES
from robot_field_toolkit.core.slip_check import CODES as SLIP_CODES

CODES = MOTION_CODES + SLIP_CODES + SCAN_CODES + INTERLOCK_CODES + NAV_CODES

YAML_PATH = Path(__file__).resolve().parents[1] / 'config' / 'alarm_codes.yaml'


@pytest.fixture(scope='module')
def codes():
    return load_alarm_codes(YAML_PATH)


def test_all_motion_codes_defined(codes):
    """판정 로직이 내는 코드는 모두 코드표에 있어야 한다."""
    for code in CODES:
        assert code in codes


def test_first_check_is_status_display(codes):
    """점검 순서 1번은 항상 '상태 표시 확인'."""
    for item in codes.values():
        assert item.checks[0].startswith('상태 표시 확인'), item.code


def test_causes_and_checks_not_empty(codes):
    for item in codes.values():
        assert item.causes and item.checks, item.code


def test_bad_level_rejected():
    with pytest.raises(ValueError):
        parse_alarm_codes({'E999': {
            'name': 'x', 'level': 'DANGER', 'meaning': 'x', 'causes': ['x'], 'checks': ['x']}})


def test_missing_key_rejected():
    with pytest.raises(ValueError):
        parse_alarm_codes({'E999': {'name': 'x', 'level': 'WARN'}})


def test_format_text_has_numbered_checks(codes):
    text = codes['E101'].format_text()
    assert 'E101' in text and '1. 상태 표시 확인' in text
