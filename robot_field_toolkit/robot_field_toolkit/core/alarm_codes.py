"""
알람 코드표(alarm_codes.yaml) 읽기. rclpy 없이 순수 파이썬.

yaml 구조: 코드 -> name, level, meaning, causes(목록), checks(목록)
"""

from dataclasses import dataclass, field

import yaml

LEVELS = ('WARN', 'ALARM', 'EMERGENCY')
REQUIRED_KEYS = ('name', 'level', 'meaning', 'causes', 'checks')


@dataclass
class AlarmCode:
    """알람 코드 하나."""

    code: str
    name: str
    level: str
    meaning: str
    causes: list = field(default_factory=list)
    checks: list = field(default_factory=list)

    def format_text(self):
        """터미널에 보여 줄 여러 줄 안내문."""
        lines = [
            f'[{self.level}] {self.code} {self.name}',
            f'  의미: {self.meaning}',
            '  점검 순서:',
        ]
        lines += [f'    {i}. {c}' for i, c in enumerate(self.checks, start=1)]
        return '\n'.join(lines)


def parse_alarm_codes(data):
    """yaml에서 읽은 dict를 {코드: AlarmCode}로 바꾼다. 형식이 틀리면 ValueError."""
    codes = {}
    for code, item in data.items():
        missing = [k for k in REQUIRED_KEYS if k not in item]
        if missing:
            raise ValueError(f'{code}: 빠진 항목 {missing}')
        if item['level'] not in LEVELS:
            raise ValueError(f"{code}: level '{item['level']}'은 {LEVELS} 중 하나여야 함")
        codes[code] = AlarmCode(code=code, **{k: item[k] for k in REQUIRED_KEYS})
    return codes


def load_alarm_codes(path):
    """알람 코드 yaml 파일을 읽어서 {코드: AlarmCode}를 돌려준다."""
    with open(path, encoding='utf-8') as f:
        return parse_alarm_codes(yaml.safe_load(f))
