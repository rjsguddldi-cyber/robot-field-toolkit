"""
PHM 4단계 자동 대응: 예지 경보 수준 -> 속도 제한, 팔 정밀 작업 보류, 권고. 순수 파이썬.

경보를 사람이 보고 판단하기 전에, 열화가 사고(도착 오차 -> 팔 작업 실패)로 커지지 않게
먼저 안전 쪽으로 운용을 바꾼다. 해제는 경보 수준이 내려가면 자동.
"""

from dataclasses import dataclass

# 수준 -> (속도 오버라이드 %, 팔 정밀 작업 보류, 권고)  ※ 11/23 초안
RESPONSES = {
    'learning': (100.0, False, ''),
    'normal': (100.0, False, ''),
    'caution': (70.0, False, '추세 감시 강화: 다음 정기 점검에서 해당 지표 확인'),
    'warning': (50.0, True, '재캘리브레이션 권고 (odom 바퀴 반지름·간격)'),
}


@dataclass
class Response:
    """자동 대응 하나."""

    speed: float          # safety_interlock speed_override [%]
    hold_arm: bool        # 팔 정밀 작업(approach~retreat) 보류
    advice: str           # 정비 권고 문구

    def label(self):
        parts = [f'속도 {self.speed:.0f}%']
        if self.hold_arm:
            parts.append('팔 작업 보류')
        if self.advice:
            parts.append(self.advice)
        return ', '.join(parts)


def response_for(status, load='empty', loaded_speed=70.0):
    """
    경보 수준(phm.Verdict.status) + 하중 상태 -> Response. 모르는 수준은 가장 보수적으로.

    하중별 속도 프로파일: empty가 아니면(짐을 들면 관성이 커짐) loaded_speed까지 제한.
    PHM 대응과 겹치면 더 낮은 속도를 쓴다.
    """
    speed, hold, advice = RESPONSES.get(status, RESPONSES['warning'])
    if load != 'empty' and loaded_speed < speed:
        speed = loaded_speed
        advice = ', '.join(a for a in (f'적재({load}) 속도 프로파일', advice) if a)
    return Response(speed, hold, advice)
