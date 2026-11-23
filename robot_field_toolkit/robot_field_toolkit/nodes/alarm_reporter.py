"""
알람 발행 공통 도우미 (ROS 쪽). 감시 노드마다 같은 방식으로 알람을 낸다.

- 판정이 바뀔 때: /field_toolkit/alarms (String JSON) + 터미널 안내
- 1Hz: /diagnostics (WARN->WARN, ALARM/EMERGENCY->ERROR)
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robot_field_toolkit.core.alarm_codes import load_alarm_codes
from std_msgs.msg import String

DIAG_LEVEL = {
    'WARN': DiagnosticStatus.WARN,
    'ALARM': DiagnosticStatus.ERROR,
    'EMERGENCY': DiagnosticStatus.ERROR,
}


def default_alarm_codes_path():
    return os.path.join(
        get_package_share_directory('robot_field_toolkit'), 'config', 'alarm_codes.yaml')


class AlarmReporter:
    """판정 결과(Judgement 목록)를 알람 이벤트와 진단 메시지로 바꾼다."""

    def __init__(self, node, group, codes_file=''):
        """group: 진단 이름 앞부분 (예: 'motion' -> 'field_toolkit/motion: E101 ...')."""
        self.node = node
        self.group = group
        self.codes_file = codes_file or default_alarm_codes_path()
        self.alarm_codes = load_alarm_codes(self.codes_file)
        self.active = {}
        self.latest = []
        self.diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.alarm_pub = node.create_publisher(String, '/field_toolkit/alarms', 10)
        node.create_timer(1.0, self.publish_diagnostics)

    def update(self, t, judgements):
        """판정 결과를 넣는다. 켜짐/꺼짐이 바뀐 코드는 이벤트를 낸다."""
        self.latest = judgements
        for j in judgements:
            if j.active != self.active.get(j.code, False):
                self.active[j.code] = j.active
                self._on_change(t, j)

    def _on_change(self, t, judgement):
        info = self.alarm_codes[judgement.code]
        event = {
            'stamp': round(t, 3),
            'code': info.code,
            'name': info.name,
            'level': info.level,
            'state': 'raised' if judgement.active else 'cleared',
            'detail': judgement.detail,
        }
        self.alarm_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))
        log = self.node.get_logger()
        if judgement.active:
            text = f'\n{info.format_text()}\n  현재값: {judgement.detail}'
            # rclpy 로거는 호출 위치마다 심각도를 기억한다. 한 줄에서 warn·error를 번갈아 부르면
            # ValueError로 노드가 죽으므로 호출 위치를 나눈다 (11/23, DEVLOG 22번)
            if info.level == 'WARN':
                log.warn(text)
            else:
                log.error(text)
        else:
            log.info(f'[해제] {info.code} {info.name}')

    def publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.node.get_clock().now().to_msg()
        for j in self.latest:
            info = self.alarm_codes[j.code]
            st = DiagnosticStatus()
            st.name = f'field_toolkit/{self.group}: {j.code} {info.name}'
            st.hardware_id = 'my_robot'
            if j.active:
                st.level = DIAG_LEVEL[info.level]
                st.message = f'{info.level}: {info.meaning}'
            else:
                st.level = DiagnosticStatus.OK
                st.message = 'OK'
            st.values = [
                KeyValue(key='detail', value=j.detail),
                KeyValue(key='first_check', value=info.checks[0]),
            ]
            arr.status.append(st)
        self.diag_pub.publish(arr)
