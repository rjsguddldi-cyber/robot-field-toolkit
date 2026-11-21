# MoMa 사이클 시운전 상태 흐름

`robot_field_toolkit/nodes/commissioning_runner.py`의 상태 머신입니다.
사이클 1회 = 작업 지점 하나 방문. 속도 오버라이드 30% → 60% → 100% 단계마다 N회(기본 3회).

```mermaid
stateDiagram-v2
    [*] --> START
    START --> SET_SPEED: Nav2 준비됨
    SET_SPEED --> NAVIGATE: speed_override 설정 (속도 단계가 바뀔 때만)
    NAVIGATE --> SETTLE: 도착(성공)
    NAVIGATE --> RECORD: 이동 실패 / 시간 초과
    SETTLE --> CHECK_ARRIVAL: settle_time 대기
    CHECK_ARRIVAL --> RECORD: E401 팔 작업 금지 (도착 오차 초과)
    CHECK_ARRIVAL --> APPROACH: 오차 정상
    APPROACH --> WORK: reached
    WORK --> RETREAT: reached
    RETREAT --> STOWED: reached
    STOWED --> RECORD: reached (성공)
    APPROACH --> RECORD: 시간 초과 (stowed 요청)
    WORK --> RECORD: 시간 초과 (stowed 요청)
    RETREAT --> RECORD: 시간 초과 (stowed 요청)
    RECORD --> SET_SPEED: 다음 사이클
    RECORD --> REPORT: 모든 사이클 끝
    REPORT --> [*]: Markdown + JSON 저장, 속도 100% 복귀
```

- 팔 요청은 `/arm/pose_request` → safety_interlock → arm_controller (주행 중이면 E302로 거부)
- 사이클 중 `/field_toolkit/alarms`에서 raised된 알람 코드를 사이클 기록에 남김
