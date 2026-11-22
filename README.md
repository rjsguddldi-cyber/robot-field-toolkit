# robot-field-toolkit
MoMa(Mobile Manipulator) 현장 셋업·유지보수를 돕는 진단 툴킷입니다.
주행과 팔이 각각 정상이어도 둘이 만나는 지점에서 생기는 문제를 감지하고,
알람 코드와 점검 순서, 시운전 리포트, 로그 번들로 원인 추적을 돕습니다.

> 개발 중 (2026-11-26 완성 목표)

## 구성
| 패키지 | 내용 |
|---|---|
| `my_robot_description` | 시뮬레이션 로봇 모델 (수업 자료 기반, LiDAR 포함) |
| `my_robot_gazebo` | Gazebo 월드, 브리지, 실행 파일 (수업 자료 기반) |
| `my_robot_slam` | slam_toolbox 지도 작성 실행 파일과 설정 (수업 자료 기반) |
| `my_robot_navigation` | Nav2(AMCL) 주행 실행 파일, 설정, 지도 (수업 자료 기반) |
| `robot_field_toolkit` | 고장 주입, 감시, 알람, 시운전, 로그 수집 (직접 개발) |

## 개발 환경
Ubuntu 24.04 (WSL2), ROS 2 Jazzy, Gazebo Harmonic, Python

## 진행 상황
- [x] 작업공간 구성, LiDAR 동작 확인
- [x] 고장 주입 (E1xx)
- [x] 동작 감시와 알람 (E101~E104)
- [x] SLAM 지도와 Nav2 주행 연동
- [x] 2축 팔과 자세 제어
- [x] 주행·팔 인터락 (E3xx)과 속도 오버라이드
- [x] LiDAR 고장 감시 (E2xx)
- [ ] Nav2 위치·도착 감시 (E4xx), 바퀴 헛돎 (E105) — `wip` 브랜치, 재시험 중
- [ ] MoMa 사이클 시운전 리포트 — `wip` 브랜치
- [ ] 알람 이력과 로그 번들

## 알람 코드
현장 알람 매뉴얼처럼 **코드 → 의미 → 원인 → 점검 순서(돈·시간 안 드는 것부터)** 로 안내합니다.
전체 내용은 [config/alarm_codes.yaml](robot_field_toolkit/config/alarm_codes.yaml)에 있습니다.

| 등급 | 동작 (두산 협동로봇 정지 방식) |
|---|---|
| WARN | 정지 안 함, 기록 |
| ALARM | Soft Stop: 감속 정지 후 점검 |
| EMERGENCY | Quick Stop: 즉시 정지, 수동 복구 |

| 코드 | 이름 | 등급 | 판정 기준 |
|---|---|---|---|
| E101 | 구동 정지 | ALARM | 주행 명령 중 실제 속도가 명령의 20% 미만, 1.0초 이상 |
| E102 | 출력 저하 | WARN | 실제/명령 20~70%, 2.0초 이상 |
| E103 | 명령 전달 손실 | ALARM | 최근 2초 구동기 입력 수 < 상위 명령 수의 70% |
| E104 | 방향 이탈 | WARN | 직진 명령 중 실제 회전 0.15 rad/s 초과, 1.5초 이상 |
| E105 | 바퀴 헛돎 | ALARM | 바퀴 odom 2초 15cm 이상 이동인데 LiDAR 스캔 변화 3cm 미만 (위치 재설정까지 유지, 팔 작업 금지) |
| E201 | 센서 가림 | ALARM* | LiDAR 유효 거리값 60% 미만, 2.0초 이상 |
| E202 | 센서 끊김 | ALARM* | LiDAR 수신 5Hz 미만 또는 1초 넘게 수신 없음 |
| E301 | 팔 펼침 중 주행 차단 | WARN* | 팔이 접힘이 아닌데 주행 명령 → 주행 차단 (펼친 채 주행하면 무게중심이 지지면 가장자리로 가서 넘어짐 위험, 풋프린트가 커져 충돌 위험) |
| E302 | 주행 중 팔 요청 거부 | WARN* | 주행 명령 또는 실제 움직임 중 팔 요청 → 거부 |
| E303 | 팔 도달 시간 초과 | ALARM* | 팔이 8초 안에 목표 자세 도달 못 함 |
| E401 | 작업 지점 도착 오차 초과 | WARN* | 도착 후 오차 10cm 또는 8.6도 초과 → 팔 작업 금지 |
| E402 | 위치 상실 | ALARM* | AMCL 표준편차 0.35m 또는 0.45rad 초과, 2.0초 이상 (기준 확정) |
| E403 | 경로 차단 | ALARM* | 복구 3회 이상 또는 15초 동안 남은 거리 0.2m도 안 줄어듦 |

\* 2층 코드의 등급과 기준은 초안입니다 (확정 전).

## 실행 구조
```
명령:  teleop / Nav2 / 시운전 러너 -> /cmd_vel -> safety_interlock(속도 %) -> /cmd_vel_checked
       -> fault_injector -> /wheel_cmd -> Gazebo
팔:    /arm/pose_request -> safety_interlock -> /arm/pose_command -> arm_controller -> Gazebo
LiDAR: Gazebo -> /scan_raw -> scan_fault_injector -> /scan -> SLAM, Nav2, sensor_monitor
감시:  motion_monitor(E101~E105), sensor_monitor(E2xx), safety_interlock(E3xx), nav_monitor(E4xx)
       -> /diagnostics, /field_toolkit/alarms
```
시운전 상태 흐름: [docs/commissioning_flow.md](docs/commissioning_flow.md)

## 출처
`my_robot_description`, `my_robot_gazebo`, `my_robot_slam`, `my_robot_navigation`은 로보스타 채용연계 지능형 로봇 엔지니어 양성과정(경기테크노파크) 수업 자료를 바탕으로 합니다.

## 개발 기록
막힌 점과 해결 과정은 [docs/DEVLOG.md](docs/DEVLOG.md)에 있습니다.
