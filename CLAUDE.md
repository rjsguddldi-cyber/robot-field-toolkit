# CLAUDE.md — robot-field-toolkit 작업 안내

다음 세션의 Claude가 이 파일만 읽고 이어서 작업할 수 있도록 정리한 문서.
**새 세션을 시작하면 먼저 `git log --oneline`과 `git status`로 아래 "현재 상태"가 맞는지 확인할 것.**

## 역할
- 사용자(건형)는 ROS2 초급자. Claude는 가르치면서 같이 만드는 선생님.
- 한국어로 설명하고, 전문용어는 처음 나올 때 쉽게 풀어 준다.
- 파일마다 "라이브러리/ROS가 정한 이름"과 "작성자가 지은 이름"을 **표로** 구분하고, 이름 바꾸기 실험을 1개씩 제안한다.

## 프로젝트 맥락
- **robot-field-toolkit**: MoMa(Mobile Manipulator) 현장 셋업·유지보수용 진단 툴킷. 로보스타 채용 면접용 개인 프로젝트. **마감 11/26.**
- 핵심 관점: MoMa는 주행과 팔이 각각 정상이어도 둘이 만나는 이음새(상호 인터락, 작업 지점 도착 정확도, 작업 순서)에서 문제가 생긴다. 툴킷은 이음새를 감시하고 **알람 코드 + 점검 순서(돈·시간 안 드는 것부터)** 로 안내한다. 형식은 현장 알람 매뉴얼: 코드 → 의미 → 원인 → 점검 순서.
- 알람 등급(두산 협동로봇 정지 방식):
  - WARN: 정지 안 함, 기록
  - ALARM: Soft Stop(감속 정지) 후 점검
  - EMERGENCY: Quick Stop(즉시 정지), 수동 복구
  - 점검 순서 1번은 항상 "상태 표시 확인".
- 건형은 수업에서 두산 협동로봇 + RealSense 캘리브레이션 + 비전 픽앤플레이스를 실물로 해 봤다. **파지·캘리브레이션은 시뮬레이션으로 다시 만들지 않는다.**
- 면접 포인트: core/ 판정 로직은 ROS가 아니어도 재사용 가능.

## 환경
- WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Python만.
- 작업공간 `~/field_toolkit_ws` (빌드: 이 폴더에서 `colcon build --symlink-install`).
- git 루트 `~/field_toolkit_ws/src` (GitHub: robot-field-toolkit, main).
- Gazebo는 `GALLIUM_DRIVER=d3d12`로 GPU 렌더링(DEVLOG 11/19). 정상 /scan 약 9.2Hz.
- **모든 노드 use_sim_time=True** (RTF가 1보다 낮을 수 있음 → 시뮬레이션 시간 기준 판정).

## 패키지 구조
| 패키지 | 종류 | 내용 |
|---|---|---|
| `my_robot_description` | CMake, 수업 자료 | diff drive + LiDAR + 카메라 URDF(xacro) |
| `my_robot_gazebo` | CMake, 수업 자료 | 월드, 브리지(`params/my_robot_bridge.yaml`), `gazebo.launch.py` |
| `robot_field_toolkit` | ament_python, MIT | 내 패키지 |
| `my_robot_slam`, `my_robot_navigation` | 수업 자료 | 현재 `~/course_pkgs`에 보관. 2층 1단계에서 src로 옮김. Nav2는 AMCL |

robot_field_toolkit 내부 원칙:
- `robot_field_toolkit/core/`: 판정 로직. **rclpy import 금지**, 순수 파이썬. 시간은 인자로 받는다(테스트에서 가짜 시간). pytest로 테스트.
- `robot_field_toolkit/nodes/`: ROS 노드. core를 호출만 하는 얇은 껍데기.
- `config/`: `alarm_codes.yaml`(코드마다 name, level, meaning, causes, checks), 파라미터 yaml. `setup.py` data_files로 설치.
- `test/`: pytest. ament 기본 flake8/pep257 테스트도 통과시켜야 함(멀티라인 docstring은 둘째 줄부터 시작: D213).

## 명령 흐름
- 현재(1층): 명령원(teleop, Nav2) → `/cmd_vel` → fault_injector → `/wheel_cmd` → 브리지 → Gazebo `cmd_vel`
- 2층 3단계 이후 목표: 명령원 → `/cmd_vel` → safety_interlock → `/cmd_vel_checked` → fault_injector → `/wheel_cmd`
  - 이때 motion_monitor의 명령 속도와 E103 비교 기준도 `/cmd_vel_checked`로 바꾼다(인터락 차단 때문에 E101 오알람 금지).

## 현재 상태 (2026-11-20 기준)
1층 1단계와 2층 1단계는 커밋됨. 1층 2~4단계는 코드 작성·값 확정·Gazebo 확인 완료, 커밋 대기. 다음은 2층 2단계(2축 팔).

| 1층 단계 | 상태 |
|---|---|
| 1단계 브리지 변경 + `nodes/fault_injector.py` | 완료, 커밋 982de93. Gazebo·Nav2에서 stuck 정지 확인 |
| 2단계 `core/motion_check.py` + `test/test_motion_check.py` | 작성·pytest 21개 통과. 판정 숫자 11/20 건형 확정(초안 그대로). 미커밋 |
| 3단계 `config/alarm_codes.yaml`, `core/alarm_codes.py`, `nodes/motion_monitor.py`, `config/motion_monitor.yaml`, `launch/toolkit_base.launch.py` | 작성·빌드·lint·pytest(총 30개) 통과. 등급·점검 순서 11/20 건형 확정(E101 ALARM, E102 WARN, E103 ALARM, E104 WARN). 11/21 Gazebo에서 mode별 E101~E104 발생·해제 확인. launch에 diagnostic_aggregator 포함(rqt_robot_monitor는 /diagnostics_agg를 봄). rqt는 WSLg 렉으로 안 쓰기로 함(건형 결정). 미커밋 |
| 4단계 DEVLOG 11/20 막힌 것 5개 + README 알람 코드 표 | 작성. README 체크 완료. 미커밋 |

| 2층 단계 | 상태 |
|---|---|
| 1단계 SLAM + Nav2 | 완료, 커밋 e00c91a. 건형이 Nav2 Goal 주행, stuck 정지, normal 복귀 확인 |

1층 설계 메모 (motion_check):
- 판정 숫자는 `MotionThresholds` 한곳 + `config/motion_monitor.yaml`(같은 이름의 파라미터).
- 실제/명령 비율: 움직이라는 축(v, w)마다 구해서 **큰 쪽** 사용(오알람 줄이기). 명령 반대 방향 움직임은 0으로 봄.
- 마지막 명령을 유지(Gazebo DiffDrive도 마지막 명령 유지, teleop은 키 누를 때만 발행).
- 해제는 조건이 `clear_time`(1.0s) 동안 계속 사라져야 함(깜빡임 방지). E103은 `loss_min_cmds`(5) 미만이면 판정 안 함.
- odom이 `odom_timeout`(0.5s) 넘게 없으면 E101/E102/E104 판정 보류.
- motion_monitor는 판정 10Hz, /diagnostics 1Hz, /field_toolkit/alarms는 변화 때만 JSON(stamp, code, name, level, state raised/cleared, detail).
- 3층 인터락 이후 `cmd_topic` 파라미터를 `cmd_vel_checked`로 바꿀 것.

2층 1단계 알아 둔 것:
- Nav2 /cmd_vel 타입은 Twist(fault_injector와 일치).
- cmd_vel 고리 수정: controller_server·behavior_server → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel`.
- Nav2 주행 노드는 `TimerAction(10초)` 뒤에 켜짐. 실패하면 Nav2를 껐다 켜야 함(manage_nodes STARTUP 재요청은 이미 active인 노드 때문에 실패).
- 진단 팁: Nav2 노드 로그는 `~/.ros/log/<노드이름>_<pid>_*.log`에 남는다.
- `--symlink-install`이어도 CMake `install(DIRECTORY maps)`는 빌드 시점의 파일만 링크 → 지도 저장 후 다시 빌드.
- 브리지가 /cmd_vel을 직접 받지 않으므로 SLAM 지도 만들 때도 fault_injector를 켜야 teleop으로 움직인다.
- map_saver_cli는 `-p save_map_timeout:=10.0` 필요(WSL 디스커버리 느림).
- `ros2 topic hz`는 처음부터 평균 → `--window 20`으로 볼 것.

fault_injector 요약: 파라미터 `mode`(normal/stuck/weak/drop/drift, 실행 중 `ros2 param set` 가능), 모르는 값이면 경고 1회 후 normal. `/fault_injector/mode`(String) 1Hz. stuck=0, weak=40%, drop=50% 버림, drift=w+0.3.

## 2층 계획 (1층 완료 후, 순서대로 한 단계씩)
1. **SLAM 지도 + Nav2**: course_pkgs → src, 빌드. README 구성·출처 추가. 지도(yaml, pgm)는 저장소 안(예: my_robot_navigation/maps), launch는 패키지 경로로 찾게. Nav2 `/cmd_vel` 타입을 `ros2 topic info -v`로 확인(Twist vs TwistStamped) → 안 맞으면 제안만 하고 건형 결정. **2시간 안에 Nav2가 안 뜨면 멈추고 원인 보고**(Nav2 없는 계획으로 전환). 확인: rviz2 2D Pose Estimate → Nav2 Goal → fault_injector 경유 이동, stuck에서 E101.
2. **2축 팔**: URDF에 팔 받침 + 어깨·팔꿈치 pitch revolute(한계·관성 포함). 접힘 자세에서 LiDAR 스캔 평면 가리면 안 됨(펼친 자세에서 가리면 DEVLOG에 기록·보고). Gazebo 관절 위치 제어 + 관절 상태 발행 + 브리지, 기존 바퀴 joint_states와 충돌 금지. `nodes/arm_controller.py`: yaml의 이름 붙은 자세 stowed → approach → work → retreat, 현재 자세 이름과 상태(moving/reached) 발행. 각도 초안은 표로 확인받기.
3. **인터락 E301~E303 + 속도 오버라이드**: `nodes/safety_interlock.py`. E301 팔 비stowed인데 주행 → 차단, E302 주행 중 팔 요청 → 거부, E303 팔 목표 도달 시간 초과. `speed_override`(%) 파라미터(두산 티치펜던트 이동속도%, 시운전 30→60→100%). fault_injector 입력 토픽을 파라미터로 `/cmd_vel_checked`. 판정은 core/ + pytest. 인터락 차단 시 E101 안 뜨는지 pytest와 실제 실행 둘 다 확인.
4. **LiDAR 고장 + sensor_monitor E2xx**: 브리지 scan → ROS `/scan_raw`. `nodes/scan_fault_injector.py` `/scan_raw`→`/scan`, mode normal/occlude(가린 구간은 유효 범위 밖 값)/dropout. `core/scan_check.py` + `nodes/sensor_monitor.py`: E201 가림(유효 거리값 비율), E202 끊김(수신 주기). diagnostics·alarms는 motion_monitor와 같은 방식. Nav2 주행 중 occlude 시 위치 추정 흔들림 관찰 → DEVLOG.
5. **nav_monitor E401~E403**: E401 도착 오차 초과 → 팔 작업 금지 신호(인터락이 stowed 외 거부), E402 /amcl_pose 공분산 초과 지속, E403 복구 반복 또는 남은 거리 정체. 목표 좌표·복구 횟수 얻는 방법(NavigateToPose 피드백 distance_remaining, number_of_recoveries 등)을 Jazzy 기준으로 조사·제안 후 확인받고 구현. `core/nav_check.py` + pytest.
6. **MoMa 사이클 시운전 러너**: 작업 지점 A, B는 config yaml(건형이 rviz2 Publish Point `/clicked_point`로 찍음). `nodes/commissioning_runner.py` 상태 머신: 이동 → 도착 확인(E401) → approach → work → retreat → stowed → 다음 지점. mermaid 흐름도도 작성. speed_override 30→60→100%, 단계마다 N회(기본 3). `/field_toolkit/alarms` 구독해 사이클별 알람 기록. 리포트 `~/field_toolkit_reports/`에 Markdown + JSON(맨 위 "Gazebo 시뮬레이션 기준"), 집계는 `core/commissioning.py` + pytest.

## 작업 규칙
- **한 단계씩만.** 각 단계 끝에 멈추고 ① 만든·바꾼 파일 설명 ② 확인 방법(터미널별 명령) ③ 이름 바꾸기 실험 1개.
- 판정 숫자, 알람 등급, 점검 순서, 자세 각도 같은 값은 **초안을 표로** 보여 주고 의미와 바꿨을 때 영향을 설명한 뒤 확인받는다. 최종 결정은 건형.
- Gazebo, rviz2, teleop, Nav2 등 GUI·상시 실행 명령은 Claude가 실행하지 말고 알려 준다. colcon build, pytest, git은 Claude가 해도 된다.
- 건형이 직접 확인하고 자기 말로 설명한 뒤 **"커밋해"라고 하기 전에는 커밋 금지.** 커밋 메시지 형식: `feat: ...`, `docs: ...` (한국어).
- 수업 패키지를 바꿀 때는 무엇을 왜 바꿨는지 `docs/DEVLOG.md`에 남긴다.
- DEVLOG의 막힌 문제는 **[증상 → 진단 도구·방법 → 조치 → 전후 수치]** 형식, **직접 잰 숫자만**. 내용은 건형에게 물어보고 건형의 말로 적는다.
- 새 알람 코드는 `config/alarm_codes.yaml`에 1층과 같은 형식(name, level, meaning, causes, checks)으로 추가.
- 단계마다 마무리: README "진행 상황" 체크박스와 알람 코드 표 갱신, DEVLOG 추가.
- **하지 말 것**: 파지·캘리브레이션 구현, 자체 경로 계획·회피 알고리즘, 블랙박스·로그 번들(3층에서 함).
