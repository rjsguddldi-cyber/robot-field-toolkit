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
- 알람 등급(두산 협동로봇 정지 방식): WARN(정지 안 함, 기록) / ALARM(Soft Stop 감속 정지 후 점검) / EMERGENCY(Quick Stop 즉시 정지, 수동 복구). 점검 순서 1번은 항상 "상태 표시 확인".
- 건형은 수업에서 두산 협동로봇 + RealSense 캘리브레이션 + 비전 픽앤플레이스를 실물로 해 봤다. **파지·캘리브레이션은 시뮬레이션으로 다시 만들지 않는다.**
- 면접 포인트:
  - core/ 판정 로직은 ROS가 아니어도 재사용 가능 (rclpy 없음, 시간은 인자).
  - 인터락이 막은 명령으로 E101 오알람이 안 나게 비교 기준을 /cmd_vel_checked로 둔 것.
  - Nav2는 25cm 안이면 "도착"이지만 팔 작업엔 부족 → E401로 작업 금지.
  - 벽에 막혀 바퀴만 돌 때 Nav2가 거짓 성공 → 바퀴 정보만 믿는 감시(E101, E401)는 못 잡음 → odom과 LiDAR 교차 확인(E105).

## 환경
- WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Python만. CPU 6코어, RAM 8GB.
- 작업공간 `~/field_toolkit_ws` (빌드: 이 폴더에서 `colcon build --symlink-install`).
- git 루트 `~/field_toolkit_ws/src` (GitHub: robot-field-toolkit). 브랜치: **main**(Gazebo 확인 끝난 것) / **wip**(재시험 남은 것, main 위에 1커밋). 11/22 둘 다 push.
- Gazebo는 `GALLIUM_DRIVER=d3d12`(.bashrc)로 GPU 렌더링. 정상 /scan 약 9.2Hz(실시간 기준), 약 10Hz(시뮬레이션 시간 기준).
- **모든 노드 use_sim_time=True.**
- WSL DDS 디스커버리가 느림: Claude 셸에서 `ros2 topic echo/hz`가 자주 시간 초과. **실행 중 시스템 확인은 건형에게 터미널 명령과 기대 결과를 주고 결과를 받는다** (11/21 건형 요청). 빌드·pytest·로그 파일(`~/.ros/log`) 읽기는 Claude가 직접.
- rqt_robot_monitor는 WSLg 화면 렉이 심해서 안 씀(건형 결정). 알람은 터미널(launch 출력, `ros2 topic echo /field_toolkit/alarms --field data`)로 본다.

## 패키지 구조
| 패키지 | 종류 | 내용 |
|---|---|---|
| `my_robot_description` | CMake, 수업 자료 | diff drive + LiDAR + 카메라 + **2축 팔(2층 추가)** URDF(xacro), `rviz/my_robot.rviz` |
| `my_robot_gazebo` | CMake, 수업 자료 | 월드 `simple_room`, 브리지 `params/my_robot_bridge.yaml`, `gazebo.launch.py` |
| `my_robot_slam` | CMake, 수업 자료 | slam_toolbox `mapping.launch.py`, `mapping_rviz.launch.xml` |
| `my_robot_navigation` | CMake, 수업 자료 | Nav2(AMCL) `navigation.launch.py`, `navigation_rviz.launch.xml`, `params/nav2_params.yaml`, 지도 `maps/simple_room.yaml/.pgm` |
| `robot_field_toolkit` | ament_python, MIT | 내 패키지 |

robot_field_toolkit 구조:
- `core/` (rclpy 금지, pytest): `common.py`(Judgement, Debounce, 각도 도구), `motion_check.py`(E101~E104), `slip_check.py`(E105), `scan_check.py`(E201·E202, occlude 계산), `interlock.py`(E301~E303, 속도 %), `nav_check.py`(E401~E403, pose_error), `arm_pose.py`(자세 추적), `commissioning.py`(계획·집계·리포트), `alarm_codes.py`(코드표 읽기)
- `nodes/` (얇은 껍데기): `fault_injector`, `motion_monitor`(E101~E105), `arm_controller`, `safety_interlock`(E301~E303), `scan_fault_injector`, `sensor_monitor`(E201·E202), `nav_monitor`(E401~E403), `commissioning_runner`, 공통 `alarm_reporter.py`(AlarmReporter: /diagnostics 1Hz + /field_toolkit/alarms JSON + 터미널 안내)
- `config/`: `alarm_codes.yaml`, `motion_monitor.yaml`, `safety_interlock.yaml`, `sensor_monitor.yaml`(scan_fault_injector 포함), `nav_monitor.yaml`, `arm_poses.yaml`, `commissioning.yaml`, `work_points.yaml`, `diagnostic_aggregator.yaml`
- `launch/`: `toolkit_base.launch.py`(툴킷 노드 7개 + diagnostic_aggregator), `commissioning.launch.py`
- `test_assets/`: `wall_partial.sdf`(3m), `wall_full.sdf`(10m) 고정 벽 (E403·E105 시험)
- `test/`: pytest 92개 + ament flake8/pep257 통과 (11/22 기준). 멀티라인 docstring은 둘째 줄부터(D213), 첫 단어 대문자 규칙(D403) 주의, import는 알파벳순(rcl_interfaces가 rclpy보다 앞).
- `docs/commissioning_flow.md`: 시운전 상태 흐름 mermaid.

## 신호 흐름
- 주행: 명령원(teleop, Nav2, 시운전 러너) → `/cmd_vel` → safety_interlock(E301 차단, speed_override %) → `/cmd_vel_checked` → fault_injector(input_topic) → `/wheel_cmd` → 브리지 → Gazebo `cmd_vel`
- Nav2 내부: controller_server·behavior_server → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel` (수업 launch의 되먹임 고리 수정)
- 팔: `/arm/pose_request` → safety_interlock(E302 거부, 작업 금지) → `/arm/pose_command` → arm_controller → `/arm/shoulder_cmd`, `/arm/elbow_cmd`(Float64) → 브리지 → Gazebo JointPositionController. 상태 `/arm/state`(JSON pose/status/elapsed/error)
- LiDAR: Gazebo scan → 브리지 → `/scan_raw` → scan_fault_injector → `/scan` (SLAM·Nav2·sensor_monitor·motion_monitor). **toolkit_base.launch.py 없이는 /scan이 없다** (SLAM 지도 만들 때도 켜야 함).
- 팔 작업 금지: nav_monitor `/field_toolkit/arm_work_inhibit`(E401) + safety_interlock이 `/field_toolkit/alarms`에서 `inhibit_codes`(기본 E105) 활성 여부를 봄.
- E105 해제: `/initialpose`(rviz2 2D Pose Estimate)를 받으면 motion_monitor가 초기화.

## 실행 순서 (이 순서를 지킬 것)
1. `ros2 launch my_robot_gazebo gazebo.launch.py`
2. `ros2 launch robot_field_toolkit toolkit_base.launch.py`
3. `ros2 launch my_robot_navigation navigation.launch.py` (10~15초 뒤 `Managed nodes are active`)
4. `ros2 launch my_robot_navigation navigation_rviz.launch.xml` → 2D Pose Estimate
5. (시운전) `ros2 launch robot_field_toolkit commissioning.launch.py note:='메모'`
- 고장 주입: `ros2 param set /fault_injector mode stuck|weak|drop|drift|normal`, `ros2 param set /scan_fault_injector mode occlude|dropout|normal` (`occlude_width_deg`, `dropout_rate_hz`)
- 팔: `ros2 topic pub --once /arm/pose_request std_msgs/msg/String "{data: approach}"`
- 속도: `ros2 param set /safety_interlock speed_override 30.0`
- 벽: `ros2 run ros_gz_sim create -world simple_room -name wall_partial -file $(ros2 pkg prefix robot_field_toolkit)/share/robot_field_toolkit/test_assets/wall_partial.sdf -x 1.0 -y 0.0 -z 0.25` / 지우기 `gz service -s /world/simple_room/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "wall_partial" type: MODEL'`
- 지도 좌표 참고: 지도 10m×10m(원점 -5,-5), 장애물 대략 x -2.5~-1.5/y 2.5~3.5, x -2.5~-1.5/y -2.5~-1.5, x 1.5~2.5/y 1.5~2.5. 원점 주변은 비어 있음.

## 현재 상태 (2026-11-22 기준)
빌드·pytest·lint는 두 브랜치 모두 통과 (main 64개, wip 92개).

**main** (Gazebo 확인 끝남), 커밋 순서:
`982de93` fault_injector / `e00c91a` SLAM·Nav2 연동 / `6736e1d` 동작 판정 core / `631306a` 동작 감시·E101~E104 / `29139f5` 1층 기록 / `297ef37` 2축 팔 / `3002fc7` 인터락 E301~E303 + 속도 % / `56dca2f` LiDAR E201·E202 / 그다음 CLAUDE.md·문서 커밋

**wip** (main + 1커밋, 재시험 남음): 주행 감시 E401~E403(nav_monitor), 바퀴 헛돎 E105(motion_monitor), 시운전 러너, Nav2 robot_radius 0.20, test_assets 고정 벽, docs/commissioning_flow.md, README·DEVLOG 전체판.
- wip에서 `toolkit_base.launch.py`는 nav_monitor까지 켬. motion_monitor는 /scan·/initialpose도 구독(E105).
- 확인이 끝나면 wip을 main에 합치기 (건형 결정).

| 단계 | 상태 |
|---|---|
| 1층 (E101~E104, fault_injector, motion_monitor) | main. 11/21 Gazebo에서 mode별 발생·해제 확인 |
| 2층 1단계 SLAM+Nav2 | main. 지도 저장, Nav2 주행, stuck 정지 확인 |
| 2층 2단계 2축 팔 | main. 네 자세 도달 0.6~1.7s, 네 자세 모두 LiDAR 가림 0개, E303 시험 OK, PID 게인 조정으로 적분 누적 해결 |
| 2층 3단계 인터락 | main. 팔 펼침 중 주행 → 0으로 차단 + E301, E101 안 뜸 / 30% → 0.06m/s / 주행 중 팔 요청 → E302 거부 / 150% 거부 |
| 2층 4단계 LiDAR | main. occlude → E201, dropout → E202, normal → 해제 |
| 2층 5단계 nav_monitor | **wip, 재시험 대기.** E401은 TF 기준으로 바꾼 뒤 15.8cm 측정(정상 동작). E403(진척 없음) 발생 확인. E402 기준 0.35m/0.45rad. 벽 시험 중 바퀴 헛돎 발견 → robot_radius 0.20 + E105 추가 |
| 2층 6단계 시운전 러너 | **wip.** 코드·pytest만, Gazebo 실행 전. `config/work_points.yaml`의 A, B 좌표가 null → 건형이 rviz2 Publish Point(`/clicked_point`)로 찍어서 채워야 실행됨 |

### 다음 할 일 (순서대로, wip 브랜치에서)
1. `git checkout wip` → colcon build → **5단계 재시험** (Nav2와 툴킷 재시작 후): wall_partial 너머 Goal → 돌아가는지(robot_radius 0.20 효과) / 걸리면 E105가 뜨는지, 2D Pose Estimate로 해제되는지 → wall_full로 E403 → occlude 300도로 E201 → E402 순서(안 떠도 결과로 기록). DEVLOG 11번·13번 "전후" 채우기.
2. **초안 값 확정** (아래 표) — 건형에게 표로 보여 주고 결정받기.
3. 6단계: Publish Point로 A, B 찍기 → 시운전 normal 1회 + LiDAR occlude 1회 → 리포트 2개 비교 → DEVLOG에 직접 잰 숫자.
4. 확인 끝난 것부터 main에 합치기(건형 "커밋해"/"합쳐" 후), README 진행 상황 체크, push.

### 알람 코드와 확정 여부
| 코드 | 이름 | 등급 | 기준 | 확정 |
|---|---|---|---|---|
| E101 | 구동 정지 | ALARM | 실제/명령 < 20%, 1.0s | ✅ 11/20 |
| E102 | 출력 저하 | WARN | 20~70%, 2.0s | ✅ 11/20 |
| E103 | 명령 전달 손실 | ALARM | 2s 동안 wheel_cmd 수 < 명령 수 70% | ✅ 11/20 |
| E104 | 방향 이탈 | WARN | 직진 중 \|w\| > 0.15, 1.5s | ✅ 11/20 |
| E105 | 바퀴 헛돎 | ALARM | odom 2s 15cm 이상인데 스캔 변화 중앙값 < 3cm, 재설정까지 유지 | ✅ 11/22 (방식·값) |
| E201 | 센서 가림 | ALARM | 유효 거리값 < 60%, 2.0s | 초안 |
| E202 | 센서 끊김 | ALARM | < 5Hz 또는 1s 무수신 | 초안 |
| E301 | 팔 펼침 중 주행 차단 | WARN | 팔 stowed 아님 + 주행 명령 | 초안 |
| E302 | 주행 중 팔 요청 거부 | WARN | 거부 후 3s 표시 | 초안 |
| E303 | 팔 도달 시간 초과 | ALARM | 8s | 초안 |
| E401 | 작업 지점 도착 오차 초과 | WARN | 10cm / 0.15rad (TF 기준, 성공 1s 뒤) | 초안 |
| E402 | 위치 상실 | ALARM | AMCL 표준편차 0.35m / 0.45rad, 2.0s | 기준 ✅ 11/21, 등급 초안 |
| E403 | 경로 차단 | ALARM | 복구 3회 또는 15s 동안 0.2m 진척 없음 | 초안 |

그 밖의 초안: 팔 자세 각도(stowed 0/2.6, approach 0.8/1.2, work 1.29/0.93, retreat 0.5/1.8), 인터락 기준값, 시운전 파라미터(30/60/100%, 속도당 3회, settle 2s). 확정된 것: Nav2 robot_radius 0.20(11/22), 사이클 1회 = 지점 1곳 방문.

### 알아 둔 것 (막혔던 것 요약, 자세한 건 docs/DEVLOG.md)
- Nav2 /cmd_vel 타입은 Twist. 수업 launch는 cmd_vel 되먹임 고리(발행자 6개)가 있어서 cmd_vel_nav로 고침.
- Nav2 주행 노드는 `TimerAction(10초)` 뒤에 켜짐(global_costmap이 map TF 기다리다 0.5초 만에 실패하던 문제). 실패하면 Nav2를 껐다 켜야 함.
- map_saver_cli는 `-p save_map_timeout:=10.0` 필요. 지도 저장 후 my_robot_navigation 다시 빌드(CMake install은 빌드 시점 파일만 링크).
- rqt_robot_monitor는 /diagnostics_agg만 봄 → diagnostic_aggregator를 launch에 넣음 (묶음 Motion/Sensor/Interlock/Navigation).
- 도착 위치는 /amcl_pose가 아니라 TF로 읽는다 (/amcl_pose는 0.25m·0.2rad 움직여야 갱신).
- Gazebo 도구 막대 상자는 1kg 동적 물체라 로봇이 밂 → test_assets의 static 벽 사용.
- IMU 융합(robot_localization)은 구현 가능하지만 직진 헛돎(일정 속도로 헛돎)은 못 잡아서 보류. 회전 미끄러짐 보강용으로 나중에 검토.
- 확인용 스크립트를 여러 번 켜고 끈 뒤 Nav2가 실제 움직임을 못 보고 빙글빙글 돈 적 있음 → 전부 새로 켜니 해결(DEVLOG 9번).
- Nav2 노드 로그는 `~/.ros/log/<노드이름>_<pid>_*.log`. `ros2 topic hz`는 `--window 20`으로.
- 팔 PID: 어깨 P30/I2/D1/i_max0.5, 팔꿈치 P15/I1/D0.3/i_max0.3. 0 = 위로 곧게, +각도 = 앞으로. 한계 shoulder -1.0~1.57, elbow -0.5~2.7. E303 시험 자세 `test_beyond_limit`.

## 작업 규칙
- 원칙은 **한 단계씩**. 각 단계 끝에 ① 만든·바꾼 파일 설명 ② 확인 방법(터미널별 명령) ③ 이름 바꾸기 실험 1개. (건형이 "전부 해줘"처럼 묶어서 요청하면 그대로 하되 값은 초안으로 두고 마지막에 표로 확인받는다.)
- 판정 숫자, 알람 등급, 점검 순서, 자세 각도 같은 값은 **초안을 표로** 보여 주고 의미와 바꿨을 때 영향을 설명한 뒤 확인받는다. 최종 결정은 건형.
- Gazebo, rviz2, teleop, Nav2 등 GUI·상시 실행 명령은 Claude가 실행하지 말고 알려 준다. colcon build, pytest, git은 Claude가 해도 된다.
- **"커밋해"라고 하기 전에는 커밋 금지.** 원래는 건형이 자기 말로 설명한 뒤 커밋하지만, 건형이 바쁘다고 하면 생략 가능(11/21). 커밋 메시지는 한국어 `feat: ...`, `docs: ...`.
- 수업 패키지를 바꿀 때는 무엇을 왜 바꿨는지 `docs/DEVLOG.md`에 남긴다.
- DEVLOG의 막힌 문제는 **[증상 → 진단 도구·방법 → 조치 → 전후 수치]** 형식, **직접 잰 숫자만**.
- 새 알람 코드는 `config/alarm_codes.yaml`에 같은 형식(name, level, meaning, causes, checks)으로 추가하고 `test_alarm_codes.py`의 CODES에 해당 core 모듈을 넣는다.
- 단계마다 마무리: README "진행 상황" 체크박스와 알람 코드 표 갱신, DEVLOG 추가.
- **하지 말 것**: 파지·캘리브레이션 구현, 자체 경로 계획·회피 알고리즘, 블랙박스·로그 번들(3층에서 함).
