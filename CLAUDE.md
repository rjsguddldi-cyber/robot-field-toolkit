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

## 프로젝트 방향 (11/22 변경: PHM)
건형의 말: **"자율화된 환경에서는 작은 오차가 연쇄적인 운영 리스크로 증폭된다. 그래서 PHM 시스템을 도입한다."**
(PHM = Prognostics and Health Management, 고장이 나기 전에 건강 상태를 재서 미리 대응하는 정비 체계)

- **최대한 주입 없이**: 메시지를 조작하는 고장 주입 대신, 실제로 생기는 오차와 운용 조건 변화(바퀴 마모, 미끄러짐, 하중, 환경 변화)로 열화를 만든다. 기존 `fault_injector`, `scan_fault_injector`는 지우지 않고 **기존 알람 시험용으로만** 남긴다.
- 보여 줄 것:
  1. 작은 오차가 연쇄적으로 커지는 과정 측정: odom 잔차 → 위치 불확실성 → 작업 지점 도착 오차 → 팔 작업 위험 → 사이클 타임·정지
  2. PHM이 **첫 고리에서, 기존 알람(E105, E401 등)보다 먼저** 잡아낸다 (몇 사이클 앞서는지).
- **Gazebo 실제 위치(ground truth)는 감지 입력으로 절대 쓰지 않는다.** 감지 시점·정확도 채점용(별도 CSV 열)으로만 쓴다.
- **통계 기반만**(평균, 표준편차, 이동평균, 추세선). 머신러닝은 쓰지 않는다.
- README는 11/25에 PHM 목적으로 다시 쓴다. 그 전까지는 진행 상황 체크만.

### PHM 단계
1. **IMU 추가**: my_robot_description에 IMU 센서(수업 6장 LiDAR와 같은 방식), 브리지, Gazebo 센서 시스템 확인. 확인: `ros2 topic hz`로 IMU 주기, 제자리 회전 때 IMU 각속도 ≈ odom 각속도.
2. **건강지표** `core/health_indicators.py` + pytest (rclpy 없이, 가짜 값으로 테스트). 사이클 1회 데이터로: ① 회전 잔차(odom vs IMU 각속도 평균·RMS) ② 이동 잔차(odom 이동 거리 vs TF map→base_link 이동 거리) ③ 구동 추종률(실제/명령 /cmd_vel_checked) ④ AMCL 불확실성(공분산 평균·최대) ⑤ 작업 지점 도착 오차(TF) ⑥ 팔 자세 도달 시간 ⑦ LiDAR 유효 거리값 비율 ⑧ 통신·타이밍(주요 토픽 수신 간격 평균·지터, 헤더 시각 vs 수신 시각). **각 지표의 정의·단위·필요한 이유를 표로 먼저 보여 주고 확인받기.**
3. **정기 건강 점검 러너** (commissioning_runner 개조): 고정 경로 MoMa 사이클(A → 도착 확인 → 팔 approach/work/retreat/stowed → B …) 반복, A·B는 건형이 Publish Point로 찍음. 사이클마다 지표를 CSV 한 줄 + ground truth 위치 오차 열(채점용). 기준선 = 정상 조건 처음 K사이클 평균·표준편차, 기준선 재설정 명령(정비·재캘리브레이션 후). 건강도 0~100(지표별 기준선 이탈을 합친 점수, **계산식 초안 확인받기**). 예지 경보는 이동평균이 N회 연속 기준 초과일 때만. 확인: 정상 조건에서 건강도 안정, 오경보 없음.
4. **주입 없는 조건 변화 시나리오 + 자동 대응**: A 바퀴 마모(URDF 실제 바퀴 반지름을 실행마다 단계 축소, DiffDrive odom 계산은 원래 반지름) / B 미끄러짐(경로 중간 저마찰 바닥) / C 하중 증가(차체·팔 끝 질량) / D 환경 변화(지도에 없는 장애물). 각각 실행 명령 한 줄. **Gazebo에서 되는지 먼저 조사해서 보고, 1시간 안에 안 되는 시나리오는 빼고 주입으로 대체하지 않는다.** 자동 대응: 예지 경보 수준별 speed_override 제한, 도착 정확도 나쁘면 팔 정밀 작업 보류 + "재캘리브레이션 권고", 사이클 종료 시 정비 리포트(Markdown: 어떤 지표가, 언제부터, 얼마나, 권고 조치와 우선순위). 잔여 사이클 추정(최근 추세선으로 기존 알람 기준까지). 확인: 시나리오 A에서 PHM 경보가 기존 알람보다 먼저, 몇 사이클 앞서는지.

### PHM 추가 요구 (11/22 건형)
1. **하중 조건별 기준선** (3단계, 가장 중요): 무거운 걸 들어 이동 잔차·구동 추종률이 바뀌는 건 열화가 아니라 운용 조건 변화. 사이클마다 하중 상태(예: empty, loaded_1kg …)를 CSV 열로 기록, **기준선을 하중 상태별로 따로 학습하고 같은 하중끼리만 비교**. 기준선 없는 하중이면 경보 대신 "기준선 학습 중"으로 K사이클 학습. pytest로 "하중만 바뀌면 예지 경보 없음"과 "같은 하중에서 지표가 나빠지면 경보"를 둘 다 테스트, Gazebo로도 확인.
2. **하중 시나리오** (4단계): Gazebo에서 팔 끝(또는 차체)에 질량을 붙이는 방법 조사·제안(메시지 조작 주입 금지). 하중 상태를 러너·감시 노드가 토픽이나 파라미터로 공유. 질량 단계 초안을 표로 확인받기.
3. **하중별 자동 대응** (4단계): 적재 상태에서는 speed_override로 최고 속도 제한(빈 상태/적재 상태 속도 프로파일). 프로파일 초안 확인받기.
4. **팔 상태별 풋프린트** (4단계, 1시간 안에 안 되면 빼기): 팔이 stowed가 아니거나 큰 물건을 들면 Nav2 costmap에 큰 풋프린트, stowed면 원래대로. rviz2에서 외곽선이 바뀌는지 확인.
5. E301 설명 보강 (11/22 완료): 펼친 채 주행 → 무게중심이 지지면 가장자리로(넘어짐), 풋프린트 커짐(충돌).

사전 조사 (11/22, 설치본 확인):
- 풋프린트: Jazzy Costmap2DROS가 `footprint` 토픽(geometry_msgs/Polygon)을 구독함(`footprint_sub_`), 파라미터 동적 변경 콜백도 있음 → `/local_costmap/footprint`, `/global_costmap/footprint`에 다각형을 보내면 실행 중 변경 가능할 것으로 보임 (실행 확인 전)
- 질량: Gazebo `gz-sim-detachable-joint-system` 설치됨 → 짐 모델을 팔 끝에 붙였다 떼는 방식(attach/detach 토픽) 후보. 또는 xacro 인자로 짐 질량을 넣고 실행마다 바꾸는 방식(재시작 필요)

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
- `core/` (rclpy 금지, pytest): `common.py`(Judgement, Debounce, 각도 도구), `motion_check.py`(E101~E104), `slip_check.py`(E105), `scan_check.py`(E201·E202, occlude 계산), `interlock.py`(E301~E303, 속도 %), `nav_check.py`(E401~E403, pose_error), `arm_pose.py`(자세 추적), `commissioning.py`(계획·집계·리포트), `alarm_codes.py`(코드표 읽기), `health_indicators.py`(PHM 지표, wip)
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

## 현재 상태 (2026-11-22 기준, PHM 전환 전 시점)
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

### 다음 할 일 (11/22 PHM 방향으로 변경)
- 작업 브랜치: **wip** (PHM은 시운전 러너·nav_check 등 wip 코드 위에 만든다). main은 Gazebo 확인 끝난 것만.
- 위 "PHM 단계" 1 → 4 순서. 1단계 IMU 추가는 커밋 d15e4bc(wip, push됨).
- **2단계 건강지표 (11/22 작성, 미커밋)**: `core/health_indicators.py` + `test/test_health_indicators.py`(18개). 건형 확정(추천대로): ① 회전 잔차는 차이(평균·RMS)와 비율(최소제곱 기울기) 둘 다, 회전 중(|odom w|>0.1) 샘플만 / ② 이동 잔차는 시작·끝 직선 이동량 비교(누적 경로 길이 X) / 보정 전후 odom 비교는 4단계 여유 있으면. `CycleData`에 (시각, 값) 목록을 넣고 `compute_indicators()` → CSV 한 줄 dict. `INDICATORS` 표에 하중 영향 표시(②③⑥ → 3단계 하중별 기준선). ⑧ 타이밍 토픽: scan, odom, imu, joint_states. 결과 열(사이클 타임, 성공, 알람, 복구 횟수)과 채점 열(ground truth 오차)은 3단계 러너에서.
- 1단계 발견: 제자리 회전에서 IMU 1.192rad/s vs odom 1.000rad/s (유효 바퀴 간격 약 0.21m, 바퀴 안쪽 모서리 접촉 추정). **건형 결정: 보정하지 않음** (기준선에 흡수, 4단계 재캘리브레이션 사례로 사용). DEVLOG 11/22.
- Gazebo GUI OpenGL 오류는 `wsl --shutdown`으로 해결(DEVLOG 14번).
- 기존 2층 재시험(wall_partial 우회, E105, E403, occlude → E402)은 PHM 4단계 시나리오와 겹치므로 그때 함께 확인.
- 남은 초안 값 확정(아래 표)은 PHM 지표·건강도 확인 때 함께.

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
- **"커밋해"라고 하기 전에는 커밋 금지.** 원래는 건형이 자기 말로 설명한 뒤 커밋하지만, 건형이 바쁘다고 하면 생략 가능(11/21). 커밋 메시지는 한국어 `feat: ...`, `docs: ...`. **커밋하면 그날 push.**
- 수업 패키지를 바꿀 때는 무엇을 왜 바꿨는지 `docs/DEVLOG.md`에 남긴다.
- DEVLOG의 막힌 문제는 **[증상 → 진단 도구·방법 → 조치 → 전후 수치]** 형식, **직접 잰 숫자만**.
- 새 알람 코드는 `config/alarm_codes.yaml`에 같은 형식(name, level, meaning, causes, checks)으로 추가하고 `test_alarm_codes.py`의 CODES에 해당 core 모듈을 넣는다.
- 단계마다 마무리: DEVLOG 추가, README는 "진행 상황" 체크만 (11/25에 PHM 목적으로 다시 씀).
- PHM: ground truth는 채점용으로만, 통계 기반만(ML 금지), 고장 주입 대신 조건 변화로 열화.
- **하지 말 것**: 파지·캘리브레이션 구현, 자체 경로 계획·회피 알고리즘, 블랙박스·로그 번들(3층에서 함).
