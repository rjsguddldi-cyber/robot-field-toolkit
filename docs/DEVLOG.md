# 개발 일지

## 11/19
- `gz: command not found` → 설치는 돼 있었는데 ROS 환경을 불러오지 않음 → `source /opt/ros/jazzy/setup.bash`, `.bashrc`에 추가
- Gazebo RTF 60%, /scan 6.5Hz → `glxinfo`로 확인하니 llvmpipe(CPU 렌더링) → `/dev/dxg`로 WSL의 GPU 인식 확인 → `GALLIUM_DRIVER=d3d12` 설정 → /scan 9.2Hz
- 수업 작업공간을 그대로 복사했더니 build/install/log에 교육장 노트북 경로가 남아 있음 → 지우고 다시 빌드
- `gh auth login` 브라우저 자동 열기 실패, 요청 과다(slow_down) → 기다린 뒤 브라우저에서 직접 코드 입력

## 11/20
### 막힌 것
형식: 증상 → 진단 도구·방법 → 조치 → 전후 수치

**1. SLAM 지도 저장 실패**
- 증상: `map_saver_cli`가 `Failed to spin map subscription`으로 종료, 지도 파일이 안 생김
- 진단: `ros2 topic info /map -v` → 발행자 slam_toolbox 1개, Durability `TRANSIENT_LOCAL`로 정상. 지도는 있는데 받기 전에 끝남. 로그상 대기 시작부터 실패까지 약 2초(기본 timeout)
- 조치: `-p save_map_timeout:=10.0`으로 대기 시간 늘림 (WSL에서 노드끼리 찾는 디스커버리가 느린 것으로 추정)
- 전후: 2초 대기에서 실패 → 10초 대기에서 `simple_room.yaml/.pgm` 저장 성공

**2. Nav2 명령 되먹임 고리**
- 증상: 수업 launch 그대로면 /cmd_vel에 명령이 섞여 들어감 (E103 판정이 틀어질 구조)
- 진단: `ros2 topic info /cmd_vel -v` → 발행자 6개(controller_server 1, behavior_server 4, velocity_smoother 1), 구독자에 velocity_smoother 포함 → 스무더가 자기 출력을 다시 입력으로 받음
- 조치: 공식 nav2_bringup처럼 controller·behavior 출력과 스무더 입력을 `cmd_vel_nav`로 remap
- 전후: /cmd_vel 발행자 6개 → 1개(velocity_smoother), 구독자 2개 → 1개(fault_injector)

**3. Nav2 Goal 거부**
- 증상: rviz2 Nav2 Goal → `Goal was rejected by server`
- 진단: `ros2 lifecycle get`으로 노드 상태 확인 → 9개 중 4개만 active, planner_server부터 inactive. `~/.ros/log/planner_server_*.log`에 `Failed to activate global_costmap because transform from base_footprint to map did not become available before timeout`. 설정 대기 시간은 60초인데 0.5초 만에 실패, AMCL의 map TF는 그 직후 나옴. `manage_nodes` STARTUP 재요청은 이미 active인 controller_server 때문에 실패
- 조치: `lifecycle_manager_navigation` 시작을 `TimerAction`으로 10초 늦춤 (위치 추정이 먼저 자리 잡게)
- 전후: active 4/9 → 9/9, Nav2 Goal 주행 성공. stuck에서 정지, normal 복귀 후 주행 확인

**4. /wheel_cmd 주기가 11.8Hz로 보임**
- 증상: `ros2 topic hz /wheel_cmd`의 average rate 11.8Hz (Nav2 명령은 20Hz일 텐데)
- 진단: 같은 출력의 `max: 48.041s` → 목표 없이 쉬던 48초 공백이 평균에 섞임. `window` 값이 1초에 약 20개씩 증가
- 조치: `--window 20`으로 최근 값만 보기
- 전후: 전체 평균 11.8Hz → 주행 중 실제 약 20Hz

**5. rviz2 창이 화면보다 큼**
- 증상: rviz2 창이 화면을 넘거나 꽉 차서 쓰기 불편
- 진단: rviz 설정 파일 `Window Geometry` 확인 → mapping 2748x1662, navigation 1854x1011 (내 화면 1920x1080)
- 조치: 두 파일 창 크기를 1280x720으로 줄임
- 전후: 화면보다 큰 창 → 1280x720

### 수업 패키지 변경 (무엇을, 왜)
- `my_robot_slam`, `my_robot_navigation`을 `~/course_pkgs`에서 저장소(src)로 옮김. Nav2 주행 감시(E4xx)와 시운전에 필요.
- `my_robot_navigation/launch/navigation.launch.py`
  - `map` 인자 기본값: 빈 문자열 → 패키지 경로의 `maps/simple_room.yaml`. 지도를 저장소 안에 두고 절대경로 없이 실행되게.
  - cmd_vel remap: 위 막힌 것 2번.
  - `lifecycle_manager_navigation` 10초 지연: 위 막힌 것 3번.
- `my_robot_navigation/CMakeLists.txt`: 설치 폴더에 `maps` 추가 (지도 기본 경로가 설치 폴더를 가리키므로).
- `my_robot_slam/rviz/mapping.rviz`, `my_robot_navigation/rviz/navigation.rviz`: 창 크기 축소 (위 막힌 것 5번).

**6. rqt_robot_monitor에 아무것도 안 보임**
- 증상: motion_monitor가 /diagnostics를 내는데 rqt_robot_monitor 화면이 비어 있음
- 진단: 설치된 rqt_robot_monitor 코드(`robot_monitor_plugin.py`)를 보니 `/diagnostics`가 아니라 `/diagnostics_agg`를 구독. `/diagnostics_agg`를 내는 diagnostic_aggregator가 없었음
- 조치: `ros-jazzy-diagnostic-aggregator` 설치, toolkit_base.launch.py에 aggregator 노드 추가 (`field_toolkit/motion`으로 시작하는 항목을 Motion 묶음으로)
- 전후: rqt_robot_monitor 빈 화면 → All devices에 `Motion OK` 표시

**7. rqt_robot_monitor 화면 반응이 매우 느림 (보류)**
- 증상: rqt_robot_monitor 클릭·스크롤 반응이 느림. Gazebo 창 최소화해도 그대로
- 진단: `top -o %CPU -c` → Gazebo 서버 130%, GUI 70~90%, CPU 전체 52~59% 여유. rqt_robot_monitor CPU 1%. `/proc/<pid>/maps`로 Gazebo·rqt 모두 d3d12(GPU) 사용 확인 → CPU·GPU 부족이 아니라 WSLg 화면 전달 쪽으로 추정
- 조치: rqt는 쓰지 않고 터미널(launch 출력, `ros2 topic echo /field_toolkit/alarms`)로 알람 확인. diagnostic_aggregator는 남겨 둠(/diagnostics_agg 표준 형식)
- 전후: 해결 안 함. 마감 우선으로 보류

### 1층 동작 확인
- Gazebo + teleop에서 fault_injector mode별로 E101(stuck), E102(weak), E103(drop), E104(drift)가 뜨고 normal로 돌리면 해제되는 것 확인 (11/21)

## 11/21
### 수업 패키지 변경 (무엇을, 왜)
- `my_robot_description/urdf/my_robot.urdf.xacro`: 2축 팔 추가
  - 차체 뒤쪽(x=-0.06m)에 팔 받침, 어깨·팔꿈치 pitch 관절(revolute, 한계·관성 포함), 끝에 툴 자리 표시용 상자
  - Gazebo `JointPositionController` 2개 (gz 토픽 `arm/shoulder_cmd`, `arm/elbow_cmd`), 시작 자세는 접힘
  - 기존 `JointStatePublisher` 플러그인에 팔 관절 2개를 추가 (바퀴 관절 상태와 같은 /joint_states로 나감, 충돌 없음)
- `my_robot_gazebo/params/my_robot_bridge.yaml`
  - 팔 관절 명령 2개 추가 (Float64 → gz.msgs.Double, ROS_TO_GZ)
  - LiDAR: ROS 쪽 이름을 `scan` → `scan_raw`로 변경. scan_fault_injector가 `/scan`으로 다시 냄 (LiDAR 고장 주입 E2xx용). **이제 SLAM·Nav2를 쓰려면 toolkit_base.launch.py를 꼭 켜야 /scan이 나옴**

### 팔과 LiDAR 스캔 평면 (설계 근거, 실측 전)
- LiDAR 스캔 평면 높이: base_link 기준 z=0.05m (차체 윗면 0.03m보다 2cm 위). 차체 위 구조물은 이 평면을 지나갈 수밖에 없음
- 그래서 팔 받침을 LiDAR(x=0.05)에서 0.11m 뒤에 둠. gpu_lidar 최소 거리 0.15m 안쪽이라 스캔에 안 잡힐 것으로 예상
- 접힘(stowed) 자세의 위팔·아래팔은 계산상 스캔 평면보다 위(z 0.09m 이상)
- 펼친 자세(work)는 툴이 차체 앞 x≈0.18m, z≈0.06m 근처까지 내려옴 → LiDAR에서 약 0.13m라 역시 최소 거리 안쪽일 것으로 예상
- 확인: 아래 "팔 Gazebo 확인"의 LiDAR 가림 측정 참고

### 팔 Gazebo 확인 (11/21, 직접 잰 값)
- 시작 시 stowed 도달, 오차 0.0rad. /joint_states에 shoulder·elbow·바퀴 2개 함께 나옴
- 자세별 도달 시간(시뮬레이션 시간): approach 6.7s, work 3.5s, retreat 1.0s, stowed 3.1s
- **LiDAR 가림: stowed·approach·work·retreat 모두 /scan_raw 유효 360/360, stowed 대비 바뀐 빔 0개** → 이 팔 배치에서는 펼친 자세도 스캔을 가리지 않음 (팔이 LiDAR 최소 거리 0.15m 안쪽에 있음)
- E303 시험(test_beyond_limit, 어깨 목표 2.0 > 한계 1.57): 어깨가 한계에서 멈춤(오차 0.43rad), 8.1s에 E303 raised → 의도대로

**8. 팔이 stowed로 돌아와도 도달 판정이 안 됨 (E303 오알람)**
- 증상: test_beyond_limit 뒤 stowed 요청 → 15.6s 지나도 오차 0.061rad(허용 0.05)에서 멈춤, E303 raised
- 진단: 한계에서 버티는 동안 PID 적분 항이 크게 쌓임(i_max 2.0Nm), i_gain 1.0으로는 풀리는 데 오래 걸림. approach 6.7s도 P 게인(10)이 작아 중력 처짐을 적분에 기대서 느린 것으로 추정
- 조치: 어깨 P 10→30, D 0.5→1.0, I 1.0→2.0, i_max 2.0→0.5 / 팔꿈치 P 5→15, D 0.15→0.3, I 0.5→1.0, i_max 1.0→0.3
- 전후: 한계 시험 후 stowed 복귀 15.6s 미도달(오차 0.061) → 1.7s 도달(오차 0.025), E303 해제 후 재발 없음. 자세별 도달 approach 6.7→1.7s, work 3.5→0.6s, retreat 1.0→1.0s, stowed 3.1→0.9s. E303 시험은 그대로 8.2s에 raised

### 인터락 Gazebo 확인 (11/21, 직접 잰 값)
- 팔 work 자세에서 /cmd_vel 0.2m/s → /cmd_vel_checked 0.0, 실제 v 0.000, E301 raised, **E101 안 뜸** (motion_monitor 기준을 /cmd_vel_checked로 바꾼 효과)
- speed_override 30% → /cmd_vel 0.2 → /cmd_vel_checked 0.060, 실제 v 0.060
- 30%로 주행 중 approach 요청 → 거부(팔 stowed 유지), E302 raised → 3초 뒤 cleared
- speed_override 150 설정 시도 → 거부됨 (0~100만 허용)

### LiDAR 고장 Gazebo 확인 (11/21, 직접 잰 값)
- occlude(정면 180도) → 유효 50%, 약 2초 뒤 E201 raised → normal 복귀 후 cleared (유효 100%)
- dropout(1Hz) → E202 raised (5.0Hz, 마지막 수신 1.1s 전) → normal 복귀 후 cleared (9.5Hz, 시뮬레이션 시간 기준)

**9. Nav2 목표로 가지 않고 제자리에서 빙글빙글 돎**
- 증상: Claude가 확인용 스크립트·명령을 여러 번 켜고 끈 뒤 Nav2 목표를 주자, Gazebo의 로봇은 계속 회전하는데 Nav2 피드백의 남은 거리는 1.4m 그대로
- 진단: /cmd_vel_nav → /cmd_vel → /wheel_cmd까지 명령은 나감(0.26m/s). Claude 셸에서는 /odom·/clock이 잘 안 잡혔음(WSL 디스커버리 지연). Nav2가 실제 움직임을 못 보는 상태로 추정, 원인은 확정 못 함
- 조치: Gazebo → toolkit_base → Nav2 → rviz2 순서로 전부 새로 켬
- 전후: 회전만 반복 → Nav2 Goal 정상 도착 (재발하면 rviz2 로봇이 Gazebo와 같이 움직이는지, /odom·/clock 주기부터 확인)

### 주행 감시 Gazebo 확인 1차 (11/21, 건형이 직접 잰 값)
- 시험 A (정상 도착): E401 detail `거리 25.0cm, 각도 11.8도` → raised
- 시험 B (통로 일부 막기): Gazebo 도구 막대 상자는 1kg 동적 물체라 로봇이 밀고 지나가 목표 도착
- 시험 C (occlude 300도, 유효 17%): 508.7s E201 → 509.2s E402. E403(15s 진척 없음)은 4번 raised, 도착하면 cleared. 도착 오차 14.8~46.3cm

**10. E401 도착 오차가 너무 크게 나옴 (25~46cm)**
- 증상: Nav2는 성공(goal checker 허용 25cm)인데 E401 오차가 25.0, 46.3, 35.8cm
- 진단: 도착 위치를 /amcl_pose에서 읽었음. AMCL은 로봇이 0.25m(update_min_d) 또는 0.2rad(update_min_a) 움직여야 /amcl_pose를 새로 냄 → 멈춘 순간보다 최대 25cm 전 위치일 수 있음. 첫 값이 정확히 25.0cm인 것과 맞음
- 조치: nav_monitor와 시운전 러너가 도착 위치를 TF(map→base_footprint, AMCL 보정 + 바퀴 주행 거리)로 읽게 수정. /amcl_pose는 공분산(E402)에만 사용
- 전후: 25.0 / 46.3 / 35.8cm → 재측정 15.8cm, 8.5도 (Nav2 허용 25cm 안, 팔 기준 10cm 초과라 E401 raised)

**11. E402(위치 상실)가 LiDAR를 가리지 않아도 뜸**
- 증상: E201 없이 409.2s, 496.2s, 1080.0s에 E402 raised
- 진단: raised 때 표준편차 0.27~0.30m / 0.27~0.29rad, cleared 때 0.21~0.24m / 0.17~0.22rad → 이 환경의 평상시 AMCL 표준편차가 기준(0.25m, 0.35rad) 근처
- 조치: 기준을 0.35m / 0.45rad로 올림 (건형 결정). 300도 가림 때도 0.28~0.29m라 E402가 안 뜰 수 있음 → 그 경우 "AMCL 공분산만으로는 가림을 못 잡고 E201이 먼저 잡는다"로 기록
- 전후: (재시험 후 기록)

**12. 상자를 놓아도 로봇이 밀고 지나감**
- 조치: 고정(static) 벽 모델 `test_assets/wall_partial.sdf`(3m), `wall_full.sdf`(10m) 추가, `ros_gz_sim create`로 배치

**13. 벽에 막혔는데 Nav2가 '도착 성공' (바퀴 헛돎)**
- 증상: 고정 벽(wall_partial) 너머로 Nav2 Goal → 로봇이 벽 끝에 걸려 멈춤. 그런데 Nav2는 성공으로 끝나고 E401 `거리 23.9cm, 각도 14.0도` raised. E101·E403 안 뜸
- 진단: Gazebo에서는 로봇이 벽 끝에 붙어 있는데 rviz2의 로봇은 방 가운데로 가 있음. rviz2 스캔 점이 지도 벽과 안 맞고, AMCL 파티클이 여러 곳으로 갈라짐 → 바퀴가 헛도는 동안 바퀴 기반 odom이 계속 전진해서 TF 위치가 벽을 뚫고 감 → Nav2가 목표 25cm 안이라고 판단. E101은 명령과 odom(바퀴)을 비교해서 헛돎을 못 잡고, E401도 TF(odom 포함)라 못 잡음
- 의미: 실제 MoMa에서 "도착했다고 믿고 팔 작업 시작"으로 이어질 수 있는 상황. 바퀴 정보만 믿는 감시는 헛돎을 못 잡음
- 조치 (11/22 건형 결정):
  - Nav2 `robot_radius` 0.18 → 0.20 (URDF상 차체 모서리까지 0.180m라 여유가 0이었음). 수업 패키지 nav2_params.yaml 2곳
  - E105 바퀴 헛돎 추가: 바퀴 odom 이동량과 LiDAR 스캔 변화(빔별 거리 변화의 중앙값)를 비교. 위치 재설정(/initialpose)까지 유지, 인터락이 팔 작업 금지, 시운전 러너는 사이클 실패 처리
  - IMU 융합 검토: Gazebo IMU 시스템·robot_localization 모두 있어 구현은 가능. 하지만 EKF는 직진 속도를 바퀴에서 받아서 '벽에 막혀 일정 속도로 헛도는' 경우를 못 잡음(가속도 0으로 같음). 주로 회전 미끄러짐에 효과. TF 주인을 DiffDrive에서 EKF로 바꾸는 큰 수정이라 마감 전에는 보류
- 전후: (재시험 후 기록)

## 11/22 (PHM 방향 전환)
### 수업 패키지 변경 (무엇을, 왜)
- `my_robot_description/urdf/my_robot.urdf.xacro`: IMU 추가 (`imu_link` 차체 중심 + `<gazebo reference="imu_link">` IMU 센서, 100Hz, 잡음 각속도 0.002rad/s·가속도 0.02m/s² 초안). 이유: PHM 회전 잔차(odom 각속도 vs IMU 각속도)를 재려면 바퀴와 독립된 회전 측정이 필요
- `my_robot_gazebo/params/my_robot_bridge.yaml`: `imu` 항목 추가 (gz.msgs.IMU → sensor_msgs/msg/Imu, GZ_TO_ROS)
- 월드는 그대로: `simple_room.world`에 `gz-sim-imu-system`이 이미 있음

### IMU Gazebo 확인 (11/22, 건형이 직접 잰 값)
- /imu 주기(실제 시간 기준) 59.6 / 65.7 / 85.2Hz, Gazebo RTF와 비슷하게 변함 → 시뮬레이션 시간 기준 100Hz
- teleop 제자리 회전(명령 1.0rad/s): IMU 각속도 1.1927 / 1.1922 / 1.1913rad/s, odom 각속도 1.0000rad/s → **IMU가 약 19% 더 빠름**
- 계산: 0.25m × 1.000 ÷ 1.192 ≈ 0.21m = 두 바퀴 안쪽 모서리 사이 거리(0.25 − 바퀴 폭 0.04). 원기둥 바퀴의 접촉점이 안쪽 모서리에 잡히는 것으로 추정 (확정 아님)
- 결정(건형): 보정하지 않고 그대로 둠. 일정한 오차는 PHM 기준선에 흡수되고, 나중에 "재캘리브레이션 권고 → 보정 → 기준선 재설정" 사례로 씀

**14. Gazebo GUI가 OpenGL 컨텍스트를 못 만듦**
- 증상: `gazebo.launch.py` 실행 시 `[QT] Failed to create OpenGL context for format QSurfaceFormat(version 2.0 ...)`로 Gazebo 꺼짐 (IMU 추가 직후였지만 GUI 문제라 무관)
- 진단: `GALLIUM_DRIVER=d3d12 glxinfo -B` → `D3D12 (NVIDIA GeForce GTX 1660 Ti)`로 GPU는 정상. WSL이 5시간 넘게 켜져 있었음 → WSLg 화면 기능이 꼬인 것으로 추정
- 조치: PowerShell에서 `wsl --shutdown` 후 터미널 다시 열기 (새 셸이라 .bashrc는 자동 적용)
- 전후: Gazebo GUI 실행 실패 → 정상 실행

### PHM 2단계 건강지표 정의 (11/22 건형 확정)
- 사이클 1회 = 이동 시작 → 도착 확인 → 팔 approach/work/retreat/stowed 끝
- ① 회전 잔차: IMU − odom 각속도의 평균·RMS + 비율(최소제곱 기울기), 회전 중 샘플만 (정상값 비율 약 1.19, 1단계 실측)
- ② 이동 잔차: 시작·끝 직선 이동량 (odom − TF)/TF %. 누적 경로 길이는 AMCL 보정 때마다 부풀어서 쓰지 않음
- ③ 구동 추종률: odom/명령 선속도 (명령 유지 가정). 바퀴 기반이라 바퀴 마모는 안 보임
- ④ AMCL 표준편차 평균·최대 ⑤ 도착 오차(TF) ⑥ 팔 도달 시간 ⑦ LiDAR 유효 비율 ⑧ 토픽 수신 간격·지터·헤더 지연
- 하중 영향 지표: ②③⑥ → 3단계에서 하중 상태별 기준선으로 비교

## 11/23
### 수업 패키지 변경 (무엇을, 왜)
- `my_robot_description/urdf/my_robot.urdf.xacro`: `gz-sim-odometry-publisher-system` 추가 (`ground_truth/odom`, 20Hz, 2D). 이유: PHM 채점용 실제 위치. 감지 입력으로는 쓰지 않음. `tf_topic`을 `ground_truth/tf_unused`로 따로 둬서 로봇 TF와 섞이지 않게 함
- `my_robot_gazebo/params/my_robot_bridge.yaml`: `ground_truth/odom` 항목 추가 (gz.msgs.Odometry → nav_msgs/msg/Odometry, GZ_TO_ROS)
- 가정: 지도(map) 좌표 = Gazebo 월드 좌표 (SLAM을 월드 원점·방향 0에서 시작). Gazebo 첫 실행 때 TF와 ground truth 차이로 확인 예정

### PHM 3단계 정기 건강 점검 (11/23 건형 확정: 초안대로)
- 러너: 새 노드 `health_runner` (시운전 러너는 그대로). A ↔ B 사이클 반복, 사이클마다 CSV 한 줄 `~/field_toolkit_reports/phm_<시각>.csv`
- 기준선: 하중 상태(`/field_toolkit/load_state`)별 평균·표준편차, 성공 사이클 K=8개. 처음 보는 하중은 "기준선 학습 중 (k/K)"로 경보 없음. `phm_baseline.json`에 저장. 재설정 `/health_runner/reset_baseline` (std_srvs/Trigger, 현재 하중만)
- 건강도: d = (값 − 평균)/max(표준편차, 최소 표준편차), 나쁜 방향만 → 지표 점수 100 − 20·max(0, d−1) → (점수 평균 + 최저 점수)/2
- 예지 경보: 최근 M=5사이클 이동평균의 d가 2(주의)·3(경고)을 N=3사이클 연속 넘을 때
- pytest(가짜 데이터): 하중만 바꿔도 경보 없음 / 같은 하중에서 이동 잔차가 사이클당 0.4%씩 커지면 10번째(4.0%)에 주의, 12번째(4.8%)에 경고 / 정상 200사이클 × 5시드 경보 0회
- Gazebo 확인 (11/23, `phm_20261123_012512.csv`, 24사이클, 15~17번 조치 후):
  - 성공 24/24, Nav2 중단 0회, E401 0회
  - 1~8 empty 학습 → 9~16 판정: 8사이클 모두 [정상], 건강도 83.3~100, 예지 경보 0회
  - 16번 건강도 83.3: AMCL 위치 표준편차 0.301m (기준선 0.251 ± 0.016 → 바닥값 0.02로 d = 2.5, 지표 점수 70). 한 사이클만 튀어서 점수에는 보이고 경보는 안 뜸 (설계대로)
  - 16번 사이클 중 하중 상태를 `loaded_2kg`으로 변경 → 17~24 "기준선 학습 중 (1/8)…(8/8)", 경보 0회. 기준선 파일에 empty·loaded_2kg 각각 8개 따로 저장
  - 한계: 이번엔 이름표만 바꿨고 실제 질량은 그대로라 loaded 지표가 empty와 거의 같음(이동 잔차 2.59 vs 2.70%, 팔 시간 4.36 vs 4.30s). "하중별 분리 로직"만 확인. 실제 하중 효과는 4단계. 판정 사이클도 8개뿐이라 오경보 0의 근거는 pytest(정상 200사이클 × 5시드)가 주, Gazebo는 보조
  - 기준선 관찰: empty 기준선 10개 지표 중 8개가 측정 표준편차보다 최소 표준편차(바닥값)가 큼. 예: 도착 거리 오차 평균 4.95cm, 표준편차 0.12cm(로봇이 매번 5cm 기준 원 가장자리에 섬) → 바닥값 2cm가 없었다면 0.4cm 차이만으로 d = 3(경고). 시뮬레이션이 너무 조용해서 바닥값이 필요하다는 초안 근거가 실측으로 확인됨
  - 채점 열: 실제 도착 오차 0.95~11.8cm(empty, 평균 5.5cm), 믿는 위치 ↔ 실제 위치 0.87~14.9cm(평균 7.9cm)
  - 기존 알람: E403 24/24 사이클(판정 버그, 18번), E402 7/24 사이클(AMCL 표준편차 0.35m 초과 순간)
  - 표시 수정: 모든 지표가 100점일 때 '가장 나쁜 지표'에 동점 중 첫 번째(rot_ratio)가 찍히던 것을 '-'로 (pytest 1개 추가)
- 작업 지점 (건형 rviz2 Publish Point): A (4.02, −3.69, yaw −0.69), B (−3.90, 2.79, yaw 2.46). 방 대각선 약 10.2m, 도착 방향 = 진행 방향

**15. 매 사이클 E401로 실패해서 기준선을 못 배움 (Nav2 도착 기준 ↔ E401 기준 모순)**
- 증상: health_runner 1차 실행(24사이클 예정) 5사이클 모두 실패, "기준선 학습 중 (0/8)"에서 멈춤. 4번은 "E401 도착 오차 초과로 팔 작업 생략", 1번은 Nav2 이동 실패(status 6)
- 진단: CSV(`phm_20261123_005349.csv`)의 TF 도착 오차가 24.9 / 26.4 / 27.7 / 24.5cm로 매번 25cm 근처. `nav2_params.yaml`의 `general_goal_checker.xy_goal_tolerance` 0.25m와 일치 → Nav2는 25cm 원 가장자리에 닿자마자 "성공"하고 멈추는데, E401은 10cm 초과를 실패로 봄. 로봇이 나빠진 것이 아니라 설정끼리 모순. 11/21 짧은 거리 시험(15.8cm)에서는 천천히 다가가 원 안쪽까지 들어가서 드러나지 않았던 것으로 추정
- 조치 (11/23 건형 확정, 수업 패키지 `my_robot_navigation/params/nav2_params.yaml`):
  - `general_goal_checker.xy_goal_tolerance` 0.25 → 0.08m (E401 0.10m보다 안쪽)
  - `general_goal_checker.yaw_goal_tolerance` 0.25 → 0.12rad (E401 0.15rad보다 안쪽)
  - `FollowPath.xy_goal_tolerance` 0.25 → 0.08m. DWB는 이 거리 안에서 제자리 회전만 하므로, goal checker만 줄이면 25cm에서 회전만 하다 진척 없음으로 실패함 → 둘을 같이 바꿈
- 함께 추가: 채점 열 `gt_loc_err_cm` = 도착 순간 TF 위치와 실제 위치(ground truth)의 거리. 1차 실행에서 TF 도착 오차와 실제 도착 오차가 5~11cm 달랐는데(예: 24.9 vs 13.3cm), map↔월드 어긋남인지 AMCL 위치 추정 오차인지 기존 열로는 구분할 수 없었음
- 전후: TF 도착 오차 24.5~27.7cm, 성공 0/5 → 16번 조치까지 한 뒤 4.57~6.19cm, 성공 24/24 (`phm_20261123_012512.csv`)
- 같이 본 정상 지표 (1차, 5사이클): 회전 비율 1.135~1.161, 구동 추종률 0.999~1.001, LiDAR 유효 99.9~100%, /scan 간격 99.7~100.0ms·지터 10.1~12.9ms, AMCL 위치 표준편차 평균 0.24~0.25m, 이동 잔차 2.7~7.9%(흔들림 큼, 기준선 확보 후 재확인)
- 미확인 2건 (재실행 후 다시 볼 것):
  - 3번 사이클 이동 실패(status 6, 14.8s): 로그에 planner "Failed to create plan ... from (1.85, 2.03)"(box1 바로 옆)이 있으나 이 실행의 로그인지 확실하지 않음
  - E403(진척 없음)이 1, 2, 5번 사이클에서 발생: 긴 경로 마지막 접근 구간의 오경보일 가능성

**16. 도착 기준을 줄여도 E401: E401이 실제 도착 정확도보다 AMCL 위치 추정 흔들림을 잼**
- 증상: 15번 조치(도착 기준 0.08m) 후 재실행(`phm_20261123_010904.csv`) 1사이클째 Nav2는 성공(`Reached the goal!`)했지만 TF 도착 오차 10.15cm로 E401(기준 10cm) → 팔 작업 생략, 학습 0/8
- 진단 도구·방법: 15번에서 추가한 채점 열 `gt_loc_err_cm`(도착 순간 TF 위치와 Gazebo 실제 위치의 거리)으로 "로봇이 믿는 위치"와 "실제 위치"를 분리
  | 값 | 1차 (기준 0.25m) | 재실행 (기준 0.08m) |
  |---|---|---|
  | TF 도착 오차 (믿는 위치) | 24.5~27.7cm | 10.15cm |
  | 실제 도착 오차 (채점용) | 13.3~23.3cm | 4.6cm |
  | 믿는 위치 ↔ 실제 위치 `gt_loc_err_cm` | (열 없음) | 13.4cm |
  | AMCL 위치 표준편차 평균 | 0.24~0.25m | 0.25m |
- 해석:
  - 로봇은 실제로 목표에서 4.6cm에 정확히 섰는데, AMCL이 위치를 13.4cm 틀리게 믿어서 E401이 "10cm 초과"로 판정. E401(TF 기준 10cm)이 AMCL 흔들림(표준편차 약 25cm)보다 좁아서, 실제 도착 정확도보다 위치 추정 흔들림을 더 많이 재고 있음
  - PHM 연쇄의 둘째 고리(odom 잔차 → **위치 불확실성 → 도착 판정** → 팔 작업 위험)가 실제로 나타난 사례. 감지는 ground truth 없이 해야 하므로, PHM에서는 ④ AMCL 불확실성 지표가 이 고리를 맡음
  - 8cm에서 멈춘 뒤 10.15cm가 된 2cm는 감속 중 밀림인지 AMCL 보정인지 아직 구분 못 함
- 5사이클까지 분포 (같은 CSV):
  | 사이클 | 결과 | TF 도착 오차 | 실제 도착 오차 | `gt_loc_err_cm` |
  |---|---|---|---|---|
  | 1 A | E401 | 10.15cm | 4.6cm | 13.4cm |
  | 2 B | 성공 | 7.88cm | 8.1cm | 3.1cm |
  | 3 A | 이동 실패 (17번) | - | - | - |
  | 4 B | 성공 | 7.80cm | 4.5cm | 3.9cm |
  | 5 A | 성공 | 7.85cm | 5.0cm | 12.7cm |
  - 성공한 사이클도 TF 오차가 7.80~7.88cm로 도착 기준 8cm 원의 가장자리에서 멈춤 → E401(10cm)까지 여유 2cm뿐. 멈춘 뒤 2cm만 밀려도 1번처럼 실패
  - 믿는 위치와 실제 위치 차이는 3.1~13.4cm로 들쭉날쭉 (AMCL 흔들림)
- 조치 (11/23 건형 확정): A안. `general_goal_checker.xy_goal_tolerance`와 `FollowPath.xy_goal_tolerance` 0.08 → 0.05m (E401까지 여유 5cm). B(E401 완화)는 팔 작업 안전 기준을 약하게 해서, C(AMCL 튜닝)는 마감 전 시간 위험으로 보류. 도착 오차의 정상값이 바뀌므로 기준선(`phm_baseline.json`)을 지우고 다시 학습
- 전후 (24사이클): TF 도착 오차 7.80~10.15cm(성공 3/4) → 4.57~6.19cm(성공 24/24, E401 0회), 실제 도착 오차 0.95~11.8cm, 사이클 타임 51.2~54.6s(B↔A, 이전 51.2~52.8s와 비슷, 마지막 접근이 느려지지 않음). 믿는 위치 ↔ 실제 위치 0.87~14.9cm로 여전히 들쭉날쭉 (AMCL 흔들림은 그대로)
- 해석 (한계): 늘어난 여유 5cm는 **믿는 위치(TF) 기준**이다. 실제 도착 오차는 4.5~8.1cm → 2.7~8.0cm로 거의 그대로이고, 믿는 위치 ↔ 실제 위치 차이(2.2~12.8cm)도 그대로. 이번 조치는 Nav2 ↔ E401 설정 모순을 없앤 것이지 로봇을 실제로 더 정확하게 만든 것이 아니다. 실제 정확도는 AMCL 흔들림 아래로 내려가지 않음
  - 현장이라면: ground truth가 없으므로 E401 통과 ≠ 정확한 도착. 측위가 흔들리는 날엔 E401을 통과해도 팔이 빗나갈 수 있음
  - 기준을 정하는 순서는 반대여야 함: 팔이 감당할 수 있는 위치 오차(작업 범위, 비전 보정 범위) → 측위 오차가 그 안에 드는지 → Nav2 도착 기준은 그 안쪽. 여기서는 E401 10cm를 먼저 두고 Nav2를 맞춤 (시뮬레이션이라 가능한 순서)
  - 현장 일반 방식은 2단계 접근: 주행 스택으로는 수 cm~수십 cm까지, 마지막 정밀 위치는 마커·반사판·비전이나 팔 카메라 보정. 이 프로젝트 범위 밖이라 구현하지 않음
  - 대가: 도착 기준을 좁히면 실제 로봇은 마지막 수 cm에서 왔다 갔다 하기 쉬움(사이클 타임, 구동부 마모). 시뮬레이션에서는 사이클 타임 변화 없음, 현장에서는 재확인 필요
  - PHM 결론: E401 통과만으로 실제 정확도를 보장할 수 없으므로 ④ AMCL 불확실성을 따로 추세로 보고, 평소보다 흔들리면 E401이 통과해도 "정밀 작업 보류, 재캘리브레이션 권고"(4단계 자동 대응 근거)
- 관찰: E403(경로 차단)이 세 번의 실행 모두 거의 매 사이클 발생하고, 도착 직전에 해제됨. 처음엔 마지막 접근 때문으로 추정했으나 틀림 → 18번

**17. 주행 도중 Nav2가 목표를 중단 (status 6): BT가 follow_path 응답을 20ms 안에 못 받음**
- 증상: health_runner 1차 실행 3번, 재실행 3번 사이클이 "이동 실패(status 6 = ABORTED)". 복구 동작 0회
- 진단: `~/.ros/log/bt_navigator_*.log`에서 중단 직전 `Timed out while waiting for action server to acknowledge goal request for follow_path` → `Aborting handle` → `Goal failed`. 같은 시각 controller_server 로그는 1초마다 `Passing new path to controller`(경로 재계획 때마다 BT가 follow_path에 목표를 다시 보냄)로 정상 동작 중. `nav2_params.yaml`의 `bt_navigator.default_server_timeout`이 20ms → WSL에서 DDS 응답이 늦을 때 한 번만 넘어도 주행 전체가 중단됨
- 조치 (11/23 건형 확정): `bt_navigator.default_server_timeout` 20 → 100ms
- 전후: 재실행 5사이클 중 1회 중단 → 24사이클 중 0회
- 참고: 1차 실행 3번 사이클의 원인은 로그 시각을 맞춰 보지 않아 미확인 (같은 status 6이라 같은 원인으로 추정만 함)

**18. E403(경로 차단)이 정상 주행에서 매 사이클 발생: 진척 기준점이 '남은 거리 0'에 고정되는 판정 버그**
- 증상: `phm_20261123_012512.csv`의 alarms 열에 E403이 24/24 사이클. 성공 24/24, 복구 0회인데도 발생하고 도착(성공) 때 해제
- 진단 도구·방법: nav_monitor 로그(`~/.ros/log/python3_4722_*.log`)의 E403 발생 시각과 health_runner 로그의 사이클 시작 시각을 대조. 실제 시간을 사이클별 RTF(시뮬레이션 사이클 타임 ÷ 실제 시간, 0.57~0.67)로 환산
  - 24개 모두 출발 후 시뮬레이션 시간 **14.4~16.1초**에 발생 = E403 진척 창 15초와 일치
  - 그 시각 로봇은 0.26m/s로 주행 중(15초면 약 4m) → 마지막 접근이 아니라 주행 초반. "기준점이 한 번도 갱신되지 않는다"는 뜻
  - 원인(유력 가설): Nav2가 경로를 계산하기 전 남은 거리 0 피드백을 먼저 보냄 → `core/nav_check.py`가 기준점을 (t, 0m)로 잡음 → 실제 10.2m가 와도 0 − 10.2 < 0.2라 '줄지 않음' → 15초 뒤 E403. 피드백 첫 값을 직접 본 것은 아님
  - 11/21에 못 본 이유: 벽 시험은 "막히면 뜨는지"만 확인했고 "정상 주행에서 안 뜨는지"는 확인하지 않음
- 조치: `update_feedback`에서 남은 거리 0 이하 피드백은 진척 기준점으로 쓰지 않음. "남은 거리가 늘면 기준점 재설정" 방식은 진짜로 막혀 경로가 계속 길어질 때 E403이 영원히 안 뜰 수 있어서 쓰지 않음
  - pytest 2개 추가: 0 피드백 뒤 10.2m에서 줄어들면 E403 없음 (**수정 전 코드에서 실패, 수정 후 통과**로 버그 재현 확인) / 0 피드백 뒤 실제로 막히면 E403 발생
- 전후: 정상 주행 E403 24/24 사이클 → 0/4 사이클 (11/23 `wear_A.csv` 마모 0% 4사이클, 21번)
- 교훈: 매 사이클 뜨는 알람은 운영자가 무시하게 되고, 진짜 경로 차단도 묻힌다. 알람 시험은 "뜨는지"와 "정상에서 안 뜨는지"를 둘 다 해야 함

### PHM 3단계 설계 결정: 초안에서 달라진 점 (11/23)
Gazebo에서 실패해서 고친 것이 아니라, 코드를 쓰면서 숫자를 손으로 계산하다 발견한 빈칸이다. 테스트로 막아 둠.

1. **이동평균은 창(M=5)이 다 찬 뒤에만 경보 판정**
   - 빈칸: 기준선 완성 직후 창이 덜 찼을 때의 처리가 초안에 없었음
   - 계산: 덜 찬 창으로 평균을 내면, 첫 사이클에 d=10이 한 번 튈 때 창 [10] → 10, [10, 0] → 5, [10, 0, 0] → 3.3. 3연속으로 2를 넘어 주의 경보 (오경보)
   - 결정: 지표마다 값이 5개 모이기 전에는 경보 판정 안 함. 건강도 점수는 첫 사이클부터 표시
2. **이동평균에 넣기 전 사이클 1회 이탈을 ±6에서 자름 (`clip_d: 6.0`, 새 값)**
   - 빈칸: 창이 다 차도 아주 큰 값 하나가 평균을 혼자 끌어올림
   - 계산: 기준선이 평균 4cm, 표준편차 2cm일 때 도착 오차 50cm 한 번 → d = 23. 자르지 않으면 5사이클 평균이 23/5 = 4.6이고, 이 값이 창에 머무는 5사이클 동안 3을 넘어 경고. ±6에서 자르면 6/5 = 1.2로 경보 없음
   - 6인 이유: 건강도 0점 기준(d=6)과 같음. "이 이상은 어차피 최악"
   - 한계: 5사이클 안에 큰 값이 두 번이면 (6+6)/5 = 2.4라 주의가 뜰 수 있음. 두 번이면 실제 이상일 가능성이 높다고 보고 허용
   - 건강도 점수는 자르지 않은 값으로 계산 → 튄 사이클은 점수에 바로 0점으로 보이고, 자동 대응(경보)만 추세를 기다림
   - 테스트: `test_single_spike_gives_no_alarm`
3. **값이 없는 사이클은 그 지표의 연속 횟수를 그대로 둠**
   - 빈칸: 이동 실패로 도착 오차가 None일 때 "N사이클 연속"을 어떻게 셀지
   - 리셋하면 나빠지는 중에 실패가 끼어 경보가 늦어짐 (나빠질수록 실패가 늘어 거꾸로 됨). 증가시키면 값 없이 "넘었다"고 치는 셈 → 그대로 둠
   - 테스트: `test_missing_value_skipped_not_alarmed`
4. **학습과 판정의 경계**
   - 실패 사이클도 판정은 함 (학습에만 안 넣음). 나빠지는 로봇은 실패가 늘어서, 빼면 가장 중요한 순간을 놓침
   - K번째 사이클은 학습에 쓰고 판정은 다음 사이클부터. 기준선을 만든 데이터로 자기 자신을 채점하지 않기 위해
   - 기준선에서 값이 2개 미만인 지표는 판정 안 함 (표준편차를 못 구함, 예: 토픽이 안 들어온 경우)
   - 테스트: `test_learning_progress_and_success_only`
5. **ground truth 도구: PosePublisher → OdometryPublisher**
   - PosePublisher는 모든 링크 위치를 TF 형태 한 묶음으로 보내서, base 링크를 이름으로 골라야 하고 로봇 TF와 섞일 위험이 있음. OdometryPublisher는 본체 위치 하나를 nav_msgs/Odometry 별도 토픽으로 보냄
   - 직접 써 보고 막힌 것이 아니라 문서상 구조를 보고 단순한 쪽을 고름
   - 월드 기준 절대 위치를 내는지는 **Gazebo 확인 전**. 정지 상태에서 x, y가 0 근처인지로 검증 예정 (로봇이 월드 원점에서 시작하므로 상대 위치여도 결과는 같음)

### PHM 4단계 시나리오 A 바퀴 마모: 준비 (11/23 건형 확정: 초안대로)
- 수업 패키지 변경 (무엇을, 왜):
  - `my_robot_description/urdf/my_robot.urdf.xacro`: xacro 인자 `wheel_wear`(기본 0). 바퀴의 충돌·모양·관성 반지름만 `0.05 × (1 − wheel_wear)`로 줄이고, DiffDrive `<wheel_radius>`는 원래 0.05 그대로 → odom이 실제보다 많이 갔다고 믿음. 메시지 조작 없음
  - `my_robot_gazebo/launch/gazebo.launch.py`: launch 인자 `wheel_wear` (전체 재시작 방식 대비용)
- 단계 전환: 처음엔 로봇 모델만 다시 스폰하는 방식(Gazebo·Nav2·툴킷 계속 실행)을 시도 → 19·20번 문제로 **전체 재시작 방식으로 변경** (`gazebo.launch.py wheel_wear:=0.01` 등, 단계마다 터미널 4개 다시 켜기)
- 러너 변경: 이동평균·연속 수(운영 이력)를 기준선 파일에 저장 → 단계마다 러너를 다시 켜도 추세가 이어짐 (pytest: 중간에 다시 켜도 경보 시점 동일). `csv_name`으로 한 CSV에 이어 쓰기, `condition` 열(시험 조건 표시, 판정에는 안 씀)
- 단계: 1, 2, 3, 4, 6% × 4사이클 (0%는 3단계 empty 기준선 재사용). 최대 6%인 이유: 바퀴(앞)만 작아지면 뒤 볼캐스터와 세 점 지지라 차체가 앞으로 숙여지고 LiDAR(바퀴 바로 위, 높이 약 0.10m)도 같이 숙여짐. 앞쪽 빔이 바닥에 닿는 거리(계산): 3% 11m, 4% 8.5m, 6% 5.7m, 8% 4.2m → 4% 이상은 "LiDAR 기울어짐"이 섞임, 해석 때 따로 표시
- 예상(계산, 측정 전): 이동 잔차 +마모율만큼, 회전 비율 1.149 × (1 − 마모율), 구동 추종률 변화 없음 → 2%에서 주의, 3%에서 경고. E105·E401은 6%까지 안 뜰 가능성이 높음

**19. 로봇 재스폰 뒤 AMCL이 /initialpose를 못 받아 rviz2와 Gazebo 위치가 어긋남**
- 증상: `respawn_robot --wear 0.0 --point B` 뒤 Gazebo에서는 로봇이 B에 있는데 rviz2에서는 다른 곳(오른쪽 위)에 있고 파티클이 넓게 퍼지고 스캔이 지도와 어긋남. 이어서 돌린 러너 1사이클째 이동 실패(status 6), E402·E105, 건강도 14.3
- 진단 도구·방법: `~/.ros/log` 대조. `create_*.log` "Entity creation successful"(재생성 성공), motion_monitor 로그 "위치 재설정 수신"(/initialpose 발행은 됨), 그런데 `amcl_*.log`에는 시작 때 기본값 외에 `initialPoseReceived`가 없음 → **AMCL만 못 받음**. 스크립트가 구독자 수가 1 이상(motion_monitor가 먼저 연결)이 되자마자 한 번만 보내서, 아직 연결 안 된 AMCL은 놓침 (새 발행자는 구독자마다 연결되는 데 시간이 걸림)
- 조치: 그래프에 보이는 `/initialpose` 구독자 전부(amcl 포함)와 연결될 때까지 최대 15초 기다린 뒤 0.5초 간격으로 3번 발행. 시간 초과 시 rviz2 2D Pose Estimate 안내
- 실패한 시험 데이터 정리: `wear_A.csv` → `wear_A_trial1_failed.csv`로 보관, 기준선 파일의 이동평균 이력(그 1사이클이 들어감)만 비움 (평균·표준편차는 그대로, 정리 전 파일은 `phm_baseline_before_trial1_cleanup.json`)
- 재스폰 자체(삭제 → 생성)는 Gazebo가 죽지 않고 동작함
- 전후: AMCL /initialpose 수신 0회 → 재시험에서 rviz2와 Gazebo 위치 일치 (건형 확인)

**20. 로봇 재스폰 뒤 IMU가 두 배로 나와 회전 비율이 절반으로 떨어짐 → 재스폰 방식 포기**
- 증상: 19번 조치 후 재스폰(마모 0%) 뒤 2사이클이 성공했지만 건강도 39.9 / 38.1, 가장 나쁜 지표 rot_ratio. 회전 비율 0.433 / 0.539 (기준선 1.149), 회전 잔차 RMS 0.516 / 0.398rad/s (기준선 0.097). 마모가 없는데도 크게 나쁨
- 진단 도구·방법: 같은 CSV의 ⑧ 타이밍 열 비교. `imu_interval_ms`가 9.99ms(3단계, 100Hz) → **5.02ms(200Hz)**. scan 99.9ms, odom 20.0ms, joint_states 1.5ms는 그대로 → IMU만 두 배. 지운 로봇의 IMU 센서가 Gazebo IMU 시스템에 남아 같은 `imu` 토픽으로 계속 발행하는 것으로 추정(값은 확인 안 함). 회전 비율이 기준선의 약 0.38~0.47배인 것도 "가장 가까운 IMU 샘플과 짝짓기"에서 절반 가까이가 엉뚱한 센서 값과 짝지어진 것과 맞음
- 조치: 재스폰 방식 포기, 스크립트(`respawn_robot`) 삭제. 계획해 둔 대체안인 전체 재시작으로 단계 전환 (사전 합의: "재스폰이 30분 안에 안 되면 B")
- 실패한 시험 데이터 정리: `wear_A.csv` → `wear_A_trial2_imu_dup.csv`로 보관, 기준선 파일의 이동평균 이력만 다시 비움
- 교훈: ⑧ 통신·타이밍 지표가 시뮬레이션 도구 문제(센서 중복)를 바로 드러냄. 건강도가 갑자기 떨어지면 "로봇 열화"인지 "측정 경로 이상"인지 타이밍 지표부터 확인

**21. 마모 1% 단계에서 변화가 없음: 마모가 실제로 적용되지 않았음**
- 증상: `condition:=wheel_wear=0.01`로 4사이클, 모두 [정상]. 회전 비율 평균 1.146(기준선 1.149, 예상 1.138), 이동 잔차 평균 2.60%(기준선 2.57%, 예상 약 3.6%) → 예상한 변화가 없음
- 진단 도구·방법: 실행 중인 `/robot_state_publisher`의 robot_description에서 바퀴 cylinder 반지름 확인 → `radius="0.05"`(마모 1%면 0.0495). Gazebo는 이 description으로 로봇을 만들므로 마모가 안 들어간 상태. 설치된 launch·xacro는 `wheel_wear` 반영 확인됨 → Gazebo를 `wheel_wear:=0.01` 없이 켠 것으로 추정
- 조치: 이 4사이클의 condition 열을 `wheel_wear=0.00`으로 고침(원본 `wear_A_before_relabel.csv`). 러너를 켜기 전에 반지름 확인 명령을 필수 단계로 넣음
- 덤으로 확인된 것 (마모 0%, 전체 재시작 직후 4사이클): E403 0회(18번 수정 후 첫 Gazebo 확인, 수정 전 24/24), 오경보 0회, 건강도 93~100, /imu 간격 9.99ms(중복 없음), 재시작 후 이동평균 이력 이어짐

### 시나리오 A 바퀴 마모 측정 (11/23, `wear_A.csv`, 단계마다 전체 재시작, 4사이클)
| 마모 | 실제 반지름 | 회전 비율 평균 (예상) | 이동 잔차 평균 (예상) | PHM 상태 | 건강도 | 기존 알람 |
|---|---|---|---|---|---|---|
| 0% (기준선 empty 8사이클) | 50.0mm | 1.149 | 2.57% | - | - | - |
| 0% (재시작 확인) | 50.0mm | 1.146 | 2.60% | 정상 4/4 | 93~100 | E402 1회 |
| 1% | 49.5mm | 1.137 (1.138) | 3.59% (3.6) | 정상 4/4 | 82~96 | E402 2회 |
| 2% | 49.0mm | 1.126 (1.126) | 4.17% (4.6) | 정상 4/4 | 67~92 | E402 2회 |
| 3% | 48.5mm | 1.110 (1.115), 3사이클 | 4.97% (5.2) | 정상 2 → **주의**(3번째, rot_ratio) | 64~67 | E402 3회 |
| 4% | 48.0mm | 1.115 (1.103)* | 5.91% (6.7) | 주의(1사이클째) → **경고**(3사이클째) | 45~73 | E402 2회 |
- 4% (\*): 1사이클째 주의로 자동 대응(속도 70% → 경고 뒤 50%)이 들어가서 속도 변화가 지표에 섞임. 속도 100%였던 1사이클째 회전 비율 1.091(예상 1.103), 느려진 2~4사이클째 1.118~1.131
- 사이클 타임: 51.0s(0~3%) → 71.6s(속도 70%) → 75.2s, 95.2s(속도 50%, 4사이클째는 팔 작업 보류) = 자동 대응의 대가
- **결론: PHM 주의 15번째 사이클(3%), 경고 18번째(4%). E105·E401은 19사이클(4%) 끝까지 0회.** 기존 알람만 있었다면 4% 마모까지 아무 신호가 없었음
- 6% 단계는 실행하지 않음 (11/23 마감 일정, 자동 대응의 속도 변화 혼입, 4% 이상 LiDAR 기울어짐 혼입)
- 정비 리포트: `ros2 run robot_field_toolkit phm_report ~/field_toolkit_reports/wear_A.csv` → `docs/report_wear_A.md`, 원자료 `docs/wear_A.csv`
- 2%: 회전 비율은 예상과 같음. 이동 잔차는 예상보다 작게 오름(기준선 대비 +1.6%p, 예상 +2.0%p). 단계 끝 이동평균 d: 회전 비율 1.97, 이동 잔차 1.47 → 주의 기준 2 바로 아래. "2%에서 주의" 예상은 빗나감
- 1%: 두 지표 모두 계산한 예상값과 거의 같게 움직임 → 마모가 odom에 들어가는 경로(실제 바퀴만 작고 DiffDrive는 원래 반지름)가 의도대로 동작. 이동평균 이탈 d는 이동 잔차 약 1(평소 흔들림 안)이라 경보 없음 = 예상대로

**22. 첫 예지 경보(주의)를 로그로 찍다가 러너가 죽음: rclpy 로거 심각도 고정**
- 증상: 마모 3% 3번째 사이클에서 health_runner가 `ValueError: Logger severity cannot be changed between calls.`로 종료. 그 사이클이 처음으로 [주의]가 된 사이클
- 진단 도구·방법: traceback이 `finish_cycle`의 로그 한 줄(`log = warn if 주의·경고 else info` 후 `log(...)`)을 가리킴. rclpy 로거는 호출 위치마다 첫 심각도를 기억하고 다른 심각도로 부르면 예외를 냄. 같은 패턴으로 `python3 -c`에서 한 줄로 warn·error를 번갈아 부르면 같은 예외가 나는 것을 재현. 같은 패턴이 `alarm_reporter.py`(WARN 코드는 warn, ALARM 코드는 error를 한 줄에서)에도 있었음 → WARN(E401)과 ALARM(E402·E403)을 함께 쓰는 nav_monitor가 같은 이유로 죽을 수 있음. 이전 실행에서 실제로 죽었는지는 확인 못 함 (노드 로그에 이 예외 기록 없음)
- 조치: 두 곳 모두 심각도별로 호출 줄을 나눔. 3번째 사이클의 CSV 줄과 기준선·이력 저장은 로그 전에 끝나서 데이터 손실 없음
- 교훈: 정상일 때만 돈 코드 경로(경보 로그)는 경보가 처음 뜰 때 터진다. pytest는 core만 봐서 노드의 로그 경로를 못 잡음

### PHM 4단계 자동 대응과 정비 리포트 (11/23, 값은 초안)
- `core/phm_response.py`: 주의 → speed_override 70%, 경고 → 50% + 팔 정밀 작업 보류 + 재캘리브레이션 권고. 러너가 판정 뒤 `/safety_interlock` speed_override를 바꾸고, 경고면 다음 사이클 도착 후 팔 작업을 건너뜀(실패가 아니라 대응, `note` 열). 마모 4%에서 실제 동작 확인 (로그 `[자동 대응] 속도 오버라이드 70%` → `50%`, 4사이클째 note "PHM 경고: 팔 정밀 작업 보류")
- `core/phm_report.py` + `phm_report` 명령: 조건별 요약, PHM vs 기존 알람 첫 발생, 지표별 이탈·추세선(최근 8사이클 최소제곱)으로 남은 사이클, 권고 우선순위, ground truth 채점 표
- 러너가 이어 쓰는 CSV에 열이 늘면(response, note) 기존 줄을 새 열 순서로 다시 씀 (4% 실행 때 "CSV 열 갱신: 15줄 다시 씀")
- 권고 문구에 "팔 정밀 작업 보류"가 두 번 찍히던 것 수정 (리포트 파일도 고침)
