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
