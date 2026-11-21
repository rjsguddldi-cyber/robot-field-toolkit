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
