# 개발 일지

## 11/19
- `gz: command not found` → 설치는 돼 있었는데 ROS 환경을 불러오지 않음 → `source /opt/ros/jazzy/setup.bash`, `.bashrc`에 추가
- Gazebo RTF 60%, /scan 6.5Hz → `glxinfo`로 확인하니 llvmpipe(CPU 렌더링) → `/dev/dxg`로 WSL의 GPU 인식 확인 → `GALLIUM_DRIVER=d3d12` 설정 → /scan 9.2Hz
- 수업 작업공간을 그대로 복사했더니 build/install/log에 교육장 노트북 경로가 남아 있음 → 지우고 다시 빌드
- `gh auth login` 브라우저 자동 열기 실패, 요청 과다(slow_down) → 기다린 뒤 브라우저에서 직접 코드 입력

## 11/20
### 수업 패키지 변경
- `my_robot_slam`, `my_robot_navigation`을 `~/course_pkgs`에서 저장소(src)로 옮김. Nav2 주행 감시(E4xx)와 시운전에 필요.
- `my_robot_navigation/launch/navigation.launch.py`: `map` 인자 기본값을 빈 문자열에서 패키지 경로의 `maps/simple_room.yaml`로 변경. 이유: 지도를 저장소 안에 두고 PC가 바뀌어도 절대경로 없이 실행되게.
- `my_robot_navigation/CMakeLists.txt`: 설치 폴더에 `maps` 추가 (위 기본 경로가 설치 폴더를 가리키므로).
- `my_robot_slam/rviz/mapping.rviz`, `my_robot_navigation/rviz/navigation.rviz`: 창 크기를 1280x720(위치 50,50)으로 줄임. 이유: 저장된 창 크기(mapping 2748x1662, navigation 1854x1011)가 내 화면(1920x1080)보다 크거나 꽉 차서 rviz2를 쓰기 불편.
- `my_robot_navigation/launch/navigation.launch.py`: cmd_vel remap 수정. 이유: `ros2 topic info /cmd_vel -v`에서 발행자 6개(controller_server 1, behavior_server 4, velocity_smoother 1)이고 velocity_smoother가 /cmd_vel을 구독도 함 → 스무더가 자기 출력을 다시 입력으로 받는 되먹임 고리. 공식 nav2_bringup처럼 controller_server·behavior_server 출력과 velocity_smoother 입력을 `cmd_vel_nav`로 바꾸고, 스무더 출력만 `/cmd_vel`로 냄. 수정 후 /cmd_vel 발행자 1개(velocity_smoother), 구독자 1개(fault_injector).
- `my_robot_navigation/launch/navigation.launch.py`: `lifecycle_manager_navigation` 시작을 `TimerAction`으로 10초 늦춤. 이유: Nav2 Goal이 `Goal was rejected by server`로 거부됨 → `ros2 lifecycle get`으로 보니 controller·smoother만 active, planner_server부터 inactive → `~/.ros/log/planner_server_*.log`에 `Failed to activate global_costmap because transform from base_footprint to map did not become available before timeout`. 설정상 기다림 시간(initial_transform_timeout)은 60초인데 활성화 시작 0.5초 만에 실패했고, AMCL의 map TF는 그 직후에 나옴. 위치 추정(AMCL)이 먼저 자리 잡은 뒤 주행 노드를 켜도록 순서를 늦춤. 수정 후 Nav2 노드 9개 모두 active, planner_server 오류 없음.
