# 개발 일지

## 11/19
- `gz: command not found` → 설치는 돼 있었는데 ROS 환경을 불러오지 않음 → `source /opt/ros/jazzy/setup.bash`, `.bashrc`에 추가
- Gazebo RTF 60%, /scan 6.5Hz → `glxinfo`로 확인하니 llvmpipe(CPU 렌더링) → `/dev/dxg`로 WSL의 GPU 인식 확인 → `GALLIUM_DRIVER=d3d12` 설정 → /scan 9.2Hz
- 수업 작업공간을 그대로 복사했더니 build/install/log에 교육장 노트북 경로가 남아 있음 → 지우고 다시 빌드
- `gh auth login` 브라우저 자동 열기 실패, 요청 과다(slow_down) → 기다린 뒤 브라우저에서 직접 코드 입력
