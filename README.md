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
- [ ] 고장 주입 (E1xx)
- [ ] 동작 감시와 알람
- [ ] 주행·팔 인터락 (E3xx)
- [ ] LiDAR 고장 감시 (E2xx)
- [ ] Nav2 위치·도착 감시 (E4xx)
- [ ] MoMa 사이클 시운전 리포트
- [ ] 알람 이력과 로그 번들

## 출처
`my_robot_description`, `my_robot_gazebo`, `my_robot_slam`, `my_robot_navigation`은 로보스타 채용연계 지능형 로봇 엔지니어 양성과정(경기테크노파크) 수업 자료를 바탕으로 합니다.

## 개발 기록
막힌 점과 해결 과정은 [docs/DEVLOG.md](docs/DEVLOG.md)에 있습니다.
