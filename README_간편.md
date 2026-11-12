# point_lio_orig 간편 사용법

원본 Point-LIO(+ 켤 수 있는 in-filter ZUPT)를 Go2 실기에서 띄우는 방법입니다. 자세한 설명은 `README_상세.md`, 토픽 내용은 `TOPICS_필드설명.txt`를 보세요.

## 0. 준비 (한 번만)

- Ubuntu 22.04 + ROS2 Humble, `unitree_ros2`(unitree_go 메시지)가 설치돼 있어야 합니다.
- 로봇 연결 설정(`RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_DOMAIN_ID`)은 **평소 쓰시는 대로** 터미널에 해 두세요. 실행 스크립트는 이 값을 바꾸지 않습니다.

```bash
# 이 폴더를 원하는 곳에 둡니다. 예: ~/go2/PL_orig_ws
cd <이 폴더>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`unitree_ros2`가 `~/unitree_ros2`가 아닌 곳에 있으면 실행 전에 알려 주세요:

```bash
export UNITREE_ROS2_SETUP=<unitree_ros2 경로>/cyclonedds_ws/install/setup.bash
```

## 1. 실행

```bash
cd <이 폴더>
live/run_live.sh            # 기본: ZUPT 끔, /path 끔, PCD 저장 끔
live/run_live.sh --zupt     # 정지 구간 ZUPT 켜기 (추천: 정지·출발이 잦을 때)
```

- 켜진 뒤 **로봇을 몇 초 가만히 세워 두세요** (초기화).
- 멈출 때는 **Ctrl+C** 한 번 → 띄운 노드가 모두 꺼집니다.
- 실행마다 `live/runs/<날짜_시각>/`에 로그와 설정 사본이 남습니다.

| 옵션 | 뜻 |
|---|---|
| `--zupt` | in-filter ZUPT 켜기 (정지 판정 노드도 같이 뜸) |
| `--path` | `/hknu/path` 발행 켜기 (주행 내내 계속 쌓임) |
| `--pcd` | 종료할 때 지도 PCD 저장 (`src/point_lio_orig/PCD/`) |
| `--pcd-interval N` | PCD를 N 스캔씩 나눠 저장 (기본 -1 = 한 파일, 메모리 주의) |
| `--name 이름` | 기록 폴더 이름에 붙일 꼬리표 |

## 2. 받을 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/hknu/aft_mapped_to_init` | `nav_msgs/Odometry` | L1 위치·자세 (프레임 `camera_init` → `aft_mapped`) |
| `/hknu/cloud_registered` | `sensor_msgs/PointCloud2` | 이번 스캔을 월드 좌표로 바꾼 점군 (누적 아님) |
| `/hknu/Laser_map` | `sensor_msgs/PointCloud2` | 시작할 때 1회만 나오는 초기 지도 |
| `/tf` | | `camera_init` → `aft_mapped` |

주의 두 가지:
1. 위치·자세는 **L1 라이다 기준**입니다(로봇 몸통 `base_link` 아님). x축도 로봇 전진 방향이 아닙니다.
2. Odometry의 `twist.linear`는 **월드 기준 속도**입니다.

## 3. 안 될 때

| 증상 | 확인 |
|---|---|
| `unitree_go 메시지를 찾을 수 없습니다` | `UNITREE_ROS2_SETUP` 경로 지정 |
| `point_lio_orig 가 빌드되지 않았습니다` | `colcon build --symlink-install` |
| 출력이 안 나옴 | `ros2 topic hz /utlidar/cloud`, `ros2 topic hz /hknu/l1_imu_fixed`로 입력 확인. 로봇 연결 설정(DOMAIN_ID 등) 확인 |
| 시작 직후 자세가 튐 | 다시 실행하고 처음 몇 초 로봇을 가만히 두기 |
| 로그 | `live/runs/<실행>/pointlio.log`, `l1_imu_fix.log`, `zvd_node.log`, `env.txt` |
