# point_lio_orig 상세 설명

Unitree Go2 + L1 LiDAR에서 **원본 Point-LIO**를 실기로 돌리기 위한 워크스페이스입니다. 원본에 더한 것은 **in-filter ZUPT 하나**뿐이며 기본은 꺼져 있습니다. 레그 오도메트리 융합, 루프 클로저(SC-PGO)는 들어 있지 않습니다.

## 1. 폴더 구조

```
go2_PL_Orig_ws/
  src/point_lio_orig/            Point-LIO 패키지 (GPL-2.0)
    config/go2_orig.yaml         Go2 설정 (Point-LIO 노드만 읽음)
    launch/go2_orig_live.launch.py   /hknu remap, 노드 이름 /hknu/laserMapping
    src/, include/               원본 a8e2d0d + ZUPT
    Log/, PCD/                   실행 중 로그·PCD가 써지는 곳 (지우지 마세요)
  tools/                         (MIT)
    l1_imu_fix.py                IMU 브리지 -> /hknu/l1_imu_fixed
    go2_calib.py                 브리지가 쓰는 보정값 (R_LB, LEVER, ACC_SCALE)
    zvd_node.py                  정지 판정 -> /hknu/zupt_active
  live/run_live.sh               실기 실행 (한 번에 띄우고 Ctrl+C로 종료)
  live/runs/<실행>/              실행마다 로그·설정 사본
  README_간편.md, README_상세.md, TOPICS_필드설명.txt
  LICENSE, NOTICE, CITATION.cff
```

경로는 전부 스크립트 자기 위치 기준이라 폴더를 어디에 두어도 됩니다.

## 2. 원본과 다른 점

| 항목 | 원본 (dfloreaa/point_lio_ros2 a8e2d0d) | 이 패키지 |
|---|---|---|
| 패키지 이름 | `point_lio` | `point_lio_orig` (효신 스택의 `point_lio`와 같이 설치돼도 안 겹침) |
| 추정기 | 예측·IMU 갱신·점-평면 갱신·ikd-Tree | **그대로** |
| ZUPT | 없음 | `update_zupt()` 추가, `zupt_en` 기본 `false` |
| 노드 이름 | `/laserMapping` | `/hknu/laserMapping` (launch에서 지정) |
| 발행 토픽 | `/aft_mapped_to_init` 등 | launch remap으로 `/hknu/...` (C++ 발행 코드는 그대로) |
| TF | `camera_init` → `aft_mapped` | 그대로 |
| 설정 | L1 기본 | Go2용 `go2_orig.yaml` (IMU는 브리지 출력, `/path`·PCD·ZUPT 기본 끔) |

## 3. 데이터 흐름

```
로봇 ──/utlidar/cloud──────────────────────────────► Point-LIO (/hknu/laserMapping)
     ──/utlidar/imu (L1 자이로)──┐                        ▲          │
     ──/lowstate (몸통 가속도)───┴► l1_imu_fix ─/hknu/l1_imu_fixed┘          ├─► /hknu/aft_mapped_to_init
     ──/utlidar/robot_odom──► zvd_node ─/hknu/zupt_active─► (--zupt 일 때)  ├─► /hknu/cloud_registered
     ◄─/sportmodestate 계속 수신 (끊기면 LiDAR가 멈춤)                        └─► /tf, /hknu/Laser_map(1회)
```

### 3.1 IMU 브리지 (`l1_imu_fix.py`)

L1 내장 가속도계는 고장이라(실제 가속도와 상관 0.19 이하) 자이로만 L1 것을 쓰고, 가속도는 몸통 IMU 값을 L1 프레임으로 옮겨 씁니다.

$$
\mathbf a_{L1} = s\,R_{LB}\,\mathbf a_{body} + \boldsymbol\omega\times(\boldsymbol\omega\times\mathbf r) + \boldsymbol\alpha\times\mathbf r,
\qquad s=\frac{9.807}{9.465}
$$

- $R_{LB}$: base_link → L1 회전 (`go2_calib.py`), $\mathbf r$: 레버암, $\boldsymbol\alpha$: 각가속도(EMA 0.75).
- 실행 스크립트는 `time_sync:=false`로 띄웁니다 (켰을 때 결과가 1.7~2.3배 나빠졌던 검증 결과).

### 3.2 정지 판정 (`zvd_node.py`)

다리 오도메트리 속력 $|\mathbf v_{leg}|$로 판정합니다.

- 정지 시작: $|\mathbf v_{leg}| < v_{th}=0.05$ m/s 가 $0.25$ s 이상 지속
- 정지 해제: $|\mathbf v_{leg}| > 1.5\,v_{th}=0.075$ m/s (히스테리시스)

### 3.3 in-filter ZUPT (`esekfom.hpp`의 `update_zupt`)

Point-LIO 출력 모델(`use_imu_as_input: false`)의 상태에는 월드 속도 $\mathbf v$(오차 상태 인덱스 12–14)와 각속도 $\boldsymbol\omega$(15–17)가 들어 있습니다. 정지 중에는 "둘 다 0"을 측정으로 넣습니다.

$$
\mathbf z = \mathbf 0 - \begin{bmatrix}\hat{\mathbf v}\\ \hat{\boldsymbol\omega}\end{bmatrix},\qquad
H = \begin{bmatrix} 0 & \cdots & I_3 & 0 \\ 0 & \cdots & 0 & I_3 \end{bmatrix}\ (\text{열 }12\text{–}17),\qquad
R = \mathrm{diag}(\sigma_v^2 I_3,\ \sigma_\omega^2 I_3)
$$

$$
S = HPH^\top + R,\quad K = PH^\top S^{-1},\quad \delta\mathbf x = K\mathbf z,\quad
\mathbf x \leftarrow \mathbf x \boxplus \delta\mathbf x,\quad P \leftarrow P - K H P
$$

- $\sigma_v^2$ = `zupt_cov_vel` 0.01, $\sigma_\omega^2$ = `zupt_cov_omg` 0.0001.
- 점 갱신(`update_iterated_dyn_share_modified`)이 끝날 때마다, `/hknu/zupt_active`가 true이고 1.0 s(`zupt_timeout`) 안에 받은 값일 때만 호출됩니다.
- `use_imu_as_input: true`인데 `zupt_en: true`면 노드가 시작하지 않고 FATAL로 끝납니다(입력 모델에는 이 갱신이 없음).
- 효과(효신 님 실험, bag 3개 n=3): 정지 구간 yaw 붕괴를 막음. 예: ekf_test1에서 최대 yaw 차이 1926° → 26.3°.

## 4. 빌드

```bash
cd <이 폴더>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

- 필요한 패키지: `pcl_ros`, `pcl_conversions`, `tf2_ros`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, Eigen3, PythonLibs (원본과 같음).
- 실행 파일: `pointlio_mapping` (패키지 `point_lio_orig`).
- 원본은 시작할 때 `src/point_lio_orig/Log/pos_log.txt`를 열고 PCD를 `src/point_lio_orig/PCD/`에 씁니다. 두 폴더를 지우지 마세요.

## 5. 실행 스크립트 (`live/run_live.sh`)

순서: `/sportmodestate` 수신 유지 → 2 s → `l1_imu_fix.py` → (`--zupt`면) `zvd_node.py` → 2 s → Point-LIO launch.

- **로봇 연결 설정은 바꾸지 않습니다.** 터미널에 이미 있는 `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_DOMAIN_ID`를 그대로 쓰고 `env.txt`에 기록만 합니다.
- ROS가 source돼 있지 않으면 `/opt/ros/$ROS_DISTRO`(없으면 humble)를 source합니다.
- `unitree_go` 메시지가 안 보이면 `UNITREE_ROS2_SETUP`(기본 `~/unitree_ros2/cyclonedds_ws/install/setup.bash`)을 source합니다. 이 파일은 메시지만 등록하고 연결 설정은 바꾸지 않습니다.
- 노드는 각자 프로세스 그룹으로 뜨고, Ctrl+C 또는 하나라도 죽으면 전부 SIGINT → (최대 15 s) → SIGTERM → SIGKILL 순으로 끕니다. SIGINT를 먼저 보내는 이유는 Point-LIO가 PCD를 저장할 시간을 주기 위해서입니다.

실행 기록 `live/runs/<YYYYmmdd_HHMMSS>[_이름]/`:

| 파일 | 내용 |
|---|---|
| `env.txt` | 시각, 옵션, ROS·DDS 환경변수, 설정·도구 파일 sha256 |
| `cmd.txt` | 실제로 실행한 명령 |
| `config/go2_orig.yaml` | 그때 쓴 설정 사본 |
| `pointlio.log`, `l1_imu_fix.log`, `zvd_node.log`, `keepalive.log` | 노드별 출력 |

### launch만 직접 쓸 때

```bash
ros2 launch point_lio_orig go2_orig_live.launch.py zupt_en:=true path_en:=false pcd_save_en:=false
```

이때는 브리지(`/hknu/l1_imu_fixed`)와, ZUPT를 켰다면 `zvd_node`(`/hknu/zupt_active`)를 따로 띄워야 합니다.

## 6. 설정값 (`config/go2_orig.yaml` 주요 항목)

| 키 | 값 | 뜻 |
|---|---|---|
| `common.lid_topic` | `/utlidar/cloud` | L1 점군 |
| `common.imu_topic` | `/hknu/l1_imu_fixed` | 브리지 출력 |
| `preprocess.lidar_type` | 5 | Unitree L1 |
| `preprocess.timestamp_unit` | 0 | 점 시각 단위 = 초 |
| `preprocess.blind` | 0.5 | 0.5 m 안 점은 버림 |
| `use_imu_as_input` | false | 출력 모델 (ZUPT에 필요) |
| `mapping.acc_norm`, `gravity` | 9.80665 | 가속도 단위 m/s² |
| `mapping.gravity_align` | true | 월드 z = 중력 반대 |
| `filter_size_surf`, `filter_size_map` | 0.1 | 다운샘플 간격 [m] |
| `publish.path_en` | false | `--path`로 켬 |
| `pcd_save.pcd_save_en` | false | `--pcd`로 켬 |
| `zupt_en` | false | `--zupt`로 켬 |

이 파일은 효신 님 `go2_fix.yaml`(2026-11-12 작업본)에서 레그 융합(`leg_*`)을 빼고 위 기본값만 바꾼 것입니다.

## 7. 주의할 점

1. **좌표계**: 출력 자세·위치는 L1(IMU) 프레임 기준입니다. base_link가 필요하면 `TOPICS_필드설명.txt`의 변환식을 쓰세요. 월드 x·y는 시작할 때 L1이 보던 방향입니다.
2. **속도**: Odometry `twist.linear`는 월드 기준입니다.
3. **지도**: `/hknu/cloud_registered`는 매 스캔 한 장이고 누적이 아닙니다. `/hknu/Laser_map`은 시작할 때 1회만 나옵니다.
4. **시작**: 처음 몇 초 로봇을 가만히 두세요.
5. **keepalive**: `/sportmodestate` 수신이 끊기면 LiDAR 발행이 멈춥니다 (스크립트가 계속 받음).
6. **동시 실행**: 토픽과 노드 이름은 `/hknu`로 분리했지만 `/tf`의 `camera_init`→`aft_mapped`는 다른 Point-LIO와 겹칩니다.
7. **PCD**: `--pcd`에 `--pcd-interval`을 안 주면 주행 전체를 메모리에 모았다가 끝에 저장합니다. 긴 주행에서는 메모리가 부족할 수 있습니다.

## 8. 라이선스와 인용

- `src/point_lio_orig/`: GPL-2.0 (원본 Point-LIO, ROS2 포트, IKFoM). 바꾼 파일 첫 줄에 변경 표시가 있고, 바꾼 내용 전체는 `NOTICE`에 있습니다.
- 나머지(`tools/`, `live/`, 문서): MIT (`LICENSE`).
- 연구에 쓰면 Point-LIO 논문을 인용해 주세요 (`CITATION.cff`):
  D. He, W. Xu, N. Chen, F. Kong, C. Yuan, F. Zhang, "Point-LIO: Robust High-Bandwidth Light Detection and Ranging Inertial Odometry," *Advanced Intelligent Systems*, 2023, 2200459. doi:10.1002/aisy.202200459
