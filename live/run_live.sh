#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# point_lio_orig 실기(라이브) 실행 스크립트 — 한 번에 띄우고 Ctrl+C 로 전부 종료
#
# 띄우는 것 (순서대로):
#   1) /sportmodestate 수신 유지 (꺼지면 로봇 LiDAR 발행이 멈춤)
#   2) tools/l1_imu_fix.py  -> /hknu/l1_imu_fixed  (L1 자이로 + 몸통 가속도, time_sync:=false)
#   3) tools/zvd_node.py    -> /hknu/zupt_active   (--zupt 일 때만)
#   4) Point-LIO (/hknu/laserMapping)              (launch/go2_orig_live.launch.py)
#
# 로봇 연결 설정(RMW_IMPLEMENTATION, CYCLONEDDS_URI, ROS_DOMAIN_ID 등)은 건드리지 않습니다.
# 이 스크립트를 실행하는 터미널에 미리 해 둔 설정을 그대로 씁니다 (실행 기록 env.txt 에 남김).
#
# 사용법:  live/run_live.sh [--zupt] [--path] [--pcd] [--pcd-interval N] [--name 이름]
set -uo pipefail

LIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$LIVE_DIR/.." && pwd)"
TOOLS_DIR="$WS_DIR/tools"

ZUPT=false; PATH_EN=false; PCD=false; PCD_INTERVAL=""; RUN_NAME=""
usage() { sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }
while [ $# -gt 0 ]; do
  case "$1" in
    --zupt) ZUPT=true ;;
    --path) PATH_EN=true ;;
    --pcd) PCD=true ;;
    --pcd-interval) PCD_INTERVAL="${2:?--pcd-interval 값 필요}"; shift ;;
    --name) RUN_NAME="${2:?--name 값 필요}"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[run_live] 모르는 옵션: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

# ── ROS 환경 (연결 설정은 손대지 않음) ─────────────────────────────
set +u
ROS_DISTRO_USE="${ROS_DISTRO:-humble}"
if [ -z "${AMENT_PREFIX_PATH:-}" ] && [ -f "/opt/ros/$ROS_DISTRO_USE/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO_USE/setup.bash"
fi
# unitree_go 메시지: 이미 source 돼 있으면 그대로, 아니면 UNITREE_ROS2_SETUP 또는 ~/unitree_ros2 의 install 만 source
if ! python3 -c "import unitree_go.msg" 2>/dev/null; then
  UNITREE_SETUP="${UNITREE_ROS2_SETUP:-$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash}"
  [ -f "$UNITREE_SETUP" ] && source "$UNITREE_SETUP"
fi
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

fail() { echo "[run_live] 오류: $*" >&2; exit 1; }
command -v ros2 >/dev/null || fail "ros2 를 찾을 수 없습니다. ROS2 Humble 을 source 하세요."
python3 -c "import unitree_go.msg" 2>/dev/null || fail "unitree_go 메시지를 찾을 수 없습니다. UNITREE_ROS2_SETUP=<unitree_ros2 install/setup.bash 경로> 로 알려 주세요."
ros2 pkg prefix point_lio_orig >/dev/null 2>&1 || fail "point_lio_orig 가 빌드되지 않았습니다: cd $WS_DIR && colcon build --symlink-install"

# ── 실행 기록 폴더 ───────────────────────────────────────────────
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$LIVE_DIR/runs/${STAMP}${RUN_NAME:+_$RUN_NAME}"
mkdir -p "$RUN_DIR/config"
CFG="$(ros2 pkg prefix point_lio_orig)/share/point_lio_orig/config/go2_orig.yaml"
cp -L "$CFG" "$RUN_DIR/config/" 2>/dev/null || true
{
  echo "date=$(date -Iseconds)"; echo "host=$(hostname)"; echo "ws=$WS_DIR"
  echo "options: zupt=$ZUPT path=$PATH_EN pcd=$PCD pcd_interval=${PCD_INTERVAL:--}"
  echo "ROS_DISTRO=${ROS_DISTRO:-}"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"; echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-}"
  echo "config_sha256=$(sha256sum "$RUN_DIR/config/go2_orig.yaml" 2>/dev/null | cut -d' ' -f1)"
  echo "tools_sha256:"; (cd "$TOOLS_DIR" && sha256sum l1_imu_fix.py go2_calib.py zvd_node.py)
} > "$RUN_DIR/env.txt"
echo "[run_live] 기록 폴더: $RUN_DIR"

# ── 프로세스 띄우기 / 종료 처리 ────────────────────────────────────
PIDS=()
spawn() {  # spawn <이름> <명령...> : 각자 새 프로세스 그룹으로 띄우고 로그를 남김
  local name="$1"; shift
  echo "$*" >> "$RUN_DIR/cmd.txt"
  setsid "$@" > "$RUN_DIR/$name.log" 2>&1 &
  PIDS+=("$!"); echo "[run_live] $name 시작 (pid $!)"
}
STOPPING=false
cleanup() {
  $STOPPING && return; STOPPING=true
  echo; echo "[run_live] 종료 중... (Point-LIO 가 PCD 를 저장할 시간을 줌)"
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
  for _ in $(seq 1 30); do
    alive=false; for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    $alive || break; sleep 0.5
  done
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
  echo "[run_live] 끝. 로그: $RUN_DIR"
}
trap 'cleanup; exit 0' INT TERM
trap cleanup EXIT

spawn keepalive ros2 topic echo /sportmodestate --no-arr --no-str
sleep 2
spawn l1_imu_fix python3 "$TOOLS_DIR/l1_imu_fix.py" --ros-args -r __ns:=/hknu \
      -p out_topic:=/hknu/l1_imu_fixed -p time_sync:=false
if $ZUPT; then
  spawn zvd_node python3 "$TOOLS_DIR/zvd_node.py" --ros-args -r __ns:=/hknu \
        -r /zupt_active:=/hknu/zupt_active
fi
sleep 2
echo "[run_live] 로봇을 몇 초 가만히 세워 두세요 (Point-LIO 초기화)."
LAUNCH_ARGS=(zupt_en:=$ZUPT path_en:=$PATH_EN pcd_save_en:=$PCD)
[ -n "$PCD_INTERVAL" ] && LAUNCH_ARGS+=(pcd_interval:=$PCD_INTERVAL)
spawn pointlio ros2 launch point_lio_orig go2_orig_live.launch.py "${LAUNCH_ARGS[@]}"
echo "[run_live] 실행 중. 멈추려면 Ctrl+C. (Point-LIO 로그: tail -f $RUN_DIR/pointlio.log)"

# 하나라도 죽으면 전부 정리
while :; do
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "[run_live] 프로세스 종료 감지 (pid ${PIDS[$i]}) - 전체 종료합니다. 로그를 확인하세요."
      exit 1
    fi
  done
  sleep 1
done
