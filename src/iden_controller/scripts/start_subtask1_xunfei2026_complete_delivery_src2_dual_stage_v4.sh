#!/usr/bin/env bash
set -euo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
PKG_DIR="$WS/src/iden_controller"
CREDENTIALS="$PKG_DIR/config/spark_credentials.env"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

if [[ ! -f "$CREDENTIALS" ]]; then
  echo "[ERROR] Missing Spark credential file: $CREDENTIALS" >&2
  exit 1
fi
set -a
source "$CREDENTIALS"
set +a
if [[ -z "${SPARK_API_KEY:-}" || -z "${SPARK_API_SECRET:-}" ]]; then
  echo "[ERROR] SPARK_API_KEY or SPARK_API_SECRET is empty" >&2
  exit 1
fi

CONFLICT_PATTERN='^/(amcl|map_server|move_base|base_driver|my_base_driver|ydlidar_node|ucar_camera|cloud_asr_test2|follow_test|reference_line_follow_|xunfei2026_|subtask1_|factory_room_ocr)'
BASE_PORT="${BASE_SERIAL_PORT:-/dev/ttyS0}"
FLOW_STARTED=0
CLEANUP_DONE=0

flow_nodes() {
  timeout 0.35 rosnode list 2>/dev/null | grep -E "$CONFLICT_PATTERN" || true
}

stop_robot_once() {
  timeout 0.45 rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

wait_for_base_port_release() {
  for _ in {1..20}; do
    [[ -z "$(fuser "$BASE_PORT" 2>/dev/null || true)" ]] && return 0
    sleep 0.10
  done
  return 1
}

cleanup_before_start() {
  local nodes owners pid cmd
  nodes="$(flow_nodes)"
  if [[ -n "$nodes" ]]; then
    stop_robot_once
    timeout 0.65 rosnode kill $nodes >/dev/null 2>&1 || true
    timeout 0.35 rosnode cleanup -y >/dev/null 2>&1 || true
  fi
  owners="$(fuser "$BASE_PORT" 2>/dev/null || true)"
  if [[ -n "$owners" ]]; then
    for pid in $owners; do
      cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
      case "$cmd" in
        *base_driver*|*iden_my_base_driver*) kill -TERM "$pid" 2>/dev/null || true ;;
        *) echo "[ERROR] $BASE_PORT occupied by PID=$pid CMD=$cmd" >&2; return 1 ;;
      esac
    done
  fi
  wait_for_base_port_release
}

cleanup_flow() {
  [[ "$FLOW_STARTED" -eq 0 || "$CLEANUP_DONE" -eq 1 ]] && return
  CLEANUP_DONE=1
  set +e
  echo "[INFO] Cleaning up task nodes..."
  stop_robot_once
  local nodes
  nodes="$(flow_nodes)"
  [[ -n "$nodes" ]] && timeout 0.65 rosnode kill $nodes >/dev/null 2>&1 || true
  timeout 0.35 rosnode cleanup -y >/dev/null 2>&1 || true
  echo "[INFO] Task cleanup complete."
}

trap cleanup_flow EXIT
trap 'exit 130' INT TERM

echo "[INFO] SRC2 dual-stage v4: d1-d9 unchanged; src2 navigation + two src2 parking operations"
cleanup_before_start
FILTER_PATTERN='check crc16 faild|header_crc8 error|check frame end|head_len error'
set +e
FLOW_STARTED=1
rosrun iden_controller xunfei2026_launch_src2_dual_stage_v4.py "$@" 2>&1 \
  | grep --line-buffered -v -E "$FILTER_PATTERN"
STATUS="${PIPESTATUS[0]}"
exit "$STATUS"
