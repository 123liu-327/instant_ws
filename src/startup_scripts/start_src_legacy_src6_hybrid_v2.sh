#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SOURCE_ROOT="$WS/src"
PURE_DEVEL="$WS/devel_src_pure_runtime_v2"
DEFAULT_CRED_FILE="$SOURCE_ROOT/ucar_2026_smart_factory_llm/config/spark_credentials.env"
LEGACY_CRED_FILE="$SOURCE_ROOT/iden_controller/config/spark_credentials.env"

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash

if [[ ! -f "$PURE_DEVEL/setup.bash" ]]; then
  echo "[ERROR] Missing pure unified-src build: $PURE_DEVEL/setup.bash" >&2
  echo "[ERROR] Run $SOURCE_ROOT/build_src_pure_runtime_v2.sh once." >&2
  exit 1
fi
source "$PURE_DEVEL/setup.bash"
export PATH="$PURE_DEVEL/bin:/opt/ros/noetic/bin:/home/ucar/miniconda3/bin:/usr/local/bin:/usr/bin:/bin"
export PYTHONUNBUFFERED=1
export MALLOC_ARENA_MAX="${MALLOC_ARENA_MAX:-2}"
export MALLOC_TRIM_THRESHOLD_="${MALLOC_TRIM_THRESHOLD_:-131072}"
set -u

required_packages=(
  ucar_2026_competition
  ucar_2026_competition_speech
  ucar_2026_smart_factory_llm
  factory_sign_ppocr_rknn_test
  simple_navigator
  vision_triggered_navigator
  ucar_2026_strict_mission
  ucar_2026_track_end_stop
  ucar_2026_traffic_light_rknn_test
  ucar_nav
  ucar_map
  ydlidar
)
for package in "${required_packages[@]}"; do
  if ! rospack find "$package" >/dev/null 2>&1; then
    echo "[ERROR] ROS package is unavailable after migration: $package" >&2
    exit 1
  fi
done

required_files=(
  "$SOURCE_ROOT/ucar_2026_competition/launch/full_competition_legacy_src6_hybrid_v1.launch"
  "$SOURCE_ROOT/ucar_2026_competition/scripts/cloud_asr_complete_sentence_v1"
  "$SOURCE_ROOT/ucar_2026_strict_mission/launch/strict_mission_src6_hybrid_v1.launch"
  "$SOURCE_ROOT/speech_command/package.xml"
)
for file in "${required_files[@]}"; do
  if [[ ! -e "$file" ]]; then
    echo "[ERROR] Required flow file is missing: $file" >&2
    exit 1
  fi
done

if [[ ! -x /home/ucar/miniconda3/bin/python3 ]]; then
  echo "[ERROR] Missing Conda Python required by RKNN nodes." >&2
  exit 1
fi
if ! /home/ucar/miniconda3/bin/python3 -c \
  'import rospy, cv_bridge, rknnlite.api, cv2, numpy, requests' >/dev/null 2>&1; then
  echo "[ERROR] Python/RKNN runtime dependency check failed." >&2
  exit 1
fi

CRED_FILE="${SPARK_CREDENTIALS_FILE:-}"
if [[ -z "$CRED_FILE" ]]; then
  if [[ -f "$DEFAULT_CRED_FILE" ]]; then
    CRED_FILE="$DEFAULT_CRED_FILE"
  elif [[ -f "$LEGACY_CRED_FILE" ]]; then
    CRED_FILE="$LEGACY_CRED_FILE"
  fi
fi
if [[ -n "$CRED_FILE" && -f "$CRED_FILE" ]]; then
  set -a
  source "$CRED_FILE"
  set +a
fi
if [[ -z "${SPARK_API_KEY:-}" || -z "${SPARK_API_SECRET:-}" ]]; then
  echo "[ERROR] Spark credentials are unavailable." >&2
  echo "[ERROR] Expected $DEFAULT_CRED_FILE or exported SPARK_API_KEY/SPARK_API_SECRET." >&2
  exit 1
fi

ENABLE_SIMULATION_VALUE="${ENABLE_SIMULATION:-false}"
SIM_BRIDGE_PORT_VALUE="${SIM_BRIDGE_PORT:-26003}"
SIM_BRIDGE_HOST_VALUE="${SIM_BRIDGE_HOST:-}"
if [[ "$ENABLE_SIMULATION_VALUE" == "true" && -z "$SIM_BRIDGE_HOST_VALUE" ]]; then
  SSH_CLIENT_VALUE="${SSH_CLIENT:-}"
  SIM_BRIDGE_HOST_VALUE="${SSH_CLIENT_VALUE%% *}"
fi
if [[ "$ENABLE_SIMULATION_VALUE" == "true" && -z "$SIM_BRIDGE_HOST_VALUE" ]]; then
  echo "[ERROR] Simulation is enabled but SIM_BRIDGE_HOST is empty." >&2
  exit 1
fi

echo "[INFO] Unified src flow: legacy voice + Spark + src6 navigation/OCR/parking/stop-line."
echo "[INFO] Workspace: $WS"
echo "[INFO] Simulation: $ENABLE_SIMULATION_VALUE"

DRY_RUN_VALUE="${FLOW_DRY_RUN:-${SRC3_DRY_RUN:-0}}"
if [[ "$DRY_RUN_VALUE" == "1" ]]; then
  echo "[INFO] Dry run: validating the launch graph; no ROS node will start."
  roslaunch --nodes ucar_2026_competition \
    full_competition_legacy_src6_hybrid_v1.launch \
    enable_simulation:="$ENABLE_SIMULATION_VALUE" \
    sim_bridge_host:="$SIM_BRIDGE_HOST_VALUE" \
    sim_bridge_port:="$SIM_BRIDGE_PORT_VALUE" "$@"
  exit 0
fi

cleanup() {
  set +e
  rosnode kill /cloud_asr_test2 /src3_legacy_voice_bridge /smart_factory_llm \
    /competition_flow /competition_announcer >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

cleanup
sleep 0.20
if rosparam list >/dev/null 2>&1; then
  rosparam delete /map_goal_picker/goal_x >/dev/null 2>&1 || true
  rosparam delete /map_goal_picker/goal_y >/dev/null 2>&1 || true
  rosparam delete /map_goal_picker/goal_yaw >/dev/null 2>&1 || true
fi

set +e
roslaunch ucar_2026_competition full_competition_legacy_src6_hybrid_v1.launch \
  enable_simulation:="$ENABLE_SIMULATION_VALUE" \
  sim_bridge_host:="$SIM_BRIDGE_HOST_VALUE" \
  sim_bridge_port:="$SIM_BRIDGE_PORT_VALUE" "$@" 2>&1 | sed -u \
    -e '/check crc16 faild(imu)/d' \
    -e '/check crc16 faild(ahrs)/d' \
    -e '/head_len error (imu)/d' \
    -e '/TebLocalPlannerROS: trajectory is not feasible\. Resetting planner/d' \
    -e '/Control loop missed its desired rate/d'
status=${PIPESTATUS[0]}
set -e
exit "$status"
