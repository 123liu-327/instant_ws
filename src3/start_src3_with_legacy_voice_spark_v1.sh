#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SRC3="$WS/src3"
CRED_FILE="$WS/src/iden_controller/config/spark_credentials.env"
PURE_DEVEL="$SRC3/devel_pure"

# Do not inherit an already-sourced legacy catkin overlay from the terminal.
# The legacy speech process adds back only its own paths in its child wrapper.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
if [[ ! -f "$PURE_DEVEL/setup.bash" ]]; then
  echo "[ERROR] Missing pure src3 build: $PURE_DEVEL/setup.bash" >&2
  exit 1
fi
source "$PURE_DEVEL/setup.bash"
export PATH="$PURE_DEVEL/bin:/opt/ros/noetic/bin:/home/ucar/miniconda3/bin:/usr/local/bin:/usr/bin:/bin"
export PYTHONUNBUFFERED=1
# Costmap callbacks run on several Python/ROS threads.  Keep glibc from
# retaining one large allocation arena per callback thread on the 8 GB car.
export MALLOC_ARENA_MAX="${MALLOC_ARENA_MAX:-2}"
export MALLOC_TRIM_THRESHOLD_="${MALLOC_TRIM_THRESHOLD_:-131072}"
set -u

case ":${CMAKE_PREFIX_PATH:-}:" in
  *":$WS/devel:"*)
    echo "[ERROR] Legacy catkin devel leaked into the src3 runtime." >&2
    exit 1
    ;;
esac
case ":${LD_LIBRARY_PATH:-}:" in
  *":$WS/devel/lib:"*)
    echo "[ERROR] Legacy library path leaked into the src3 runtime." >&2
    exit 1
    ;;
esac
if rospack find speech_command >/dev/null 2>&1; then
  echo "[ERROR] Legacy speech_command is globally visible; isolation failed." >&2
  exit 1
fi
move_base_exec="$(catkin_find --libexec move_base move_base | head -n 1)"
if [[ "$move_base_exec" != "$PURE_DEVEL/lib/move_base/move_base" ]]; then
  echo "[ERROR] move_base is not the pure src3 executable: $move_base_exec" >&2
  exit 1
fi

if [[ ! -x /home/ucar/miniconda3/bin/python3 ]]; then
  echo "[ERROR] Missing Conda Python required by src3 RKNN nodes." >&2
  exit 1
fi
if ! /home/ucar/miniconda3/bin/python3 -c \
  'import rospy, cv_bridge, rknnlite.api, cv2, numpy, requests' >/dev/null 2>&1; then
  echo "[ERROR] src3 Python/RKNN runtime dependency check failed." >&2
  exit 1
fi

if [[ ! -f "$CRED_FILE" ]]; then
  echo "[ERROR] Missing existing Spark credential file: $CRED_FILE" >&2
  exit 1
fi
set -a
source "$CRED_FILE"
set +a
if [[ -z "${SPARK_API_KEY:-}" || -z "${SPARK_API_SECRET:-}" ]]; then
  echo "[ERROR] Existing SPARK_API_KEY or SPARK_API_SECRET is empty." >&2
  exit 1
fi

ENABLE_SIMULATION_VALUE="${ENABLE_SIMULATION:-false}"
SIM_BRIDGE_PORT_VALUE="${SIM_BRIDGE_PORT:-26003}"
SIM_BRIDGE_HOST_VALUE="${SIM_BRIDGE_HOST:-}"
if [[ "$ENABLE_SIMULATION_VALUE" == "true" && -z "$SIM_BRIDGE_HOST_VALUE" ]]; then
  # When launched over SSH, the client is normally the simulation computer.
  SSH_CLIENT_VALUE="${SSH_CLIENT:-}"
  SIM_BRIDGE_HOST_VALUE="${SSH_CLIENT_VALUE%% *}"
fi
if [[ "$ENABLE_SIMULATION_VALUE" == "true" && -z "$SIM_BRIDGE_HOST_VALUE" ]]; then
  echo "[ERROR] Full flow requires SIM_BRIDGE_HOST." >&2
  echo "[ERROR] Export the simulation computer IP before starting." >&2
  exit 1
fi

echo "[INFO] src3 trial uses existing cloud_asr_test2 and existing Spark KEY:SECRET."
echo "[INFO] src3 built-in ASR/TTS/APIPassword LLM are disabled."
echo "[INFO] Runtime overlay is pure src3; legacy devel is not sourced globally."
echo "[INFO] src3 Python runtime: /home/ucar/miniconda3/bin/python3"
echo "[INFO] Simulation: $ENABLE_SIMULATION_VALUE"
echo "[INFO] Repeated IMU/AHRS CRC warnings are kept in ROS logs but hidden here."
if [[ "$ENABLE_SIMULATION_VALUE" == "true" ]]; then
  echo "[INFO] Full stage sequence: task1 -> task2 -> task3 -> task4 -> task5"
  echo "[INFO] Simulation bridge: $SIM_BRIDGE_HOST_VALUE:$SIM_BRIDGE_PORT_VALUE"
else
  echo "[INFO] Real-car stage sequence: task1 -> task2 -> task4 -> task5"
fi

if [[ "${SRC3_DRY_RUN:-0}" == "1" ]]; then
  echo "[INFO] Dry run: credentials and launch graph only; no ROS node will start."
  roslaunch --nodes ucar_2026_competition \
    full_competition_legacy_stack_v1.launch \
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

# Remove only stale nodes owned by this adapter. Other legacy task nodes are untouched.
cleanup
sleep 0.25

# simple_navigator intentionally supports interactive map_goal_picker overrides.
# A stale override from an earlier RViz session must not redirect a competition run.
if rosparam list >/dev/null 2>&1; then
  rosparam delete /map_goal_picker/goal_x >/dev/null 2>&1 || true
  rosparam delete /map_goal_picker/goal_y >/dev/null 2>&1 || true
  rosparam delete /map_goal_picker/goal_yaw >/dev/null 2>&1 || true
fi

set +e
roslaunch ucar_2026_competition full_competition_legacy_stack_v1.launch \
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
