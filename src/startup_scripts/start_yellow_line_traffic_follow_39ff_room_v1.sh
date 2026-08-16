#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SOURCE_ROOT="$WS/src"
PURE_DEVEL="$WS/devel_src_pure_runtime_v3"

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash

if [[ ! -f "$PURE_DEVEL/setup.bash" ]]; then
  echo "[ERROR] Missing pure runtime build: $PURE_DEVEL/setup.bash" >&2
  echo "[ERROR] Run $SOURCE_ROOT/startup_scripts/build_src_pure_runtime_v3.sh first." >&2
  exit 1
fi
source "$PURE_DEVEL/setup.bash"
export PATH="$PURE_DEVEL/bin:/opt/ros/noetic/bin:/home/ucar/miniconda3/bin:/usr/local/bin:/usr/bin:/bin"
export PYTHONUNBUFFERED=1
set -u

required_packages=(
  ucar_2026_competition
  ucar_2026_competition_speech
  ucar_2026_strict_mission
  traffic_light_vision
  flow_end
  ucar_camera
  usb_cam
  ucar_nav
  ucar_map
  ydlidar
)
for package in "${required_packages[@]}"; do
  if ! rospack find "$package" >/dev/null 2>&1; then
    echo "[ERROR] Required ROS package is unavailable: $package" >&2
    exit 1
  fi
done

LAUNCH_FILE="$SOURCE_ROOT/ucar_2026_competition/launch/task4_task5_current_front_39ff_room_v1.launch"
if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "[ERROR] Missing launch file: $LAUNCH_FILE" >&2
  exit 1
fi

if rosnode list >/dev/null 2>&1; then
  active_conflicts="$(rosnode list 2>/dev/null | grep -E '^/(competition_flow|strict_mission|follow_test|traffic_light_detector|ucar_camera|usb_cam)$' || true)"
  if [[ -n "$active_conflicts" ]]; then
    echo "[ERROR] Refusing to start while front-flow nodes are already active:" >&2
    echo "$active_conflicts" >&2
    exit 1
  fi
fi

if fuser /dev/video0 >/dev/null 2>&1; then
  echo "[ERROR] /dev/video0 is already occupied; refusing camera takeover." >&2
  fuser -v /dev/video0 >&2 || true
  exit 1
fi

echo "[INFO] Front-stage flow: yellow-line centering -> camera switch -> traffic recognition -> line following."
echo "[INFO] Launch starts stationary. Call /competition/start when the area is clear."

if [[ "${FLOW_DRY_RUN:-0}" == "1" ]]; then
  roslaunch --nodes ucar_2026_competition \
    task4_task5_current_front_39ff_room_v1.launch "$@"
  exit 0
fi

set +e
roslaunch ucar_2026_competition \
  task4_task5_current_front_39ff_room_v1.launch "$@" 2>&1 | sed -u \
    -e '/check crc16 faild(imu)/d' \
    -e '/check crc16 faild(ahrs)/d' \
    -e '/head_len error (imu)/d'
status=${PIPESTATUS[0]}
set -e
exit "$status"
