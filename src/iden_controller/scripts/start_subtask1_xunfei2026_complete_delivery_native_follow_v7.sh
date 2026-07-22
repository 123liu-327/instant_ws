#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${INSTANT_WS:-/home/ucar/instant_ws}"
MODE="${REFERENCE_FOLLOW_MODE:-middle}"
case "$MODE" in left|middle|right) ;; *) echo "[ERROR] invalid mode: $MODE" >&2; exit 2 ;; esac
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

# Parking/navigation remain on the corrected v6 full flow.  Only the final
# line-follow overlay is replaced by the local implementation in this file set.
ENTRY="$SCRIPT_DIR/start_subtask1_xunfei2026_complete_delivery_src2_frame_center_v6.sh"
FLOW_PID=""
OVERLAY_PID=""
cleanup() {
  set +e
  if [[ -n "$FLOW_PID" ]] && kill -0 "$FLOW_PID" 2>/dev/null; then
    kill -TERM "$FLOW_PID" 2>/dev/null || true
    wait "$FLOW_PID" 2>/dev/null || true
  fi
  timeout 0.7 rosnode kill /reference_line_follow_takeover_v2 >/dev/null 2>&1 || true
  if [[ -n "$OVERLAY_PID" ]] && kill -0 "$OVERLAY_PID" 2>/dev/null; then
    kill -TERM "$OVERLAY_PID" 2>/dev/null || true
    wait "$OVERLAY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"$ENTRY" "$@" \
  follow_after_stop_line:=true follow_mode:="$MODE" \
  advance_before_follow_distance_m:=0.25 \
  follow_initial_turn_enabled:=true \
  follow_initial_turn_angle_deg:=60.0 \
  follow_initial_turn_pause_s:=0.5 &
FLOW_PID=$!

(
  sleep 1.20
  DEADLINE=$((SECONDS + 600))
  while (( SECONDS < DEADLINE )); do
    kill -0 "$FLOW_PID" 2>/dev/null || exit 0
    if timeout 1.20 rosnode list 2>/dev/null \
        | grep -Fxq /xunfei2026_simulation_handoff; then
      exec roslaunch iden_controller xunfei2026_native_line_follow_takeover_v1.launch \
        mode:="$MODE"
    fi
    sleep 0.25
  done
  echo "[ERROR] Timed out waiting for simulation handoff node" >&2
) &
OVERLAY_PID=$!
wait "$FLOW_PID"
STATUS=$?
FLOW_PID=""
exit "$STATUS"
