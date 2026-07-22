#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${INSTANT_WS:-/home/ucar/instant_ws}"
MODE="${REFERENCE_FOLLOW_MODE:-middle}"

case "$MODE" in
  left|middle|right) ;;
  *) echo "[ERROR] REFERENCE_FOLLOW_MODE must be left, middle, or right" >&2; exit 2 ;;
esac

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

PARKING_LOCK_ENTRY="$SCRIPT_DIR/start_subtask1_xunfei2026_complete_delivery_anchor_coverage_parking_lock_v2.sh"
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

if [[ ! -x "$PARKING_LOCK_ENTRY" ]]; then
  echo "[ERROR] Parking-lock total-flow entry is unavailable: $PARKING_LOCK_ENTRY" >&2
  exit 1
fi

echo "[INFO] New reference follower enabled: mode=$MODE"
echo "[INFO] Existing flow remains unchanged; takeover occurs after stop line + 25 cm."

set +e
"$PARKING_LOCK_ENTRY" "$@" \
  follow_after_stop_line:=true \
  follow_mode:="$MODE" \
  advance_before_follow_distance_m:=0.25 \
  follow_initial_turn_enabled:=true \
  follow_initial_turn_angle_deg:=60.0 \
  follow_initial_turn_pause_s:=0.5 &
FLOW_PID=$!

# Do not start the overlay before the old entry has completed stale-node cleanup.
# The handoff topic is latched, so no transition is lost while waiting.
(
  sleep 1.20
  # rosnode info can take longer than 0.35 s while OCR and TEB saturate the
  # controller.  Treating that RPC timeout as "node absent" made the follower
  # overlay give up even though the handoff node was running.  The master list
  # is cheaper and the wall-clock deadline matches the dual-stage watchdog.
  HANDOFF_WAIT_DEADLINE=$((SECONDS + 600))
  while (( SECONDS < HANDOFF_WAIT_DEADLINE )); do
    kill -0 "$FLOW_PID" 2>/dev/null || exit 0
    if timeout 1.20 rosnode list 2>/dev/null \
        | grep -Fxq /xunfei2026_simulation_handoff; then
      exec roslaunch iden_controller xunfei2026_reference_line_follow_takeover_v2.launch \
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
