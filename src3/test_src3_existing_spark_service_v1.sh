#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SRC3="$WS/src3"
CRED_FILE="$WS/src/iden_controller/config/spark_credentials.env"

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
source "$SRC3/devel_pure/setup.bash"
export PATH="$SRC3/devel_pure/bin:/opt/ros/noetic/bin:/home/ucar/miniconda3/bin:/usr/local/bin:/usr/bin:/bin"
set -u
set -a
source "$CRED_FILE"
set +a

export ROS_MASTER_URI="http://127.0.0.1:11319"
master_pid=""
node_pid=""
cleanup() {
  set +e
  [[ -n "$node_pid" ]] && kill "$node_pid" >/dev/null 2>&1
  [[ -n "$master_pid" ]] && kill "$master_pid" >/dev/null 2>&1
  wait >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

roscore -p 11319 >/tmp/src3_spark_selftest_roscore.log 2>&1 &
master_pid=$!
sleep 2
rosrun ucar_2026_smart_factory_llm reason_pickup_existing_spark_v1.py \
  __name:=smart_factory_llm >/tmp/src3_spark_selftest_node.log 2>&1 &
node_pid=$!
service_ready=0
for _ in $(seq 1 80); do
  if rosservice list 2>/dev/null | grep -qx "/smart_factory_llm/reason_pickup_order"; then
    service_ready=1
    break
  fi
  sleep 0.1
done
if [[ "$service_ready" != "1" ]]; then
  echo "[ERROR] Spark adapter service did not become ready." >&2
  cat /tmp/src3_spark_selftest_node.log >&2 || true
  exit 1
fi

result="$(rosservice call /smart_factory_llm/reason_pickup_order \
  "item_a: '香蕉'
item_b: '毛巾'
item_c: '电脑'
voice_instruction: '现实环境取得食品大类，仿真环境取得电子产品大类'" 2>&1)"
printf '%s\n' "$result"
grep -q "success: True" <<<"$result"
echo "[PASS] Existing Spark KEY:SECRET service adapter returned a valid result."
