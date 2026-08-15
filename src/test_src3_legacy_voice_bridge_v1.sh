#!/usr/bin/env bash
set -eo pipefail

SRC3=/home/ucar/instant_ws/src3
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
source "$SRC3/devel_pure/setup.bash"
export PATH="$SRC3/devel_pure/bin:/opt/ros/noetic/bin:/home/ucar/miniconda3/bin:/usr/local/bin:/usr/bin:/bin"
export ROS_MASTER_URI=http://127.0.0.1:11320

master_pid=""
node_pid=""
cleanup() {
  set +e
  [[ -n "$node_pid" ]] && kill "$node_pid" >/dev/null 2>&1
  [[ -n "$master_pid" ]] && kill "$master_pid" >/dev/null 2>&1
  wait >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

roscore -p 11320 >/tmp/src3_voice_bridge_test_roscore.log 2>&1 &
master_pid=$!
sleep 2
rosrun ucar_2026_competition src3_legacy_voice_bridge_v1.py \
  __name:=src3_legacy_voice_bridge >/tmp/src3_voice_bridge_test_node.log 2>&1 &
node_pid=$!
sleep 1

/home/ucar/miniconda3/bin/python3 - <<'PY'
import time

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

rospy.init_node("src3_voice_bridge_protocol_test", anonymous=True)
wakeups = []
questions = []
accepted = []
rospy.Subscriber("/wakeup", String, lambda msg: wakeups.append(msg.data))
rospy.Subscriber("/question", String, lambda msg: questions.append(msg.data))
rospy.Subscriber(
    "/factory/voice_command_accepted", Bool, lambda msg: accepted.append(msg.data)
)
raw = rospy.Publisher("/factory/voice_raw_text", String, queue_size=5)
deadline = time.time() + 2.0
while raw.get_num_connections() == 0 and time.time() < deadline:
    time.sleep(0.02)

raw.publish(String(data="小飞小飞"))
raw.publish(String(data="小飞小飞"))
time.sleep(0.25)
assert len(wakeups) == 1, wakeups

rospy.wait_for_service("/speech_command_node/start_listening", timeout=2.0)
start = rospy.ServiceProxy("/speech_command_node/start_listening", Trigger)
stop = rospy.ServiceProxy("/speech_command_node/stop_listening", Trigger)
assert start().success
raw.publish(String(data="我在"))
raw.publish(String(data="现实环境取得食品大类，仿真环境取得电子产品大类"))
time.sleep(0.25)
assert questions == ["现实环境取得食品大类，仿真环境取得电子产品大类"], questions
assert stop().success
time.sleep(0.15)
assert accepted and accepted[-1] is True, accepted
print("VOICE_BRIDGE_PROTOCOL_OK")
PY
