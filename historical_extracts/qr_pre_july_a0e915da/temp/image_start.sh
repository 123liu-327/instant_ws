#!/bin/bash

# 启动摄像头和二维码识别节点。
# 如果摄像头设备被旧进程占用，先结束旧进程，再重新启动摄像头节点。

set -e

CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
CAMERA_NODE="/ucar_camera"
CAMERA_TOPIC="/ucar_camera/image_raw"
CAMERA_LOG="/tmp/image_start_camera.log"

QR_NODE="/qr_code_scanner"
QR_LOG="/tmp/image_start_qr.log"

ROSCORE_LOG="/tmp/image_start_roscore.log"
WS_DIR="${WS_DIR:-/home/ucar/instant_ws}"

source /opt/ros/noetic/setup.bash

if [ -f "$WS_DIR/devel/setup.bash" ]; then
  source "$WS_DIR/devel/setup.bash"
  echo "已加载工作空间: $WS_DIR"
else
  echo "错误: 找不到工作空间环境文件: $WS_DIR/devel/setup.bash"
  exit 1
fi

# 如果需要连接远程 ROS master，请取消注释并修改下面的地址。
# export ROS_MASTER_URI=http://192.168.1.7:11311
# export ROS_IP=192.168.1.7
# export ROS_HOSTNAME=192.168.1.7

camera_pids() {
  local device="$1"

  {
    if command -v fuser >/dev/null 2>&1; then
      fuser "$device" 2>/dev/null || true
    fi
    if command -v lsof >/dev/null 2>&1; then
      lsof -t "$device" 2>/dev/null || true
    fi
  } | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u || true
}

release_camera_device() {
  local device="$1"
  local pids

  pids="$(camera_pids "$device")"
  if [ -z "$pids" ]; then
    echo "摄像头设备未被占用: $device"
    return 0
  fi

  echo "摄像头设备被占用: $device"
  echo "正在结束占用摄像头的旧进程 PID: $pids"
  for pid in $pids; do
    if [ "$pid" != "$$" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  sleep 1
  pids="$(camera_pids "$device")"
  if [ -n "$pids" ]; then
    echo "普通结束失败，正在强制结束占用进程 PID: $pids"
    for pid in $pids; do
      if [ "$pid" != "$$" ] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
      fi
    done
    sleep 1
  fi

  pids="$(camera_pids "$device")"
  if [ -n "$pids" ]; then
    echo "错误: 摄像头设备仍然被占用: $device PID: $pids"
    exit 1
  fi
}

echo "检查摄像头设备..."
if [ ! -e "$CAMERA_DEVICE" ]; then
  echo "错误: 摄像头设备不存在: $CAMERA_DEVICE"
  echo "请检查摄像头连接或驱动。"
  exit 1
fi
echo "摄像头设备存在: $CAMERA_DEVICE"

if ! pgrep -x roscore >/dev/null; then
  echo "roscore 未运行，正在启动..."
  roscore > "$ROSCORE_LOG" 2>&1 &
  sleep 3

  if ! pgrep -x roscore >/dev/null; then
    echo "错误: roscore 启动失败。"
    echo "日志: $ROSCORE_LOG"
    exit 1
  fi
else
  echo "roscore 已在运行。"
fi

if rosnode list | grep -q "^${CAMERA_NODE}$"; then
  echo "检测到旧摄像头节点，重启前先关闭: $CAMERA_NODE"
  rosnode kill "$CAMERA_NODE" >/dev/null 2>&1 || true
  sleep 1
fi

release_camera_device "$CAMERA_DEVICE"

echo "正在启动摄像头节点..."
rosrun ucar_camera ucar_camera.py > "$CAMERA_LOG" 2>&1 &
sleep 2

if ! rosnode list | grep -q "^${CAMERA_NODE}$"; then
  echo "错误: 摄像头节点启动失败。"
  echo "日志: $CAMERA_LOG"
  exit 1
fi
echo "摄像头节点启动成功: $CAMERA_NODE"

echo "检查摄像头图像话题..."
if ! rostopic list | grep -q "^${CAMERA_TOPIC}$"; then
  echo "错误: 摄像头图像话题不存在: $CAMERA_TOPIC"
  echo "摄像头节点可能启动失败，或设备不可用。"
  echo "日志: $CAMERA_LOG"
  exit 1
fi
echo "摄像头图像话题存在: $CAMERA_TOPIC"

if ! rosnode list | grep -q "^${QR_NODE}$"; then
  echo "正在启动二维码识别节点..."
  rosrun test qr_node > "$QR_LOG" 2>&1 &
  sleep 2

  if ! rosnode list | grep -q "^${QR_NODE}$"; then
    echo "错误: 二维码识别节点启动失败。"
    echo "日志: $QR_LOG"
    exit 1
  fi
  echo "二维码识别节点启动成功: $QR_NODE"
else
  echo "二维码识别节点已在运行: $QR_NODE"
fi

echo "正在发送二维码识别开始指令..."
rostopic pub --once /qr_node_start std_msgs/String "data: 'start!'"

echo "启动完成。"
echo "摄像头图像话题: $CAMERA_TOPIC"
echo "二维码识别结果话题: /qr_code_result"