#!/bin/bash

# 启动摄像头和二维码识别节点
# 特点：
# 1. 自动启动 roscore
# 2. 自动释放 /dev/video0
# 3. 支持节点名带后缀，例如 /ucar_camera_12345
# 4. 启动后等待节点和话题出现
# 5. 日志统一写入 /tmp

set -euo pipefail

CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
CAMERA_NODE_PREFIX="${CAMERA_NODE_PREFIX:-/ucar_camera}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/ucar_camera/image_raw}"
CAMERA_LOG="${CAMERA_LOG:-/tmp/image_start_camera.log}"

QR_NODE_PREFIX="${QR_NODE_PREFIX:-/qr_code_scanner}"
QR_START_TOPIC="${QR_START_TOPIC:-/qr_node_start}"
QR_RESULT_TOPIC="${QR_RESULT_TOPIC:-/qr_code_result}"
QR_LOG="${QR_LOG:-/tmp/image_start_qr.log}"

ROSCORE_LOG="${ROSCORE_LOG:-/tmp/image_start_roscore.log}"
WS_DIR="${WS_DIR:-/home/ucar/instant_ws}"

WAIT_TIMEOUT="${WAIT_TIMEOUT:-8}"

echo_info() {
  echo -e "\033[32m[INFO]\033[0m $*"
}

echo_warn() {
  echo -e "\033[33m[WARN]\033[0m $*"
}

echo_error() {
  echo -e "\033[31m[ERROR]\033[0m $*"
}

load_ros_env() {
  source /opt/ros/noetic/setup.bash

  if [ -f "$WS_DIR/devel/setup.bash" ]; then
    source "$WS_DIR/devel/setup.bash"
    echo_info "已加载工作空间: $WS_DIR"
  else
    echo_error "找不到工作空间环境文件: $WS_DIR/devel/setup.bash"
    exit 1
  fi
}

ros_master_alive() {
  rosnode list >/dev/null 2>&1
}

start_roscore_if_needed() {
  if ros_master_alive; then
    echo_info "roscore 已在运行。"
    return 0
  fi

  echo_warn "roscore 未运行，正在启动..."
  roscore > "$ROSCORE_LOG" 2>&1 &

  for i in $(seq 1 "$WAIT_TIMEOUT"); do
    sleep 1
    if ros_master_alive; then
      echo_info "roscore 启动成功。"
      return 0
    fi
  done

  echo_error "roscore 启动失败。日志: $ROSCORE_LOG"
  exit 1
}

# 匹配节点名前缀，支持：
# /ucar_camera
# /ucar_camera_12345
# /ucar_camera_xxx
ros_nodes_by_prefix() {
  local prefix="$1"
  rosnode list 2>/dev/null | grep -E "^${prefix}($|_)" || true
}

kill_nodes_by_prefix() {
  local prefix="$1"
  local nodes

  nodes="$(ros_nodes_by_prefix "$prefix")"

  if [ -z "$nodes" ]; then
    echo_info "未检测到旧节点: ${prefix}*"
    return 0
  fi

  echo_warn "检测到旧节点，准备关闭:"
  echo "$nodes"

  for node in $nodes; do
    rosnode kill "$node" >/dev/null 2>&1 || true
  done

  sleep 1

  nodes="$(ros_nodes_by_prefix "$prefix")"
  if [ -n "$nodes" ]; then
    echo_warn "部分节点未正常关闭，继续等待:"
    echo "$nodes"
    sleep 1
  fi
}

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
    echo_info "摄像头设备未被占用: $device"
    return 0
  fi

  echo_warn "摄像头设备被占用: $device"
  echo_warn "正在结束占用摄像头的旧进程 PID:"
  echo "$pids"

  for pid in $pids; do
    if [ "$pid" != "$$" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  sleep 1

  pids="$(camera_pids "$device")"
  if [ -n "$pids" ]; then
    echo_warn "普通结束失败，强制结束 PID:"
    echo "$pids"

    for pid in $pids; do
      if [ "$pid" != "$$" ] && kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
      fi
    done

    sleep 1
  fi

  pids="$(camera_pids "$device")"
  if [ -n "$pids" ]; then
    echo_error "摄像头设备仍然被占用: $device"
    echo_error "占用 PID: $pids"
    exit 1
  fi

  echo_info "摄像头设备已释放: $device"
}

wait_for_node_prefix() {
  local prefix="$1"
  local timeout="$2"

  for i in $(seq 1 "$timeout"); do
    if [ -n "$(ros_nodes_by_prefix "$prefix")" ]; then
      echo_info "节点已启动: ${prefix}*"
      ros_nodes_by_prefix "$prefix"
      return 0
    fi
    sleep 1
  done

  echo_error "等待节点超时: ${prefix}*"
  return 1
}

wait_for_topic() {
  local topic="$1"
  local timeout="$2"

  for i in $(seq 1 "$timeout"); do
    if rostopic list 2>/dev/null | grep -q "^${topic}$"; then
      echo_info "话题已存在: $topic"
      return 0
    fi
    sleep 1
  done

  echo_error "等待话题超时: $topic"
  return 1
}

check_camera_device() {
  echo_info "检查摄像头设备..."

  if [ ! -e "$CAMERA_DEVICE" ]; then
    echo_error "摄像头设备不存在: $CAMERA_DEVICE"
    echo_error "请检查摄像头连接或驱动。"
    exit 1
  fi

  echo_info "摄像头设备存在: $CAMERA_DEVICE"
}

start_camera_node() {
  echo_info "关闭旧摄像头节点..."
  kill_nodes_by_prefix "$CAMERA_NODE_PREFIX"

  release_camera_device "$CAMERA_DEVICE"

  echo_info "正在启动摄像头节点..."
  rm -f "$CAMERA_LOG"

  rosrun ucar_camera ucar_camera.py > "$CAMERA_LOG" 2>&1 &

  if ! wait_for_node_prefix "$CAMERA_NODE_PREFIX" "$WAIT_TIMEOUT"; then
    echo_error "摄像头节点启动失败。日志: $CAMERA_LOG"
    tail -n 50 "$CAMERA_LOG" || true
    exit 1
  fi

  if ! wait_for_topic "$CAMERA_TOPIC" "$WAIT_TIMEOUT"; then
    echo_error "摄像头图像话题不存在: $CAMERA_TOPIC"
    echo_error "日志: $CAMERA_LOG"
    tail -n 50 "$CAMERA_LOG" || true
    exit 1
  fi

  echo_info "摄像头节点启动成功。"
}

start_qr_node() {
  local qr_nodes

  qr_nodes="$(ros_nodes_by_prefix "$QR_NODE_PREFIX")"

  if [ -n "$qr_nodes" ]; then
    echo_info "二维码识别节点已在运行:"
    echo "$qr_nodes"
  else
    echo_info "正在启动二维码识别节点..."
    rm -f "$QR_LOG"

    rosrun test qr_node > "$QR_LOG" 2>&1 &

    if ! wait_for_node_prefix "$QR_NODE_PREFIX" "$WAIT_TIMEOUT"; then
      echo_error "二维码识别节点启动失败。日志: $QR_LOG"
      tail -n 50 "$QR_LOG" || true
      exit 1
    fi

    echo_info "二维码识别节点启动成功。"
  fi
}

send_qr_start() {
  echo_info "正在发送二维码识别开始指令..."

  rostopic pub --once "$QR_START_TOPIC" std_msgs/String "data: 'start!'" >/dev/null

  echo_info "二维码识别开始指令已发送: $QR_START_TOPIC"
}

main() {
  load_ros_env
  start_roscore_if_needed
  check_camera_device
  start_camera_node
  start_qr_node
  send_qr_start

  echo_info "启动完成。"
  echo_info "摄像头图像话题: $CAMERA_TOPIC"
  echo_info "二维码开始话题: $QR_START_TOPIC"
  echo_info "二维码结果话题: $QR_RESULT_TOPIC"
  echo_info "摄像头日志: $CAMERA_LOG"
  echo_info "二维码日志: $QR_LOG"
}

main "$@"