#!/bin/bash

# full_system_start.sh - 启动完整的二维码扫描和导航系统
# 包括：导航、摄像头、二维码检测、URL解析、任务处理

set -e

# ROS环境
source /opt/ros/noetic/setup.bash

# 设置工作区路径
WS_DIR="/home/ucar/instant_ws"
if [ -f "$WS_DIR/devel/setup.bash" ]; then
  source "$WS_DIR/devel/setup.bash"
  echo "✓ 加载工作区环境: $WS_DIR"
else
  echo "错误: 工作区环境文件不存在: $WS_DIR/devel/setup.bash"
  exit 1
fi

# 如果需要远程ROS master，请设置以下变量
# export ROS_MASTER_URI=http://192.168.1.7:11311
# export ROS_IP=192.168.1.7
# export ROS_HOSTNAME=192.168.1.7

# 检查摄像头设备
echo "检查摄像头设备..."
if [ ! -e "/dev/video0" ]; then
  echo "错误: 摄像头设备 /dev/video0 不存在！"
  echo "请检查摄像头硬件连接或驱动。"
  exit 1
else
  echo "✓ 摄像头设备 /dev/video0 存在"
fi

# 启动roscore（如果未运行）
if ! pgrep -x roscore > /dev/null; then
  echo "roscore 未运行，正在启动..."
  roscore > /tmp/full_system_roscore.log 2>&1 &
  sleep 3
  if ! pgrep -x roscore > /dev/null; then
    echo "错误: roscore 启动失败！"
    echo "查看日志: /tmp/full_system_roscore.log"
    exit 1
  fi
else
  echo "roscore 已在运行"
fi

# 启动导航系统
echo "启动导航系统..."
if ! rosnode list | grep -q "/move_base"; then
  roslaunch ucar_nav ucar_navigation.launch > /tmp/full_system_navigation.log 2>&1 &
  sleep 5
  if ! rosnode list | grep -q "/move_base"; then
    echo "错误: 导航系统启动失败！"
    echo "查看日志: /tmp/full_system_navigation.log"
    exit 1
  fi
  echo "✓ 导航系统启动成功"
else
  echo "导航系统已运行"
fi

# 启动摄像头节点
echo "启动摄像头节点..."
if ! rosnode list | grep -q "/ucar_camera"; then
  rosrun ucar_camera ucar_camera.py > /tmp/full_system_camera.log 2>&1 &
  sleep 2
  if ! rosnode list | grep -q "/ucar_camera"; then
    echo "错误: 摄像头节点启动失败！"
    echo "查看日志: /tmp/full_system_camera.log"
    exit 1
  fi
  echo "✓ 摄像头节点启动成功"
else
  echo "摄像头节点已运行"
fi

# 检查摄像头话题
echo "检查摄像头话题..."
if ! rostopic list | grep -q "/ucar_camera/image_raw"; then
  echo "错误: 摄像头话题 /ucar_camera/image_raw 不存在！"
  echo "摄像头节点可能启动失败或设备问题。"
  exit 1
else
  echo "✓ 摄像头话题 /ucar_camera/image_raw 存在"
fi
# sudo fuser -k /dev/video0

# 启动二维码识别节点
echo "启动二维码识别节点..."
if ! rosnode list | grep -q "/qr_code_scanner"; then
  rosrun test qr_node > /tmp/full_system_qr.log 2>&1 &
  sleep 2
  if ! rosnode list | grep -q "/qr_code_scanner"; then
    echo "错误: 二维码识别节点启动失败！"
    echo "查看日志: /tmp/full_system_qr.log"
    exit 1
  fi
  echo "✓ 二维码识别节点启动成功"
else
  echo "二维码识别节点已运行"
fi

URL_RESULT_DIR="$WS_DIR/url_result"
mkdir -p "$URL_RESULT_DIR"
QR_BAG_FILE="$URL_RESULT_DIR/qr_code_result_$(date +%Y%m%d_%H%M%S).bag"
echo "启动二维码结果话题录包..."
if ! pgrep -f "rosbag record.* /qr_code_result" > /dev/null; then
  rosbag record -O "$QR_BAG_FILE" /qr_code_result > /tmp/full_system_qr_node_result_bag.log 2>&1 &
  sleep 1
  if pgrep -f "rosbag record.* /qr_code_result" > /dev/null; then
    echo "✓ 正在录制 /qr_code_result 到: $QR_BAG_FILE"
  else
    echo "警告: /qr_code_result 录包进程可能启动失败"
    echo "查看日志: /tmp/full_system_qr_node_result_bag.log"
  fi
else
  echo "/qr_code_result 录包进程已运行"
fi
# 发送二维码识别开始指令
echo "发送二维码识别开始指令..."
rostopic pub --once /qr_node_start std_msgs/String "data: 'start!'"

# 启动URL解析器
echo "启动URL解析器..."
if ! rosnode list | grep -q "/qr_url_parser"; then
  rosrun test qr_url_parser.py > /tmp/full_system_url_parser.log 2>&1 &
  sleep 2
  if ! rosnode list | grep -q "/qr_url_parser"; then
    echo "错误: URL解析器启动失败！"
    echo "查看日志: /tmp/full_system_url_parser.log"
    exit 1
  fi
  echo "✓ URL解析器启动成功"
else
  echo "URL解析器已运行"
fi

# echo "启动任务处理器..."
# if ! rosnode list | grep -q "/qr_mission_novoice"; then
#   rosrun ucar_controller process_qr_test1.py > /tmp/full_system_processor.log 2>&1 &
#   sleep 2
#   if ! rosnode list | grep -q "/qr_mission_novoice"; then
#     echo "错误: 任务处理器启动失败！"
#     echo "查看日志: /tmp/full_system_processor.log"
#     exit 1
#   fi
#   echo "✓ 任务处理器启动成功"
# else
#   echo "任务处理器已运行"
# fi

echo ""
echo "=========================================="
echo "系统启动完成！"
echo "=========================================="
echo ""
echo "活跃节点："
rosnode list
echo ""
echo "活跃话题："
rostopic list | grep -E "(qr|url|camera|navigation)"
echo ""
echo "监控命令："
echo "  查看二维码检测: rostopic echo /qr_code_result"
echo "  查看URL解析结果: rostopic echo /qr_url_parsed"
echo "  查看导航状态: rostopic echo /move_base/status"
echo "  查看摄像头图像: rosrun image_view image_view image:=/ucar_camera/image_raw"
echo ""
echo "系统已就绪，等待二维码扫描..."