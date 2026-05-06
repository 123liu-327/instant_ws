#!/bin/bash
# start_ros.sh - 启动 ROS 核心

echo "========================================"
echo "启动 ROS 环境..."
echo "========================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 设置工作区路径
WS_DIR="$HOME/instant_ws"

# 检查当前目录
if [ ! -d "$WS_DIR" ]; then
    echo -e "${RED}错误: 工作区目录不存在: $WS_DIR${NC}"
    exit 1
fi

echo -e "${YELLOW}1. 检查是否有运行的 ROS 进程...${NC}"

# 检查是否已有 roscore 在运行
if ps aux | grep -v grep | grep roscore > /dev/null; then
    echo -e "${RED}⚠ 发现 roscore 已在运行！${NC}"
    echo "现有进程:"
    ps aux | grep -v grep | grep roscore
    echo -e "${YELLOW}是否要终止并重启？(y/n)${NC}"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        echo "终止现有 roscore..."
        $WS_DIR/scripts/clean_ros.sh
        sleep 2
    else
        echo "使用现有 roscore..."
        exit 0
    fi
fi

echo -e "${YELLOW}2. 设置 ROS 环境变量...${NC}"

# 设置 ROS 环境
source /opt/ros/noetic/setup.bash

# 设置工作区
if [ -f "$WS_DIR/devel/setup.bash" ]; then
    source "$WS_DIR/devel/setup.bash"
    echo -e "${GREEN}✓ 加载工作区: $WS_DIR${NC}"
else
    echo -e "${YELLOW}⚠ 工作区未编译，使用系统 ROS${NC}"
fi

# 设置 ROS 变量


echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo "ROS_IP=$ROS_IP"

echo -e "${YELLOW}3. 检查网络配置...${NC}"

# 检查 localhost 解析
if ping -c 1 localhost > /dev/null 2>&1; then
    echo -e "${GREEN}✓ localhost 可访问${NC}"
else
    echo -e "${RED}⚠ 无法访问 localhost${NC}"
    # 尝试修复
    echo "127.0.0.1 localhost" | sudo tee -a /etc/hosts
fi

# 检查主机名
HOSTNAME=$(hostname)
if grep -q "$HOSTNAME" /etc/hosts; then
    echo -e "${GREEN}✓ 主机名 $HOSTNAME 已在 /etc/hosts 中${NC}"
else
    echo -e "${YELLOW}⚠ 主机名 $HOSTNAME 不在 /etc/hosts 中${NC}"
    echo "127.0.0.1 $HOSTNAME" | sudo tee -a /etc/hosts
fi

echo -e "${YELLOW}4. 启动 roscore...${NC}"

# 创建日志目录
mkdir -p ~/.ros/log

# 启动 roscore 并捕获输出
echo "roscore 启动中..."
echo "日志文件: ~/.ros/log/latest/roslaunch-*.log"

# 在后台启动 roscore
roscore > /tmp/roscore_output.log 2>&1 &

# 获取进程 ID
ROS_PID=$!
echo "roscore PID: $ROS_PID"

# 等待 roscore 启动
echo -n "等待 roscore 启动"
for i in {1..10}; do
    if rostopic list > /dev/null 2>&1; then
        echo -e "\n${GREEN}✓ roscore 启动成功！${NC}"
        break
    fi
    echo -n "."
    sleep 1
done

# 检查是否启动成功
if rostopic list > /dev/null 2>&1; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}ROS 启动成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    
    echo -e "${YELLOW}ROS 状态检查:${NC}"
    echo "1. 进程状态:"
    ps aux | grep -v grep | grep roscore
    
    echo -e "\n2. 节点列表:"
    rosnode list
    
    echo -e "\n3. 主题列表:"
    rostopic list
    
    echo -e "\n4. 参数列表:"
    rosparam list
    
    # 保存 PID 到文件，方便后续管理
    echo $ROS_PID > /tmp/roscore_pid.txt
    echo -e "\n${YELLOW}roscore PID 已保存到 /tmp/roscore_pid.txt${NC}"
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}roscore 启动失败！${NC}"
    echo -e "${RED}========================================${NC}"
    echo "查看日志: /tmp/roscore_output.log"
    cat /tmp/roscore_output.log
    exit 1
fi