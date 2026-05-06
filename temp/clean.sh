#!/bin/bash
# clean_ros.sh - 清理所有 ROS 相关进程

echo "========================================"
echo "开始清理 ROS 进程和端口..."
echo "========================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}1. 终止 ROS 核心进程...${NC}"

# 终止所有已知的 ROS 进程
killall -9 roscore 2>/dev/null
killall -9 rosmaster 2>/dev/null
killall -9 rosout 2>/dev/null
killall -9 roslaunch 2>/dev/null
killall -9 rospack 2>/dev/null

# 终止所有 Python ROS 相关进程
pkill -f "/opt/ros/noetic/bin/roscore" 2>/dev/null
pkill -f "/opt/ros/noetic/lib/rosmaster" 2>/dev/null
pkill -f "/rosout" 2>/dev/null

echo -e "${YELLOW}2. 释放 ROS 端口...${NC}"

# 释放 ROS 相关端口
sudo fuser -k 11311/tcp 2>/dev/null  # ROS Master
sudo fuser -k 11312/tcp 2>/dev/null  # 如果使用了其他端口

echo -e "${YELLOW}3. 清理临时文件...${NC}"

# 清理 ROS 日志
if [ -d ~/.ros/log ]; then
    echo "清理 ROS 日志目录..."
    rm -rf ~/.ros/log/*
fi

# 可选：清理 core dump 文件
if [ -d ~/.ros/coredump ]; then
    echo "清理 core dump 文件..."
    rm -rf ~/.ros/coredump/*
fi

echo -e "${YELLOW}4. 验证清理结果...${NC}"

# 检查是否还有 ROS 进程
echo "=== 检查剩余 ROS 进程 ==="
remaining=$(ps aux | grep -E "ros|master|roscore" | grep -v grep | grep -v clean_ros)
if [ -z "$remaining" ]; then
    echo -e "${GREEN}✓ 没有发现 ROS 进程${NC}"
else
    echo -e "${RED}⚠ 发现以下 ROS 进程:${NC}"
    echo "$remaining"
    echo -e "${YELLOW}是否需要强制终止？(y/n)${NC}"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        echo "强制终止剩余进程..."
        pkill -9 -f "ros" 2>/dev/null
    fi
fi

# 检查端口是否释放
echo "=== 检查端口占用 ==="
if sudo lsof -i :11311 > /dev/null 2>&1; then
    echo -e "${RED}⚠ 端口 11311 仍被占用${NC}"
    sudo lsof -i :11311
else
    echo -e "${GREEN}✓ 端口 11311 已释放${NC}"
fi

echo "========================================"
echo -e "${GREEN}清理完成！可以安全启动新的 roscore${NC}"
echo "========================================"