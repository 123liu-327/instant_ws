#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
避障模块
"""

import rospy
import math
import os
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#7月28日：移除了第一种策略，当判断障碍物出现时直接执行绕行操作
# 同时新建立一个ros的action，使得小车到达岔路口时能够执行环岛操作

def my_print(head, head_type='info', content=None):
    # 如果content参数为空，则将其赋值为空字符串
    if content is None:
        content = ''
    # 根据head_type参数的值，设置不同的背景色和文字颜色
    if head_type == 'info':
        bg = 42
        word = 38
    elif head_type == 'warn':
        bg = 43
        word = 31
    elif head_type == 'err':
        bg = 41
        word = 38
    elif head_type == 'data':
        bg = 47
        word = 30
    else:
        bg = 45
        word = 38
    # 打印带有不同颜色头部的信息
    print(f"\n\033[{bg};{word}m   {head}   \033[0m\n{content}\n")




class ObstacleAvoidance:
    def __init__(self,shared_state):
        rospy.init_node('obstacle_avoidance_node')

        self.shared_state = shared_state # 共享状态

        # imu回调函数msg的初始值
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        
        # 参数配置
        self.obstacle_distance = 0.30  # 30cm触发距离
        self.scan_angle = math.radians(45)  # 检测角度±65度
        self.body_length = 0.342  # 车身长度34.2cm
        self.lateral_speed = 0.40  # 横向速度m/s
        self.angular_speed = 0.8  # 旋转速度rad/s
        self.linear_speed = 0.5  # 前进速度m/s
        # 避障参数
        self.start_time = 0.0
        self.safe_distance = 0.42  # 平移距离（可调试）
        # 发布器
        self.status_pub = rospy.Publisher('/avoidance_status', Bool, queue_size=1)
        self.cmd_pub = rospy.Publisher('/avoidance_cmd', Twist, queue_size=4) # 向底盘仲裁控制发布避障指令
        # self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # 直接向小车底盘发布避障指令

        # IMU订阅器（用于获取当前朝向）
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.current_yaw = 0.0

        # 里程计订阅器
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_position = None

        # 雷达订阅器（初始时不激活）
        self.scan_sub = None
        self.enabled = False

       # 新增摄像头订阅者
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/ucar_camera/image_raw",  # 根据实际话题名称修改
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True
        )
        # 图像处理参数
        self.kernel_size = 3  # 形态学操作的核大小
        self.center_threshold = 7  # 中心位置允许的偏差（像素）

        # 是否正在避障
        self.is_avoiding = False

        # 图像处理相关标志
        self.image_received = False
        self.is_centered = False



        self.last_enabled_state = self.enabled
        
        rospy.loginfo("避障节点初始化完成，等待控制台激活信号...")

    def odom_callback(self, msg):
        """里程计回调函数，获取当前位置"""

        self.current_position = msg.pose.pose.position

    def imu_callback(self, msg):
        """IMU回调函数，获取多种信息"""
        
        # 1. 姿态信息 (Orientation) - 用于旋转控制
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw
        self.current_roll = roll
        self.current_pitch = pitch


    def check_activation(self):
        """检查控制台激活状态"""
        try:
            # 从参数服务器获取控制台状态
            current_state = rospy.get_param('/enable_avoidance', False)
            
            if current_state != self.enabled:
                self.enabled = current_state
                if self.enabled:
                    rospy.loginfo("避障功能已激活，开始订阅激光雷达数据")
                    self.start_time = time.time()
                    if self.scan_sub is None:
                        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
                else:
                    rospy.loginfo("避障功能已停用")
                    if self.scan_sub is not None:
                        self.scan_sub.unregister()
                        self.scan_sub = None
        except Exception as e:
            rospy.logerr("获取控制台状态失败: %s", str(e))

    def scan_callback(self, msg):
        if self.is_avoiding or not self.enabled: # 如果正在避障或者开启标志还未传达，则不处理新的激光雷达数据不进行避障
            return
#        rospy.loginfo("激光雷达数据已接收，开始判断是否需要避障")
        # 获取有效数据
        ranges = list(msg.ranges)
        valid_ranges = []
        valid_angles = []

        # 计算有效角度范围
        for i in range(len(ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            # 角度归一化到[-π, π]
            if angle > math.pi:
                angle -= 2 * math.pi
            elif angle < -math.pi:
                angle += 2 * math.pi
                
            if -self.scan_angle <= angle <= self.scan_angle:
                if msg.range_min < ranges[i] < msg.range_max:
                    valid_ranges.append(ranges[i])
                    valid_angles.append(angle)

        # 如果没有有效数据，则返回
        if not valid_ranges: 
#            rospy.loginfo("前方没有有效数据，无需避障，程序返回")
            return

        # 如果有障碍物，则避障
#        rospy.loginfo("得到有效数据，开始判断是否需要避障")
        # 找到最小距离和对应角度
        min_distance = min(valid_ranges)
        min_index = valid_ranges.index(min_distance)
        obstacle_angle = valid_angles[min_index]

#        rospy.loginfo("障碍物距离: %f, 角度: %f", min_distance, obstacle_angle)

        if min_distance < self.obstacle_distance and time.time()-self.start_time > 8:
#            rospy.loginfo("障碍物距离小于设定值，开始避障")
            self.is_avoiding = True

            # 发布避障状态（切断巡线节点的发布）
            self.status_pub.publish(Bool(True))
            # 开始避障
            self.avoidance_maneuver(min_distance, obstacle_angle)

            self.is_avoiding = False

    def avoidance_maneuver(self, min_dist, obstacle_angle):
        try:
            # 其他状态时的常规避障流程
            my_print("执行常规避障策略")
            
            # Step 1: 转向正对障碍物
            rospy.loginfo("转向正对障碍物")
            self.rotate(obstacle_angle)  # 注意符号方向
            rospy.loginfo("转向正对障碍物完成，开始向左平移")
            
            # Step 2: 向左平移
            self.move_lateral(direction='left', distance=self.safe_distance)
            rospy.loginfo("向左平移完成，开始前进安全距离")
            
            # Step 3: 前进安全距离
            safe_distance = min_dist + 0.76 * self.body_length
            self.move_forward(safe_distance)
            rospy.loginfo("前进安全距离完成，开始向右平移复位")
            
            # Step 4: 向右平移复位（使用视觉反馈）
            rospy.loginfo("开始使用视觉反馈进行复位")
            self.move_lateral(direction='right', distance=0.75*self.safe_distance)
            
            if self.is_centered:
                rospy.loginfo("视觉反馈确认：已成功复位到赛道中央")
            else:
                rospy.logwarn("警告：未检测到理想的复位位置")

            # 避障完成，发布状态并退出
            self.status_pub.publish(Bool(False))
            my_print("避障任务完成，退出进程")
            rospy.delete_param('/enable_avoidance')

            os._exit(0)  # 强制退出当前进程

        except Exception as e:
            rospy.logerr("Avoidance error: %s" % str(e))
            # 发布避障失败状态
            self.status_pub.publish(Bool(False))

    def adaptive_threshold(self, img, block_size=13, down_value=15, low_value=0, high_value=255):
        """自适应阈值二值化处理（仅处理ROI区域）"""
        # 获取图像尺寸
        height, width = img.shape[:2]
        
        # 定义ROI区域（只处理第320行附近的区域）
        roi_height = 40  # 上下各20像素
        roi_y = max(0, 320 - roi_height//2)
        roi = img[roi_y:roi_y + roi_height, :]
        
        # 转为灰度图
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊
        blur = cv2.GaussianBlur(gray, (self.kernel_size, self.kernel_size), 0)
        
        # 自适应阈值 - 使用均值法
        binary = cv2.adaptiveThreshold(
            blur, high_value, 
            cv2.ADAPTIVE_THRESH_MEAN_C, 
            cv2.THRESH_BINARY_INV,  # 注意使用INV以得到黑色线条
            block_size, 
            down_value
        )
        
        # 形态学操作
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        dilated = cv2.dilate(closed, kernel, iterations=1)
        eroded = cv2.erode(dilated, kernel, iterations=1)
        
        # 返回处理后的ROI区域及其在原图中的位置
        return eroded, roi_y

    def check_center_position(self, binary_image, roi_y):
        """检查车辆是否在赛道中央"""
        try:
            # 获取ROI区域中对应原图第320行的像素值
            row_in_roi = 320 - roi_y
            if row_in_roi < 0 or row_in_roi >= binary_image.shape[0]:
                return False
                
            row = binary_image[row_in_roi, :]
            
            # 计算直方图（统计黑色像素的位置）
            histogram = np.where(row == 255)[0]
            
            if len(histogram) < 2:
                return False
                
            # 寻找左右两个峰值（赛道线）
            # 使用简单的聚类方法找到两个主要集群
            diff = np.diff(histogram)
            gaps = np.where(diff > 100)[0]  # 像素间隔大于20认为是不同的线
            
            if len(gaps) != 1:
                return False
                
            left_line = np.mean(histogram[:gaps[0]+1])
            right_line = np.mean(histogram[gaps[0]+1:])
            
            # 计算中心位置
            center = (left_line + right_line) / 2
            
            # 检查中心是否在图像中心附近（允许一定误差）
            image_center = binary_image.shape[1] / 2
            return abs(center - image_center) < self.center_threshold
            
        except Exception as e:
            rospy.logerr("中心位置检测错误: %s", str(e))
            return False

    def _image_callback(self, msg):
        """摄像头回调函数"""
        try:
            # 转换图像格式并缩放
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (640, 360))
            
            # 处理图像并检查是否居中
            binary_image, roi_y = self.adaptive_threshold(cv_image)
            self.is_centered = self.check_center_position(binary_image, roi_y)
            self.image_received = True
            
        except Exception as e:
            rospy.logerr("图像处理错误: %s", str(e))

    def rotate(self, target_angle, clockwise=None):
        """ 
        基于IMU反馈的旋转控制 
        
        :param target_angle: 目标旋转角度（弧度）
        :param clockwise: 旋转方向，True为顺时针，False为逆时针，None为根据角度符号自动判断
        """
        rospy.loginfo(f"需要旋转角度: {math.degrees(target_angle):.2f}°")
        
        # 获取当前角度
        start_yaw = self.current_yaw
        
        # 如果指定了旋转方向，则根据方向设置目标角度
        if clockwise is not None:
            # 将target_angle视为旋转的绝对角度值
            abs_angle = abs(target_angle)
            if clockwise:
                # 顺时针旋转（负方向）
                target_yaw = start_yaw - abs_angle
                rospy.loginfo(f"顺时针旋转 {math.degrees(abs_angle):.2f}°")
            else:
                # 逆时针旋转（正方向）
                target_yaw = start_yaw + abs_angle
                rospy.loginfo(f"逆时针旋转 {math.degrees(abs_angle):.2f}°")
        else:
            # 原有逻辑：根据角度符号自动判断方向
            target_yaw = start_yaw + target_angle

        # 标准化角度到[-π, π]
        target_yaw = self.normalize_angle(target_yaw)
        
        twist = Twist()
        
        # 持续旋转直到达到目标角度
        rate = rospy.Rate(20)  # 20Hz控制频率
        while not rospy.is_shutdown():
            # 计算当前角度误差
            error = self.normalize_angle(target_yaw - self.current_yaw)
            
            # 根据误差方向设置旋转速度
            if abs(error) < 0.03:  # 约2.9度的容差
                break
                
            # 接近目标时减速
            if abs(error) < 0.2:  # 约11.5度
                speed = 0.5 * self.angular_speed
            else:
                speed = self.angular_speed
                
            # 设置旋转方向
            twist.angular.z = math.copysign(speed, error)
            
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        self.stop()
        rospy.loginfo(f"旋转完成，当前角度: {math.degrees(self.current_yaw):.2f}°")


    def normalize_angle(self, angle):
        """ 将角度标准化到[-π, π] """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


    def move_lateral(self, direction, distance):
        """横向移动控制
        Args:
            direction: 移动方向 ('left' 或 'right')
            distance: 目标移动距离
        """
        # 等待里程计数据
        if self.current_position is None:
            rospy.logwarn("等待里程计数据...")
            rate = rospy.Rate(10)
            while self.current_position is None and not rospy.is_shutdown():
                rate.sleep()
            if self.current_position is None:
                rospy.logerr("无法获取里程计数据")
                return
        
        # 保存初始位置
        start_position = Point()
        start_position.x = self.current_position.x
        start_position.y = self.current_position.y
        
        # 创建速度命令
        cmd = Twist()
        base_speed = self.lateral_speed
        slow_speed = base_speed * 0.3  # 减速后的速度
        slowdown_distance = 0.20  # 开始减速的距离阈值（米）
        
        if direction == 'left':
            # 向左移动：完全依赖里程计
            cmd.linear.y = base_speed
            self.cmd_pub.publish(cmd)
            
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                # 计算已移动距离
                moved_distance = math.sqrt(
                    (self.current_position.x - start_position.x)**2 +
                    (self.current_position.y - start_position.y)**2
                )
                # 计算剩余距离
                remaining_distance = distance - moved_distance
                
                # 接近目标时减速
                if remaining_distance < slowdown_distance:
                    cmd.linear.y = slow_speed
                    self.cmd_pub.publish(cmd)
                
                if moved_distance >= distance:
                    break
                rate.sleep()
                
        else:
            # 向右移动：结合里程计和视觉反馈
            cmd.linear.y = -base_speed
            self.cmd_pub.publish(cmd)
            
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                # 计算已移动距离
                moved_distance = math.sqrt(
                    (self.current_position.x - start_position.x)**2 +
                    (self.current_position.y - start_position.y)**2
                )
                # 计算剩余距离
                remaining_distance = distance - moved_distance
                
                # 接近目标时减速
                if remaining_distance <= slowdown_distance:
                    cmd.linear.y = -slow_speed
                    self.cmd_pub.publish(cmd)
                
                # 检查是否达到目标距离
                if moved_distance >= distance:
                    # 达到目标距离后，检查视觉反馈
                    if self.image_received and self.is_centered:
                        rospy.loginfo("视觉反馈：已回到赛道中央")
                        break
                    else:
                        # 如果视觉反馈显示未居中，继续缓慢移动
                        cmd.linear.y = -slow_speed * 0.8  # 进一步减速
                        self.cmd_pub.publish(cmd)
                        rospy.loginfo("达到目标距离但未居中，继续微调")
                        
                # 如果收到视觉反馈且已居中，立即停止
                if self.image_received and self.is_centered:
                    rospy.loginfo("视觉反馈：已回到赛道中央")
                    break
                    
                rate.sleep()
        
        # 停止移动
        self.stop()

    def move_forward(self, distance):
        """ 基于里程计的前进控制 """
        rospy.loginfo(f"开始前进，距离: {distance}m")
        
        # 记录起始位置
        start_position = Point()
        start_position.x = self.current_position.x
        start_position.y = self.current_position.y
        
        twist = Twist()
        rate = rospy.Rate(20)  # 20Hz控制频率
        
        while not rospy.is_shutdown():
            # 计算已移动距离
            moved_distance = math.sqrt(
                (self.current_position.x - start_position.x)**2 +
                (self.current_position.y - start_position.y)**2
            )
            
            # 计算剩余距离
            remaining_distance = distance - moved_distance
            
            # 接近目标时减速
            if remaining_distance < 0.1:  # 10cm时开始减速
                speed = 0.3 * self.linear_speed
            else:
                speed = self.linear_speed
            
            twist.linear.x = speed
            
            # 当移动距离达到目标时停止
            if moved_distance >= distance:
                break
                
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.stop()

    def stop(self):
        """ 停止运动 """
        twist = Twist()
        self.cmd_pub.publish(twist)
        rospy.sleep(0.2)  # 确保停止命令生效

    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1Hz检查频率
        while not rospy.is_shutdown():
            self.check_activation()
            rate.sleep()

def escape_control_node(shared_state):
    """独立运行函数，供主进程调用"""
    server = ObstacleAvoidance(shared_state)
    rospy.spin()
