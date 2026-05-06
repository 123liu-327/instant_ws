#!/usr/bin/env python3
import sys
import os
ros_python_path = "/opt/ros/noetic/lib/python3/dist-packages"
if ros_python_path not in sys.path:
    sys.path.append(ros_python_path)
os.environ["PYTHONPATH"] = ":".join(sys.path)  # 确保子进程继承
import cv2
import numpy as np
import rospy
import time
from cv_bridge import CvBridge 
import math

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String

import signal
from multiprocessing import Event, Value, Lock, Process
import threading
import os
import multiprocessing
from followline_service_control import run_control_node

#=============================================================================
## 6月3号改动： 
## 在calculate_speed类里面新添加了停车控制器、恢复模式旋转器、检查车道线是否失效函数
## 在process_frame中改了流程：图像检查->算法执行->检查车道线有效性->检查是否有入库角点->分情况进行速度计算并发布指令（新添加检查车道线和车库角点）

## 6月23号改动：
## 在迷宫法巡线增加了断点时的处理，解决了即将进入车库时的图像断点线问题
## 以及改动了角点检测参数，增强了检测的鲁棒性
## 改动了车库出现检测函数，增加了对车库角点的检测，会优先选择下方的点对
## 另外还改动了车库控制器，当出现三个及以上的车库点是只会选择下面的两个点作为前视点进行入库


## 6月24号改动：
## 改动了停车控制器，不再根据角点来判断是否停车，而是检测前方车库上边界与基准点的距离来判断是否停车，即detect_white_line_distance函数
## 把避障class打包到了另一个py文件，在开头使用import导入

## 6月25号改动：
## 打算修改巡线算法的逻辑，因为当在环岛遇上障碍物时车子要掉头环岛，但是由于车子巡线模式未切换，所以会导致车子在环岛内绕圈
## 所以打算在巡线节点node的主循环run里面监测环岛状态，看情况切换巡线模式

## 6月30号改动：
## 弃用了之前监测环岛状态的方法，采用ros参数服务器全局参量escape_mode告知避障进程选择避障方式：掉头环岛 or 绕路前行

## 7月26号改动：
## 如果当障碍物出现在环岛出口时，小车先调头，再用固定速度固定时间绕行环岛，绕行结束后再继续巡线

## 7月28号改动：
## 按照要求写了新的roundabout.py并创建了一个action服务端，在巡线进程里自主选择是否需要环岛，如果需要则调用roundabout.py里的服务端进行环岛（已丢弃）

## 7月29号改动：
## 打算在避障完成之后将速度调低，方便入库

## 7月31号调试
## 参数一：速度为0.32，前视距离设置为110，岔路口参数设置为直行0.8s（速度为0.20），旋转1.2s（速度为0.98）比较稳定

## 8月1号改动：
## 为了解决环岛一周的问题：添加FINAL_MODE = 1，当FINAL_MODE = 1时，小车会环岛一周，否则不会环岛
## 弃用了roundabout.py
## 改为：当小车到达岔路口（双线第一次全丢时）小车直行，然后旋转一点点后恢复巡线，然后巡线会到达下一个岔路口（此时单线丢失，且另一条线很直），小车直接多旋转一点后再直行
## 这样小车又会恢复到巡线，接着会再次到达第三个岔路口（双线会再次全丢），小车直行，然后旋转后再次恢复巡线，同时关闭环岛开关，小车则会继续巡线进入终点
## 准备改动特殊控制的函数结构

## 8月2号改动：
## 新增了def _round_insection(leftx, lefty, rightx, righty, curvature, mode)函数用于监控环岛过程的标志位
## 改动了特殊控制的函数，一个是_handle_roundabout_situation函数，一个_handle_lost_line_situation函数

## 8月3号改动：
## 改变了LaneNODE的结构，使用_check_lane_valid和def _round_insection两个检查函数来确定小车到达的位置
## 新增了一个变量intersection_flag = 0:初始值—— 1:环岛岔路口1—— 2:环岛岔路口2—— 3:环岛岔路口3—— 4：环岛岔路口4
## 在_execute_state_control中会根据intersection_flag的值来选择特殊控制器


## 8月4号改动：
## 修改了_round_intersection这个检查函数，经过第一个岔路口后会调用该函数，用来判断是否到达第二岔路口，然后会控制小车倒退然后旋转

## 8月5号改动：
## 修改了国赛时的状态机运动管理的函数，两次特殊过渡控制的时间间隔不能小于3s，防止错判
## 修改了巡线图像处理函数Lanedetecion,限制了选拔赛中特殊控制只会触发一次

## 8月6号改动：
## 打算修改一下

## 8月9号改动：
## 修改了LaneDetectionNode.run函数，在开头记录打印任务开始时间，然后还添加了一个imu订阅者，在任务开始3s之后会记录imu的位姿，并在
## _execute_state_control中的特殊控制过渡添加了限制条件——首先是开始任务后的6s内不能触发，同时还添加了一个位姿的限制

# ==================调参保存=================
# 选拔赛方案
# 方案一：速度为0.32，前视距离设置为115，曲率影响参数设为2.0，岔路口参数设置
#mode == 0:(0.8, 0.16, 1.2, 0.82)
#mode == 1:(0.8, 0.16, 1.2, 0.96)
#方案二： 速度为0.55，前是距离设置为124，曲率影响设为3.5，岔路口的参数和设置为多旋转0.2秒，直行速度也增加0.1


# ================== 全局参数配置区 ==================
# 调试参数
DEBUG = False                # 启用调试模式,显示窗口

# 环岛行为参数	
NEED_ROUNDABOUT = False # 是否需要环岛
INROUNDABOUT = 0 # 
MODE0_MOVE_OFFSET = 15  # mode=0时的法线平移增加量
# 参数说明：（直行时间，平移速度，旋转速度，直行速度，旋转时间，旋转速度）
# 选拔赛路口的过渡控制参数数组
INTERSECTION_PARAMS = {
    1.0: (1.0, -0.02, 0.88, 0.34, 0.0, 0.50),  # 第1个路口的mode==0控制参数（直行/平移+旋转）
    1.1: (1.0, -0.02, 0.88, 0.34, 0.0, 0.50),  # 第1个路口的mode==1控制参数 （直行/平移+旋转）
} if not NEED_ROUNDABOUT else {
    1.0: (1.0, -0.02, 0.55, 0.23, 0.8, 0.60),  # 第1个路口的mode==0控制参数（直行/平移+旋转）
    1.1: (1.0, -0.02, 0.55, 0.26, 0.8, 0.60),  # 第1个路口的mode==1控制参数 （直行/平移+旋转）
}
# 国赛环岛的三个路口的过渡控制参数数组
FINAL_INTERSECTION_PARAMS = {
    1.0: (1.0, 0.02, -0.45, 0.30, 0.6, -0.60),  # 第1个路口的mode==0控制参数（直行/平移+旋转）
    1.1: (1.0, 0.02, -0.45, 0.30, 0.6, -0.60),  # 第1个路口的mode==1控制参数 （直行/平移+旋转）
    2: (1.1, -0.02, 0.0, -0.15, 0.99, -1.38),  # 第2个路口的控制参数（后退+旋转）
    3: (1.1, 0.00, -0.0, 0.32, 0.0, -1.00),    # 第3个路口的控制参数（直行+旋转）
}


#二值化参数
BINARY_THRESHOLD = 180
low_threshold = 40
high_threshold = 150
kernel_size = 3

# 种子点搜索参数
SEARCH_HEIGHT = 68  # 最大向上搜索范围
SEED_THRESHOLD = 180 # 种子点阈值

# 寻线方法相关参数
MOVE_OFFSET =  116		            # 向法线方向移动距离(像素)
MAX_STEPS = 200             # 最大迭代次数
BLOCK_SIZE = 3  # 迷宫法核大小
CLIP_VALUE = 6  # 裁剪值

TRACK_WIDTH = 0.39  # 米，赛道宽度 39cm
# 车库角点检测参数
CORNER_WINDOW_SIZE = 20     # 角点检测窗口大小（计算点的选取）（待调试）
ANGLE_THRESHOLD = 55       # 视为车库角点的角度阈值，可适当降低容错
NMS_THRESHOLD =  20          # 非极大值抑制，剔除附近的重复角点（待调试）

#==== 纯跟踪算法参数==================
LOOKAHEAD_DISTANCE = 115    # 前视距离
LATERAR_GAIN = 2.6            # 曲率计算部分dx横向补偿增益（待调试）

# 前向速度限制常量
MAX_LINEAR_SPEED = 0.32     # m/s 目前是调横向和转向控制的pid，所以先设为0.0前向速度
MIN_LINEAR_SPEED = 0.0     # m/s

# 转向控制参数量
MAX_ANGULAR_SPEED = 1.4   # rad/s
MIN_ANGULAR_SPEED = 0.0
KP_YAW = 0.7                # 航向P增益
KD_YAW = 0.1                # 航向D增益	
KI_YAW = 0.0                # 航向I增益（抗稳态误差）
YAW_ERROR_THRESHOLD = 0.06	   # 航向误差死区(rad)，如果觉得微小的误差影响不大，可以适当调大，反之可以调小

# 停车控制器参数
STOP_DISTANCE = 40          # 停车的判断条件是前视点与基准位置的差值小于该阈值 

# 曲率适应参数常量
CURVATURE_SPEED_FACTOR = 2.0 # 曲率对速度的影响系数（系数越大前向速度会越小）



# 相机内参
camera_matrix = np.array([
    [637.5526471889214*0.5, 0.0, 639.0844243459007*0.5],
    [0.0, 637.5149155824262*0.5, 359.5701497245531*0.5],
    [0.0, 0.0, 1.0]    ], dtype=np.float32)

# 畸变系数
dist_coeffs = np.array([
    -0.3511004161703222, 0.22238663945200993,
    0.00032134526224666563, 0.0012957640630325122,
    -0.12254735437504892    ], dtype=np.float32)


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

# ================= 图像预处理 =================
class PreProcess:
    """
    图像与处理：灰度化、高斯模糊、腐蚀、膨胀、阈值化、透视变换
    使用方法：实例化 -> 调用process方法
    """
    def __init__(self):
        """
        初始化
        """
        # 图像预处理参数配置区 ======================================
        self.img_size = (640, 360)  # 图像尺寸
        self.vehicle_pt_original = np.array([[self.img_size[0] // 2, self.img_size[1] - 1]], dtype=np.float32) # 车辆基准位置

        self.threshold = BINARY_THRESHOLD
        self.low_threshold = low_threshold
        self.high_threshold = high_threshold
        self.kernel_size = kernel_size

        # BEV参数配置区 ======================================
        self.output_size = (640, 360)  # (width, height)
        self.pitch_deg = 18	        # 初始俯仰角
        self.camera_height = 0.11     # 相机高度(米)
        self.ground_width = 0.78      # 地面检测宽度(米)
        self.ground_depth = 0.50	      # 地面检测深度(米)
        
        # 预计算变换矩阵
        self.H = None
        self._init_perspective_matrix()

    def adaptive_threshold(self, img, block_size=11, down_value=11, low_value=0, high_value=255):
        """自适应阈值二值化处理
        参数:
            img: 输入图像(BGR格式)
            block_size: 计算局部阈值的邻域大小(奇数)
            down_value: 从平均灰度值中减去的调整值
            low_value: 低于阈值时赋予的值
            high_value: 高于阈值时赋予的值
        返回:
            处理后的二值图像(黑色线条，白色背景)
        """
        # 获取图像尺寸
        height, width = img.shape[:2]

        # 转为灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

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

        # 形态学操作（可根据需要调整）
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        dilated = cv2.dilate(closed, kernel, iterations=1)
        eroded = cv2.erode(dilated, kernel, iterations=1)

        return eroded

    def THreshold(self, img):
        """二值化处理
        传入参数： img
        返回参数： 二值化处理后的图像
        处理步骤： 取图像 -> 转为灰度图 -> 高斯模糊 -> 腐蚀 -> 膨胀 -> 自适应二值化
        """
        # 获取图像的高度和宽度
        height, width = img.shape[:2]  
        roi = img[height//2:,:]
        # 转为灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # 高斯模糊
        blur = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
        # 方案1 自适应二值化
        # dual_lane = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 5)
        # 方案2 边缘检测
        dual_lane = cv2.Canny(blur, self.low_threshold, self.high_threshold)
        # 创建一个闭运算的核
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        closed = cv2.morphologyEx(dual_lane, cv2.MORPH_CLOSE, kernel)
        # 在闭运算结果上应用dilate和erode操作
        dilated = cv2.dilate(closed, kernel, iterations=1)
        eroded = cv2.erode(dilated, kernel, iterations=1)
        # 创建掩码，黑色为线条，其它部分为白色
        dual_lane = np.where(eroded == 255, 0, 255).astype(np.uint8)
        dual_lane = 255 - dual_lane

        return dual_lane

    def show_pitch_direction(self, img):
        """可视化俯仰角方向"""
        h, w = img.shape[:2]
        # 绘制相机视锥
        cv2.line(img, (w//2, h), (w//2, h//2), (0,255,0), 2)  # 中心线
        # 根据俯仰角绘制方向
        pitch_pixels = int(100 * np.tan(np.deg2rad(self.pitch_deg)))
        cv2.line(img, (w//2, h), (w//2 + pitch_pixels, h//2), (0,0,255), 2)
        cv2.imshow("Pitch Direction", img)
    
    def _init_perspective_matrix(self):
        """预计算单应性矩阵"""
        pitch = np.deg2rad(self.pitch_deg)
    
        # 地面坐标系四个角点 (世界坐标系)
        ground_points = np.array([
            [-self.ground_width/2, 0, 0],
            [self.ground_width/2, 0, 0],
            [self.ground_width/2, self.ground_depth, 0],
            [-self.ground_width/2, self.ground_depth, 0]
        ], dtype=np.float32)

        image_points = []
        for pt in ground_points:
            X_ground, Y_ground, _ = pt
        
            # 相机坐标系转换（注意Y轴方向）
            X_cam = X_ground
            Y_cam = (self.camera_height * np.cos(pitch) - Y_ground * np.sin(pitch))  # 添加负号
            Z_cam = self.camera_height * np.sin(pitch) + Y_ground * np.cos(pitch)
            
#            if Z_cam <= 0.01:
#                raise ValueError("俯仰角或地面范围设置不合理，导致点在相机后方")
            
            # 投影到图像
            u = (camera_matrix[0,0] * X_cam / Z_cam) + camera_matrix[0,2]
            v = (camera_matrix[1,1] * Y_cam / Z_cam) + camera_matrix[1,2]
            image_points.append([u, v])
        
#        for pt in image_points:
#            u, v = pt
#            if u < 0 or u >= self.img_size[0] or v < 0 or v >= self.img_size[1]:
#                print(f"警告: 投影点({u},{v})超出图像范围")
        # 目标BEV图像坐标
        bev_dst = np.array([
            [0, self.output_size[1]-1],
            [self.output_size[0]-1, self.output_size[1]-1],
            [self.output_size[0]-1, 0],
            [0, 0]
        ], dtype=np.float32)

        # 计算单应性矩阵
        self.H = cv2.getPerspectiveTransform(
            np.array(image_points, dtype=np.float32), 
            bev_dst
        )
        # 计算逆变换矩阵
        self.H_inv = cv2.getPerspectiveTransform(bev_dst, np.array(image_points, dtype=np.float32))
    
    def show_ground_area(self, img):
        """在原始图像上绘制地面检测区域"""
        pitch = np.deg2rad(self.pitch_deg)
    
        # 定义地面网格点
        X = np.linspace(-self.ground_width/2, self.ground_width/2, 5)
        Y = np.linspace(0, self.ground_depth, 5)
    
        # 绘制网格
        for x in X:
            for y in Y:
                # 计算投影
                X_cam = x
                Y_cam = self.camera_height * np.cos(pitch) - y * np.sin(pitch)
                Z_cam = self.camera_height * np.sin(pitch) + y * np.cos(pitch)

                u = int((camera_matrix[0,0] * X_cam / Z_cam) + camera_matrix[0,2])
                v = int((camera_matrix[1,1] * Y_cam / Z_cam) + camera_matrix[1,2])

                if 0 <= u < self.img_size[0] and 0 <= v < self.img_size[1]:
                    cv2.circle(img, (u, v), 3, (0, 255, 0), -1)
    
        cv2.imshow("Ground Detection Area", img)
        return img
    
    def PerspectiveTransform(self, img):
        """
        执行逆透视变换
        :param img: 输入图像(BGR格式)
        :return: 鸟瞰图(BGR格式)以及车辆基准位置（鸟瞰图中）
        """
        if self.H is None:
            raise RuntimeError("未初始化变换矩阵，请先调用_init_perspective_matrix()")
            
        # undistorted = cv2.undistort(undistorted, camera_matrix, dist_coeffs)
 
        # 图像执行变换时添加边框处理
        bev_image = cv2.warpPerspective(
            img, 
            self.H, 
            self.output_size,
            flags=cv2.INTER_LINEAR + cv2.WARP_FILL_OUTLIERS,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0  # 黑色填充
        )

        return bev_image

    def inv_PerspectiveTransform(self, img):
        """
        逆透视变换
        传入参数： img输入图像
        """
        # 进行逆透视变换
        inv_perspective_img = cv2.warpPerspective(
            img, 
            self.H_inv, 
            self.output_size
        )

        return inv_perspective_img

    def process(self, img):
        """
        处理图像
        传入参数： img输入图像
        返回值： 经过二值化、透视变换后的图像
        """
       #去畸变
        # img = cv2.undistort(src=img, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs)
        #车辆基准位置定为图像中心底部

        # 二值化
        # binary_img = self.THreshold(img)
        # cv2.imshow("binary", binary_img)

        # 自适应二值化
        binary_img = self.adaptive_threshold(img)
 
        # 透视变换（返回变化后的图像）
        bird_view_img = self.PerspectiveTransform(binary_img)
        cv2.imshow("birdview", bird_view_img)

        car_pt_in_bev_image = cv2.perspectiveTransform(
                self.vehicle_pt_original.reshape(1,1,2),  # 特殊shape：(1,1,2)
                self.H
            )[0][0]  # 得到[x',y']  

        # 返回 鸟瞰图、车辆基准位置、二值化图像
        return bird_view_img, car_pt_in_bev_image

# ================= 获取车道线行驶线，曲率 =================
class Lanedetection:
    """
    自动寻找车道线，行驶线，并计算曲率，前视点
    使用方法： 调用detect方法
    输入值： bird_view: 透视变换后的二值图像，mode: 0-左线 1-右线，car_pt: 车辆基准位置(鸟瞰图中)
    返回值： result_img （仅当DEBUG=True时） , curvature
    """
    def __init__(self, mode=1,dst_size=(640, 360)):
        """
        车道线检测器
        行驶线获取
        :param mode: 0-左线 1-右线
        """      
        self.calculspeed = calculate_speed() # 
     
        self.need_roundabout = NEED_ROUNDABOUT # 国赛需要用到的环岛开关          
        self.in_roundabout = 0 # 国赛需要用到的环岛判断，初始化为0，表示未进入
        self.not_arrived_2_intersection = 1 # 表示未到达环岛的第二个路口
        self.not_arrived_1_intersection = 1 # 表示未到达环岛的第1个路口      
        
    def find_seed_point(self, img, side, car_pt):
        """
        寻找种子点
        :param img: 二值化图像或者透视变换后的图像
        :param side: 'left' or 'right'
        :param car_pt: 车辆基准位置
        :return: 种子点坐标（x，y）
        """
        car_x, car_y = int(car_pt[0]), int(car_pt[1])
        height, width = img.shape[:2]

        # 搜索参数
        search_height = SEARCH_HEIGHT  # 最大向上搜索范围
        threshold = SEED_THRESHOLD  # 阈值

        # 根据左右侧设置不同的搜索方向
        if side == 'left':
            # 左侧搜索：优先向左搜索
            search_range = range(-200, 0, 1)  # 向左搜索
            neighbor_check = -1  # 检查左侧相邻像素
        else:  # 'right'
            # 右侧搜索：优先向右搜索
            search_range = range(0, 200, 1)  # 向右搜索
            neighbor_check = 1  # 检查右侧相邻像素

        # 1. 优先在车辆所在行附近搜索
        for y_offset in range(0, search_height, 1):  # 逐行搜索
            y = car_y - 10- y_offset
            if y < 0:
                break

            # 在当前行向指定方向搜索
            for x_offset in search_range:
                x = car_x + x_offset
                if 0 <= x < width:
                    if img[y, x] > threshold:
                        # 检查是否连续两个或以上的像素点大于阈值
                        if img[y, x + neighbor_check] > threshold:
                            return x + neighbor_check, y

        # 2. 如果附近没找到，向上逐行搜索
        for y in range(car_y - 1, max(0, car_y - search_height), -1):
            for x_offset in search_range:
                x = car_x + x_offset
                if 0 <= x < width:
                    if img[y, x] > threshold:
                        # 检查是否连续两个或以上的像素点大于阈值
                        if img[y, x + neighbor_check] > threshold:
                            return x + neighbor_check, y

        # 3. 搜索失败，返回空种子点
#        print(f"未找到有效的{side}侧种子点")
        return None, None

    def _maze_handfind_lane(self, binary_img, side, car_pt):
        """方案2：迷宫法车道检测（支持左右手法则）"""
        # 方向定义：0-上, 1-右, 2-下, 3-左
        dir_front = [(0, -1), (1, 0), (0, 1), (-1, 0)]  # 前方向量
        dir_frontleft = [(-1, -1), (1, -1), (1, 1), (-1, 1)]  # 左前方向量
        dir_frontright = [(1, -1), (1, 1), (-1, 1), (-1, -1)]  # 右前方向量
        
        # 初始化参数
        step_count = 0
        max_steps = MAX_STEPS  # 最大步数限制
        block_size = BLOCK_SIZE  # 自适应阈值块大小（奇数）
        clip_value = CLIP_VALUE  # 阈值调整值
        half = block_size // 2
        search_range = 8  # 扩展搜索范围
        max_jump_searches = 5  # 最大跳转搜索次数限制
        jump_search_count = 0  # 跳转搜索计数器

        # 获取图像尺寸并初始化搜索区域
        height, width = binary_img.shape[:2]
        car_x, car_y = int(car_pt[0]), int(car_pt[1])
        search_width = 2 * car_x
        search_height = car_y
        
        
        # 获取种子点并验证有效性
        seed_x, seed_y = self.find_seed_point(binary_img, side, car_pt)
        
        # 验证种子点有效性
        if (seed_x is None or seed_y is None):
       #     print(f"[DEBUG] {side}侧种子点无效: ({seed_x}, {seed_y})")
            return np.array([]), np.array([])  # 返回空数组
        
        x, y = int(seed_x), int(seed_y)

        dir_idx = 0  # 初始化方向索引
        
        # 初始化车道线坐标列表
        lane_x, lane_y = [], []
        turn_count = 0
        stuck_count = 0  # 添加卡死计数器
        prev_pos = (x, y)
        
        def extended_search(curr_x, curr_y, curr_dir_idx, local_thres):
            """扩展搜索函数：向前方(y轴负方向)搜索车道线"""
            #print(f"[DEBUG] {side}侧开始扩展搜索，当前位置: ({curr_x}, {curr_y})")
            
            # 固定搜索方向：向前方(y轴负方向)
            # 不再依赖当前运动方向，而是固定向图像上方搜索
            search_directions = [
                (0, -1),   # 正前方 (向上)
                (-1, -1),  # 左前方 (左上)
                (1, -1)    # 右前方 (右上)
            ]
            
            # 在每个方向搜索
            for direction in search_directions:
                dx, dy = direction
                
                # 在当前方向搜索指定范围
                for distance in range(1, search_range + 1):
                    search_x = curr_x + dx * distance
                    search_y = curr_y + dy * distance
                    
                    # 边界检查
                    if (search_x < half or search_x >= width - half or 
                        search_y < half or search_y >= height - half):
                        break
                    
                    # 额外检查：确保搜索的点在搜索区域内
                    if search_x >= search_width or search_y >= search_height:
                        break
                        
                    # 检查该点是否为车道线
                    if binary_img[search_y, search_x] >= local_thres:
                        #print(f"[DEBUG] {side}侧扩展搜索找到车道线: ({search_x}, {search_y}), 距离: {distance}")
                        return search_x, search_y
            
            #print(f"[DEBUG] {side}侧扩展搜索未找到车道线")
            return None, None

    
        # 主循环
        while (step_count < max_steps and 
            half <= x < width - half and 
            half <= y < height - half and
            stuck_count < 4 and turn_count <=2 and 
            jump_search_count < max_jump_searches):   # 添加卡死保护
            
            # 记录当前点
            lane_x.append(x)
            lane_y.append(y)
            step_count += 1
            
            # 计算局部自适应阈值
            local_region = binary_img[y-half:y+half+1, x-half:x+half+1]
            local_thres = np.mean(local_region) - clip_value
            
            # 边界检查
            front_x = x + dir_front[dir_idx][0]
            front_y = y + dir_front[dir_idx][1]
            side_x = x + (dir_frontleft[dir_idx][0] if side == 'left' else dir_frontright[dir_idx][0])
            side_y = y + (dir_frontleft[dir_idx][1] if side == 'left' else dir_frontright[dir_idx][1])
            
            # 确保坐标在有效范围内
            if (front_x < 0 or front_x >= width or front_y < 0 or front_y >= height or
                side_x < 0 or side_x >= width or side_y < 0 or side_y >= height):
                break
            
            # 获取像素值
            front_value = binary_img[front_y, front_x]
            side_value = binary_img[side_y, side_x]
            
            # 保存当前位置用于卡死检测
            old_pos = (x, y)
            
            # 检查是否需要进行扩展搜索（转向次数达到阈值）
            if turn_count >= 2:
                #print(f"[DEBUG] {side}侧转向次数达到{turn_count}，尝试扩展搜索")
                new_x, new_y = extended_search(x, y, dir_idx, local_thres)
                
                if new_x is not None and new_y is not None:
                    # 找到了新的车道线点，跳转过去
                    x, y = new_x, new_y
                    turn_count = 0  # 重置转向计数
                    stuck_count = 0  # 重置卡死计数
                    jump_search_count += 1  # 增加跳转搜索计数
                  #  print(f"[DEBUG] {side}侧扩展搜索成功，跳转到: ({x}, {y})")
                    continue
                else:
                    # 扩展搜索也没找到，结束循环
               #     print(f"[DEBUG] {side}侧扩展搜索失败，结束搜索")
                    break
            
            # 左右手法则决策
            if side == 'left':
                if front_value < local_thres:
                    dir_idx = (dir_idx + 1) % 4
                    turn_count += 1
                elif side_value < local_thres:
                    x, y = front_x, front_y
                    turn_count = 0
                else:
                    x, y = side_x, side_y
                    dir_idx = (dir_idx - 1) % 4
                    turn_count = 0
            else:
                if front_value < local_thres:
                    dir_idx = (dir_idx - 1) % 4
                    turn_count += 1
                elif side_value < local_thres:
                    x, y = front_x, front_y
                    turn_count = 0
                else:
                    x, y = side_x, side_y
                    dir_idx = (dir_idx + 1) % 4
                    turn_count = 0
            
            # 限制搜索区域
            x = int(np.clip(x, 0, search_width - 1))
            y = int(np.clip(y, 0, search_height - 1))
            
            # 卡死检测
            if (x, y) == old_pos:
                stuck_count += 1
            else:
                stuck_count = 0
        
        # 车道线有效性验证
        lane_x_arr = np.array(lane_x)
        lane_y_arr = np.array(lane_y)
        
        # 添加调试打印语句
        #print(f"此次搜寻得到车道线： {side}线数量为{len(lane_x_arr)}") 
          
        return lane_x_arr, lane_y_arr


    def detect_corners(self, leftx, lefty, rightx, righty, 
                    window_size=20, angle_threshold=60, 
                    nms_threshold=10, debug=False):
        """
        识别车道线中的角点（基于向量夹角法 + 非极大抑制）
        
        参数:
            leftx, lefty: 左车道线像素坐标数组
            rightx, righty: 右车道线像素坐标数组
            window_size: 计算角度时前后点的距离（单位：像素，默认20）
            angle_threshold: 视为角点的最小角度阈值（度，默认60）
            nms_threshold: 非极大抑制的邻域范围（像素，默认10）
            debug: 是否输出调试信息
            
        返回:
            {
                'left_corners': [(x, y, angle), ...],  # 左线角点列表(坐标+角度)
                'right_corners': [(x, y, angle), ...], # 右线角点列表
                'all_corners': [(x, y, angle, 'L'/'R'), ...] # 合并角点+左右标记
            }
        """
        def _calculate_angles(x_arr, y_arr):
            # 添加空数组检查
            if len(x_arr) == 0 or len(y_arr) == 0:
                return []
            angles = []
            n = len(x_arr)
            for i in range(window_size, n - window_size):
                # 前向向量
                dx1 = x_arr[i] - x_arr[i - window_size]
                dy1 = y_arr[i] - y_arr[i - window_size]
                # 后向向量
                dx2 = x_arr[i + window_size] - x_arr[i]
                dy2 = y_arr[i + window_size] - y_arr[i]
                
                # 计算夹角（角度制）
                dot = dx1 * dx2 + dy1 * dy2
                det = dx1 * dy2 - dy1 * dx2
                angle = math.degrees(math.atan2(det, dot))
                angle = abs(angle)  # 取绝对值（0~180°）
                angles.append((x_arr[i], y_arr[i], angle))
            
            return angles
        
        def _apply_nms(corners):
            if not corners:
                return []
            
            # 按角度降序排序
            sorted_corners = sorted(corners, key=lambda x: -x[2])
            kept = []
            
            while sorted_corners:
                current = sorted_corners.pop(0)
                kept.append(current)
                
                # 移除邻域内的其他点
                sorted_corners = [
                    c for c in sorted_corners 
                    if math.hypot(c[0]-current[0], c[1]-current[1]) > nms_threshold
                ]
            
            return kept
        
        # 处理左右车道线
        left_angles = _calculate_angles(leftx, lefty) if len(leftx) > 3*window_size else []
        right_angles = _calculate_angles(rightx, righty) if len(rightx) > 3*window_size else []
        
        # 筛选角度大于阈值的点
        left_candidates = [(x, y, a) for (x, y, a) in left_angles if a >= angle_threshold]
        right_candidates = [(x, y, a) for (x, y, a) in right_angles if a >= angle_threshold]
        
        # 非极大抑制
        left_corners = _apply_nms(left_candidates)
        right_corners = _apply_nms(right_candidates)
        
        # 合并结果（添加左右标记L/R）
        all_corners = [
            (*corner, 'L') for corner in left_corners
        ] + [
            (*corner, 'R') for corner in right_corners
        ]
        
        return {
            'left_corners': left_corners,
            'right_corners': right_corners,
            'all_corners': all_corners
        }
    
    
    def process_raw_lane_points(self, raw_x, raw_y, side):
        """
        改进版-带动态窗口调整的车道点处理
        """
        offset_pixels=MOVE_OFFSET
        # 转换为numpy数组并过滤无效点
        raw_points = np.column_stack((np.array(raw_x), np.array(raw_y)))
        raw_points = raw_points[~np.isnan(raw_points).any(axis=1)]

        if len(raw_points) < 5:  # 最少需要5个点
            return np.array([]), np.array([])
    
        # 1. 分层处理（动态调整层数）
        y_min, y_max = np.min(raw_points[:,1]), np.max(raw_points[:,1])
        bin_size = max(4, int((y_max-y_min)/20))  # 动态bin大小
        y_bins = np.arange(y_min, y_max, bin_size)
    
        # 2. 提取每层中心点
        center_points = []
        for i in range(len(y_bins)-1):
            layer_mask = (raw_points[:,1] >= y_bins[i]) & (raw_points[:,1] < y_bins[i+1])
            if np.sum(layer_mask) > 0:
                median_x = np.median(raw_points[layer_mask, 0])
                center_points.append([median_x, (y_bins[i]+y_bins[i+1])/2])
    
        if len(center_points) < 3:  # 中心点太少直接返回
            return np.array([]), np.array([])
    
        # 3. 排序中心点
        center_points = np.array(center_points)
        sorted_idx = np.argsort(center_points[:,1])
        ordered_center = center_points[sorted_idx]
    
        # 4. 动态调整平滑窗口
        n_points = len(ordered_center)
        window_length = min(15, n_points - (n_points % 2 == 0))  # 确保奇数且不超过点数
        window_length = max(3, window_length)  # 最小窗口为3
    
        try:
            from scipy.signal import savgol_filter
            # 使用Savitzky-Golay滤波（动态窗口）
            smooth_x = savgol_filter(ordered_center[:,0], 
                                    window_length=window_length, 
                                    polyorder=min(2, window_length-1))
            smooth_y = ordered_center[:,1]
        except Exception as e:
            print(f"平滑失败: {e}, 使用原始中心点")
            smooth_x, smooth_y = ordered_center[:,0], ordered_center[:,1]
    
        # 5. 法线平移
        if len(smooth_x) >= 2:
            offset_x, offset_y = self.robust_offset(smooth_x, smooth_y, offset_pixels, side)
            return offset_x, offset_y
        return np.array([]), np.array([])

    def robust_offset(self, x, y, offset, side):
        """改进版法线平移"""
        dx = np.gradient(x)
        dy = np.gradient(y)
        norm = np.sqrt(dx**2 + dy**2) + 1e-5
        nx = dy / norm
        ny = -0.5 * dx / norm
        sign = 1 if side == 'left' else -1
        if side == 'left':
            offset += MODE0_MOVE_OFFSET	
        return x + sign*offset*nx, y + sign*offset*ny


    def detect_white_line_distance(self, img, car_pt_in_bev_image):
        """
        从车辆基准位置向上搜索白线，计算距离，用于停车控制的距离判断
        
        :param img: 透视变换后的二值图像
        :param car_pt_in_bev_image: 车辆基准位置 (x, y)
        :return: distance - 车辆到白线中心的距离，如果没有检测到白线则返回0
        """
        if img is None or car_pt_in_bev_image is None:
            return 0
        
        # 获取图像尺寸
        height, width = img.shape[:2]
        car_x, car_y = int(car_pt_in_bev_image[0]), int(car_pt_in_bev_image[1])

        # 设置搜索范围：从车辆位置向上搜索到图像上方
        search_start_y = car_y
        search_end_y = 0  # 搜索到图像最上面

        # 用于记录连续白色像素的计数
        white_pixel_count = 0
        white_line_start_y = -1
        white_line_end_y = -1
        
        # 从车辆位置开始向上搜索（y坐标递减）
        for y in range(search_start_y - 1, search_end_y - 1, -1):
            # 获取当前行的像素值
            pixel_value = img[y, car_x]
            
            # 判断是否为白色像素（灰度值大于100）
            if pixel_value > 100:
                my_print(f"在 y={y} 处找到白色像素")
            
                if white_pixel_count == 0:
                    white_line_start_y = y  # 记录白线开始位置
                white_pixel_count += 1
                white_line_end_y = y  # 更新白线结束位置
                
                # 如果连续点都是白色，认为找到了白线
                if white_pixel_count >= 1:
                    # 计算白线中心点的y坐标
                    white_line_center_y = (white_line_start_y + white_line_start_y) // 2
                    
                    # 计算车辆基准位置到白线中心的距离（y坐标差值）
                    distance = abs(car_y - white_line_center_y)
                    
                    return distance
            else:
                # 遇到非白色像素，重置计数
                white_pixel_count = 0
                white_line_start_y = -1
                white_line_end_y = -1
        
        # 没有找到连续的白线，返回0
        my_print("未看到车库底线！")
        return 0


    def detect(self, img, mode=None, car_pt_in_bev_image=None):
        """
        :主流程
        :param bird_view: 透视变换后的二值图像
        :param mode: 0-左线 1-右线
        :param car_pt_in_bev_image: 车辆基准位置(鸟瞰图中)
        :return: result_img, curvature, lookahead_point, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, lost_flag, garage_detected
        流程介绍：用左右手法则函数获取车道线像素点
                            |
                     检查车道线是否丢失——如果丢失，设置返回值但保持数量一致，设置丢失标志
                            |
                     车道线未丢失则先检测是否有车库角点——如果有，返回入库标志
                            |
                     车道线正常，无入库标志，则调用纯跟踪算法，返回跟踪结果（前视点、曲率等等）
                            |
                     如果是国赛模式，则需要根据self.in_roundabout来选择是否需要监测环岛岔路口标志位
                            |
                     到达标志位之后，需要返回,传入巡线节点，进行特殊控制过渡
        """
        # 根据模式和传入的车辆基准位置，确定搜索起点
        if mode == 0:
            side = 'left'
        else:
            side = 'right'

        self.last_valid_dir = 0  # 方向记忆清零    
        current_frame = img.copy()  # 深拷贝图片

        # 初始化返回值
        curvature = 0
        lookahead_point = (0, 0)
        garage_corners = []
        lost_flag = False
        garage_detected = False
        distance = 0
        drive_line = (np.array([]), np.array([]))
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0

        try:
            # 1. 调用种子生长法以及左右手法则函数获取车道线像素点    
            leftx, lefty = self._maze_handfind_lane(current_frame, 'left', car_pt_in_bev_image)
            rightx, righty = self._maze_handfind_lane(current_frame, 'right', car_pt_in_bev_image)
        except Exception as e:
            rospy.logerr(f"迷宫巡线搜寻失败: {str(e)}")
            # 如果搜寻失败，设置为空数组
            leftx, lefty = np.array([]), np.array([])
            rightx, righty = np.array([]), np.array([])

        if self.not_arrived_1_intersection:
        # 检查车道线是否丢失（失效）调用self.calculspeed._check_lane_valid()函数
            is_lane_valid = _check_lane_valid(leftx, lefty, rightx, righty, mode)
            # 新建一个返回标志lost_flag 来接收is_lane_valid的值
            lost_flag = not is_lane_valid


        if lost_flag:
            # 如果车道线丢失，设置返回值但保持数量一致
            rospy.logwarn("车道线丢失，已到达岔路口")

            if NEED_ROUNDABOUT:
#                rospy.loginfo("国赛模式，到达岔路口，准备开始环岛")
                # 调用环岛控制算法
                self.in_roundabout =True # 设置进入环岛标志位
            else :
                self.not_arrived_1_intersection = 0

            # 仍然需要把检测到的车道线像素点传入可视化函数
            result_img = self._create_visualization(
                current_frame, 
                leftx, lefty, 
                rightx, righty,  
                drive_line,  #为空数组
                vehicle_pt=car_pt_in_bev_image,
                lookahead_pt=lookahead_point # 为（0，0）
            )
        
            # 返回值数量保持一致，返回的曲率为0，前时点也是0，车库点是0，
            return result_img, curvature, lookahead_point, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, distance

        # 3. 检测角点（车道线有效时才检测）
        corner_info = self.detect_corners(
            leftx, lefty, rightx, righty,
            window_size=CORNER_WINDOW_SIZE,     # 根据像素比例调整（建议为车道宽度的1/3~1/2）
            angle_threshold=ANGLE_THRESHOLD,    # 车库角点通常为90°，可适当降低容错
            nms_threshold=NMS_THRESHOLD,        # 避免相邻重复角点
            debug=True                          # 调试时开启
        )
        
        # 3. 提取车库角点（左右均保留）
        garage_corners = [
            (x, y, side)
            for (x, y, angle, side) in corner_info['all_corners']
            if 50 < angle < 160   # 容差
        ]

        # 4. 检查车库是否出现
        if not self.not_arrived_1_intersection:
            garage_detected = has_left_and_right_corners(leftx, lefty, rightx, righty, garage_corners)
            
            if garage_detected:
                rospy.loginfo("检测到车库，进入停车控制模式")
                # 检测到车库时，强制清空车道线数据
                leftx, lefty = np.array([]), np.array([])
                rightx, righty = np.array([]), np.array([])
                lost_flag = False  # 
                drive_line = (np.array([]), np.array([]))  # 清空行驶线

                # 计算车辆基准位置到白线中心的距离（y坐标差值）
                distance = self.detect_white_line_distance(img, car_pt_in_bev_image)

                if DEBUG:
                    # 调用车库停车控制算法
                    twist, finish_flag = self.calculspeed.perform_garage_parking_control(distance)
                
                # 生成可视化图像（使用检测到的车道线像素点）
                result_img = self._create_visualization(
                    current_frame, 
                    leftx, lefty, 
                    rightx, righty,
                    (np.array([]), np.array([])),  # drive_line设为空，因为不使用巡线
                    vehicle_pt=car_pt_in_bev_image,
                    lookahead_pt=lookahead_point
                )
                
                if DEBUG    :   
                    # 绘制车库角点
                    for x, y, side in garage_corners:
                        color = (255, 0, 255) if side == 'L' else (255, 255, 0)  # 左粉右青
                        cv2.drawMarker(result_img, 
                                    (int(x), int(y)), 
                                    color, 
                                    markerType=cv2.MARKER_STAR, 
                                    markerSize=10, 
                                    thickness=2)
                    
                    # 添加速度信息可视化
                    result_img = self._draw_twist_info(result_img, twist, car_pt_in_bev_image, curvature=curvature)
                
                return result_img, curvature, lookahead_point, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, distance


        try:
            if mode == 0:
                drive_line = self.process_raw_lane_points(leftx, lefty, side)
            elif mode == 1:
                drive_line = self.process_raw_lane_points(rightx, righty, side)
            if drive_line is None:  # 新增检查
                rospy.logwarn("无法生成行驶轨迹线，返回空数组")
                drive_line = np.array([]), np.array([])
        except Exception as e:
            rospy.logerr(f"行驶路径移动失败: {str(e)}")
            drive_line = np.array([]), np.array([])

        # 7. 纯跟踪算法获取转向曲率，并返回预瞄点坐标
        curvature, lookahead_point = self.pure_pursuit_control(drive_line, vehicle_pt=car_pt_in_bev_image)

        # 环岛过程中需要的特殊路口处理，只有在国赛中，并且先要经过岔路口1号才进行该操作
        if NEED_ROUNDABOUT and INROUNDABOUT and self.not_arrived_2_intersection:
            if _round_insection(leftx, lefty, rightx, righty, curvature, mode):
                my_print("检测到岔路口2！")
                self.not_arrived_2_intersection = False
                lost_flag = 1
                # 仍然需要把检测到的车道线像素点传入可视化函数
                result_img = self._create_visualization(
                    current_frame, 
                    leftx, lefty, 
                    rightx, righty,  
                    drive_line,  
                    vehicle_pt=car_pt_in_bev_image,
                    lookahead_pt=lookahead_point
                )
                return result_img, curvature, lookahead_point, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, distance

         
        # 8. 解析计算得到速度消息包twist（注意这里只是用于可视化绘图，所以在返回值中不需要twist）
#        twist = self.calculspeed.calculate_control(
#            curvature=curvature, 
#            lookahead_pt=lookahead_point, 
#            vehicle_pt=car_pt_in_bev_image,                        
#            ready_park=0
#            )

        # 9. 生成可视化图像
        result_img = self._create_visualization(
            current_frame, 
            leftx, lefty, 
            rightx, righty,
            drive_line,
            vehicle_pt=car_pt_in_bev_image,
            lookahead_pt=lookahead_point
        )   
        # 添加速度信息可视化
#        result_img = self._draw_twist_info(result_img, twist, car_pt_in_bev_image, curvature=curvature)
        
        return result_img, curvature, lookahead_point, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, distance

    def pure_pursuit_control(self, drive_line, vehicle_pt=None):
        """
        优化版纯跟踪控制器
        :param drive_line: 行驶路径 (drivex, drivey)
        :param vehicle_state: 包含车辆速度的字典 {'speed': m/s}（可选）
        :param debug: 调试模式开关
        :return: 标准化转向曲率（-1左转 ~ +1右转）
        """
        drivex, drivey = drive_line 
        vehicle_x, vehicle_y = vehicle_pt
    
        # 1. 确保路径点从近到远排序（越接近车辆当前位置）的点排在前面。
        sort_idx = np.argsort(drivey)[::-1]
        drivex, drivey = drivex[sort_idx], drivey[sort_idx]
    
        # 2. 只保留车辆前方的点
        forward_mask = drivey <= vehicle_y
        if not np.any(forward_mask):
            return 0, (vehicle_x, vehicle_y)
        drivex, drivey = drivex[forward_mask], drivey[forward_mask]

        # 3. 动态前视距离
        lookahead_distance = LOOKAHEAD_DISTANCE  # 调试时先用固定值

        # 4. 重新计算距离（排序和过滤后）
        distances = np.sqrt((drivex - vehicle_x)**2 + (drivey - vehicle_y)**2)
        closest_idx = np.argmin(distances)
    
        # 5. 搜索预瞄点（调试输出）
#        print(f"\nCurrent lookahead_distance: {lookahead_distance}")
#        print(f"Closest point distance: {distances[closest_idx]}")
    
        lookahead_idx = closest_idx
        while (lookahead_idx < len(drivey)-1 and 
            distances[lookahead_idx] < lookahead_distance):
            lookahead_idx += 1
    
#        print(f"Selected point index: {lookahead_idx}, y-value: {drivey[lookahead_idx]}")
    
        # 6. 强化曲率计算
        dx = (drivex[lookahead_idx] - vehicle_x) * LATERAR_GAIN
        dy = vehicle_x - drivey[lookahead_idx] 
    
        # 非线性曲率公式（对小偏差更敏感）
        curvature = 5.2 * dx / (dx**2 + dy**2)**0.75
    
        return np.clip(curvature, -1, 1), (drivex[lookahead_idx], drivey[lookahead_idx])

    def _create_visualization(self, img, leftx, lefty, rightx, righty, drive_line, vehicle_pt, lookahead_pt):
        """生成可视化结果图像
            传入参数：车道线像素点，行驶线像素点，车辆位置，前视点位置
        """
        # 创建三通道图像
        vis = np.dstack((img, img, img)) * 255 # 将二值图像转换为三通道图像
        
        # 绘制检测到的车道像素点（添加空数组检查）
        if len(leftx) > 0 and len(lefty) > 0:
            # 确保索引在有效范围内
            valid_left_mask = (lefty >= 0) & (lefty < vis.shape[0]) & (leftx >= 0) & (leftx < vis.shape[1])
            if np.any(valid_left_mask):
                vis[lefty[valid_left_mask].astype(int), leftx[valid_left_mask].astype(int)] = [255, 0, 0]  # 左线红色
        
        if len(rightx) > 0 and len(righty) > 0:
            # 确保索引在有效范围内
            valid_right_mask = (righty >= 0) & (righty < vis.shape[0]) & (rightx >= 0) & (rightx < vis.shape[1])
            if np.any(valid_right_mask):
                vis[righty[valid_right_mask].astype(int), rightx[valid_right_mask].astype(int)] = [0, 0, 255]  # 右线蓝色

        # 绘制行驶线的像素点（添加空数组检查）
        drivex, drivey = drive_line
        if len(drivex) > 0 and len(drivey) > 0:
            # 确保drivey和drivex的长度不超出vis的边界
            drivey_clipped = np.clip(drivey, 0, vis.shape[0] - 1)
            drivex_clipped = np.clip(drivex, 0, vis.shape[1] - 1)
            vis[drivey_clipped.astype(int), drivex_clipped.astype(int)] = [0, 255, 0]  # 绿色

        # 绘制基准线和前视点的像素点
        vehicle_x, vehicle_y = vehicle_pt
        cv2.circle(vis, (int(vehicle_x), int(vehicle_y)), 5, (0, 255, 255), -1)  # 黄色

        lookahead_pt_x, lookahead_pt_y = lookahead_pt
        cv2.circle(vis, (int(lookahead_pt_x), int(lookahead_pt_y)), 5, (255, 255, 0), -1)  # 紫色

        return vis


    def _draw_twist_info(self, img, twist, vehicle_pt,curvature=None):
        """在图像上正确绘制速度信息"""
        h, w = img.shape[:2]
        vis_img = img.copy()
    
        # 1. 绘制文字信息（保持不变）
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(vis_img, f"Vx: {twist.linear.x:.2f}m/s", (10,60), font, 0.7, (255,255,255), 2)
        cv2.putText(vis_img, f"Vy: {twist.linear.y:.2f}m/s", (10,90), font, 0.7, (255,255,255), 2)
        cv2.putText(vis_img, f"Wz: {twist.angular.z:.2f}rad/s", (10,120), font, 0.7, (255,255,255), 2)
        cv2.putText(vis_img, f"Curvature: {curvature:.2f}", (10,150), font, 0.7, (255,255,255), 2)
	
        # 2. 坐标转换（关键修正！）
        vx_pixel = int(twist.linear.x * 150)  # 缩放因子50像素/(m/s)
        vy_pixel = int(twist.linear.y * 150)
        center_x, center_y = int(vehicle_pt[0]), int(vehicle_pt[1])

        # 3. 分别绘制X/Y方向箭头（不同颜色）
        # X方向（前进/后退）- 绿色箭头

        end_y = center_y - vx_pixel
        cv2.arrowedLine(vis_img, 
                        (center_x, center_y),
                        (center_x, end_y),
                        (0, 255, 0), 2, tipLength=0.3)

        end_x = center_x + vy_pixel
        cv2.arrowedLine(vis_img,
                        (center_x, center_y),
                        (end_x, center_y),
                        (0, 0, 255), 2, tipLength=0.3)

        # 4. 旋转指示（红色圆弧）
        if abs(twist.angular.z) > 0.1:
            radius = 30
            start_angle = 0
            end_angle = int(180 * -twist.angular.z / 1.0)  # 取反
            cv2.ellipse(vis_img, 
                    (center_x, center_y),
                    (radius, radius), 0,
                    start_angle, end_angle,
                    (0, 0, 255), 2)

        return vis_img
        
        
# ================= 将图像处理和巡线集成 =================
class FindLaneLines:
    """图像处理与跟踪算法集成器
    主方法：forward(img, mode=None)
    输入值：img: 图像数据 mode: 巡线模式
    返回值：lane_image(鸟瞰图中检测到的车道线和行驶线), curvature(曲率), lookahead_pt(前视点), car_points(车辆基准位置), garage_corners(车库角点)
    """
    def __init__(self, lane_mode=0):
        #创建实例化
        self.lane_mode = lane_mode   #巡线模式，默认为1
        self.preprocess = PreProcess()
        self.find_lane = Lanedetection()
        # 初始化ROS图像发布器
        self.birdview_pub = rospy.Publisher('/debug/birdview', Image, queue_size=1)
        self.lane_pub = rospy.Publisher('/debug/lane_detection', Image, queue_size=1)
        self.bridge = CvBridge()  # 确保在类初始化时创建  

    def forward(self, img, mode=None):
        """处理主流程"""
        if mode is None:
            mode = self.lane_mode # 优先使用传入的mode，否则使用默认的lane_mode:1右线

        # 复制一张原图
        out_img = np.copy(img)

        # 1. 预处理获取二值化鸟瞰图和车辆基准位置
        img,car_points = self.preprocess.process(img)

        if DEBUG:
            birdview_msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
            self.birdview_pub.publish(birdview_msg)

        # 2. 将鸟瞰图和基准位置传入lanedetection中去检测车道线，以及角点，并用纯跟踪算法计算曲率和前视点,是否丢失标志lost_flag
        lane_image, curvature, lookahead_pt, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, distance = self.find_lane.detect(img, mode, car_points)

        if DEBUG:            
            # 发布车道检测结果
            lane_msg = self.bridge.cv2_to_imgmsg(lane_image, encoding="bgr8")
            self.lane_pub.publish(lane_msg)

            # # 逆透视变换并叠加到out_img上去
            # inv_perspective_image = self.preprocess.inv_PerspectiveTransform(lane_image)
            # # 将处理后的 inv_perspective_image 叠加到 out_img 上
            # final_output = cv2.addWeighted(out_img, 1, inv_perspective_image, 1, 0)
            
            # # 发布最终结果
            # final_msg = self.bridge.cv2_to_imgmsg(final_output, encoding="bgr8")
            # self.final_pub.publish(final_msg)

        return lane_image, curvature, lookahead_pt, car_pt_in_bev_image, garage_corners, lost_flag, garage_detected, distance

# ================= 处理图像，开始巡线 =================
def run_lane_detection(shared_state):
    """巡线子进程入口函数
    :param shared_state: 包含 flag_task 和 lane_mode 和shared_state.enable_event(Event)和shared_state.lock(Lock)
    """
    my_print("巡线跟踪进程启动，等待激活","info")
    rospy.init_node('lane_follow_node')
    try:
        # 初始化节点实例
        node = LaneDetectionNode(shared_state)     
        # 阻塞直到激活
        while not rospy.is_shutdown():
            shared_state.enable_event.wait(timeout=0.5)  # 带超时检测ROS关闭
            if rospy.is_shutdown(): 
                break     
            # 激活状态下运行
            node.run()
            
    except Exception as e:
	        rospy.logerr(f"车道进程崩溃: {str(e)}")
    finally:
        node.cleanup()  # 确保资源释放

class LaneDetectionNode:
    class State:
        NORMAL = 0      # 正常巡线模式
        PARKING = 1     # 停车入库模式
        COMPLETED = 2   # 任务完成模式

    def __init__(self, shared_state):
        """
        修改后版本：直接订阅摄像头Topic
        :param shared_state: 共享状态对象（仅包含控制标志）
        """
	
        # 保存引用
        self.shared_state = shared_state

        # 任务开始
        self.task_start_time = 0.0
        self.start_yaw = 0
        self.current_yaw = 0  #当前角度的初始值设置为0
        self.need_roundabout_switch_time = None

        # 图像相关状态
        self.current_frame = None  # 存储最新图像帧
        self.frame_lock = threading.Lock()  # 保护current_frame的线程锁
        self.last_frame_time = rospy.Time(0)  # 记录帧时间戳

		# 巡线方式和避障状态变量
        self.last_mode = -1  # 记录上次模式用于变化检测
        self.avoidance_active = False # 避障状态初始为false
        self.avoidance_lock = threading.Lock()  # 专用于避障状态
		
        self.has_avoided_obstacle = False  # 避障状态
        self.avoidance_history_lock = threading.Lock()  # 保护避障状态的锁

        # 双线丢失控制相关变量
        self.lost_line_forward_duration = None  # 默认前进持续时间(秒)
        self.lost_line_forward_speed = None     # 默认前进速度(m/s)
        self.is_forwarding_for_line = False    # 是否处于前进找线阶段
        self.forward_start_time = None         # 前进开始时间

        self.lost_line_start_time = None  # 丢线开始时间
        self.lost_line_rotation_duration = None  # 丢线后旋转默认持续时间（秒）
        self.lost_line_rotation_speed = None  # 丢线后旋转默认速度（m/s）
        self.is_rotating_for_line = False  # 是否正在为找线而旋转
        self.lost_line_lock = threading.Lock()  # 保护丢线状态的锁

        self.wait_start_time = None  # 新增：等待开始时间
        self.is_waiting_for_stabilization = False  # 新增：是否在等待稳定
        self.stabilization_wait_duration = 0.6  # 新增：稳定等待持续时间
                
        # 新增环岛状态机
        self.need_roundabout = NEED_ROUNDABOUT  # 是否需要环岛
        self.intersection_flag = 1   # 整个环岛过程一共会遇上四个路口，该标志位用于记录当前是第几个路口
        self.last_intersection_time = rospy.Time(0)  # 记录上次更新intersection_flag的时间
        self.angular_speed  = 0.8
        
		# ROS组件初始化
        self._init_ros_components()

		# 算法模块
        self.follow_line = FindLaneLines()
        self.calculate = calculate_speed()

        self.expected_shape = (360, 640, 3)  # 根据实际调整

        # 状态管理
        self.current_state = self.State.NORMAL
        self.parking_completed = False    # 状态机需要的标志
        self.parking_finish_flag = False  # 从停车控制函数返回的标志

        # 状态锁，保护状态变量
        self.state_lock = threading.Lock()
        
        	

    def _init_ros_components(self):
        """初始化ROS通信组件"""
        self.line_follow_pub = rospy.Publisher(
            '/line_follow_cmd', 
            Twist, 
            queue_size=4, # 增大队列防止丢指令
            latch=True,
            tcp_nodelay=True
        )
        rospy.Subscriber(
            '/avoidance_status',
            Bool,
            self._avoidance_callback,
            queue_size=10
        )
        self.bridge = CvBridge()  # 用于OpenCV->ROS图像转换
        
        # 新增摄像头订阅者
        self.image_sub = rospy.Subscriber(
            "/ucar_camera/image_raw",  # 根据实际话题名称修改
            Image,
            self._image_callback,
            queue_size=1,  # 只保留最新帧
            buff_size=2**24,  # 大缓冲区防止丢帧
            tcp_nodelay=True
        )
        # IMU订阅器（用于获取当前朝向）
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)


    def _image_callback(self, msg):
        """摄像头数据回调（线程安全）"""
        try:


            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 缩放到指定大小
            cv_image = cv2.resize(cv_image, (640, 360))
            # 使用锁保护当前帧
            with self.frame_lock:
                self.current_frame = cv_image
                self.last_received_time = time.time()  # 记录接收时间
                
        except Exception as e:
            rospy.logerr(f"图像转换失败: {str(e)}")

    def imu_callback(self, msg):
        """IMU回调函数，获取多种信息"""
        
        # 1. 姿态信息 (Orientation) - 用于旋转控制
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def _avoidance_callback(self, msg):
        """线程安全的避障状态回调"""
        with self.avoidance_lock:
            previous_state = self.avoidance_active
            self.avoidance_active = msg.data
            
            # 检测避障状态变化：从激活变为非激活，说明避障完成
            if previous_state and not msg.data:
                with self.avoidance_history_lock:
                    self.has_avoided_obstacle = True
                    rospy.loginfo("避障完成，已记录避障历史")
                    
        rospy.loginfo_throttle(1.0, 
            f"避障状态: {'激活' if msg.data else '空闲'}")
    
    def _get_avoidance_history(self):
        """线程安全获取避障历史状态"""
        with self.avoidance_history_lock:
            return self.has_avoided_obstacle

    def run(self):
        """主运行循环（带双重阻塞控制）"""
        # 添加事件状态检查
        if not self.shared_state.enable_event.is_set():
            return     
        my_print("阻塞激活，node.run循环开始")
        mode = -1
        start_time = time.time()

        self.task_start_time = rospy.Time.now()
        # 打印任务开始时间
        my_print(f"任务开始时间: {self.task_start_time}")
        # 注意：rotation>0表示向左转，rotation<0表示向右转
        # 选拔赛阶段，将第一个路口（直行+平移+旋转）的参数设置为预定义值
        if not NEED_ROUNDABOUT:
            param_key = 1.0 if self.shared_state.lane_mode.value == 0 else 1.1
            params = INTERSECTION_PARAMS[param_key]
            self.set_lost_line_forward_params(*params)

        # 国赛阶段，将第一个路口(直行+平移+旋转)的参数设置为预定义值
        if NEED_ROUNDABOUT:
            param_key = 1.0 if self.shared_state.lane_mode.value == 0 else 1.1
            params = FINAL_INTERSECTION_PARAMS[param_key]
            self.set_lost_line_forward_params(*params)
            

        # 等待首帧数据（超时10秒）
        while not rospy.is_shutdown() and time.time() - start_time < 10.0:
            with self.frame_lock:
                if self.current_frame is not None:
                    break
            rospy.loginfo_throttle(1.0, "等待首帧摄像头数据...")
            time.sleep(0.1)
    
        if rospy.is_shutdown() or self.current_frame is None:
            rospy.logerr("摄像头数据订阅超时！")
            return

        rate = rospy.Rate(30)  # 30Hz
        time.sleep(1.5) # 
        recorded_angle = False  # 添加标志位，确保角度只被记录一次
       
        while not rospy.is_shutdown():

            # 检查避障状态，如果正在避障则暂停处理
            if self._get_avoidance_status():
                rospy.loginfo_throttle(2.0, "避障进行中，巡线算法暂停")
                rate.sleep()
                continue

            # 检查是否任务完成
            with self.state_lock:
                if self.current_state == self.State.COMPLETED:
                    rospy.loginfo_throttle(1.0, "任务已完成，程序关闭...")
                    rospy.loginfo_throttle(2.0, "任务完成，准备关闭程序")
                    ret_pub = rospy.Publisher('/laneline/result', String, queue_size=10)
                    # 直接退出进程
                    msg=String()
                    msg.data="end"
                    rate=rospy.Rate(10)
                    for i in range(0,100):
                        rate.sleep()
                        ret_pub.publish(msg)
                        rospy.logwarn("send end")
                        rate.sleep()
                    continue

            # 模式切换处理
            if mode != self.last_mode:
                rospy.loginfo(f"巡线模式变更: {self.last_mode} → {mode}")
                self.last_mode = mode
            
            # 合并条件检查与帧获取（原子操作）
            frame = self._get_valid_frame()
            if frame is None:
                if not self._check_subscription_active():
                    rospy.logerr("摄像头数据流中断！")
                    break
                rate.sleep()
                continue

            # 处理当前帧
            self._process_frame(self.shared_state.lane_mode.value,frame)
            rate.sleep()

    def _check_subscription_active(self):
        """检查摄像头订阅是否活跃（最近5秒内有数据到达）"""
        with self.frame_lock:
            if self.last_received_time is None:
                return False
            return (time.time() - self.last_received_time) < 5.0  # 5秒超时            
            
    def _get_valid_frame(self):
        """线程安全获取有效帧（返回None表示无数据）"""
        with self.frame_lock:
            # 同时检查使能状态和帧数据
            if (not self.shared_state.enable_event.is_set() or 
                self.current_frame is None):
                return None
            return self.current_frame.copy()    

    def _get_avoidance_status(self):
        """线程安全获取避障状态"""
        with self.avoidance_lock:
            return self.avoidance_active

    def _process_frame(self, mode, frame):
        """基于状态机的帧处理流程"""
        try:
            # 图像有效性检查
            if not self._validate_frame(frame):
                return
                
            start_time = time.time()

            # Step 1. 获取算法结果
            result = self.follow_line.forward(frame, mode)
            lane_image, curvature, lookahead_pt, vehicle_pt, garage_corners, lost_flag, garage_detected, distance = result    
                
            # Step 2. 状态机处理
            with self.state_lock:
                self._handle_state_machine(
                    mode, lost_flag, garage_detected, 
                    curvature, lookahead_pt, vehicle_pt, garage_corners, int(distance)
                )

        except Exception as e:
            import traceback

            rospy.logerr_throttle(1.0, f"帧处理失败: {str(e)}")

            error_msg = f"帧处理失败: {str(e)}"
            stack_trace = traceback.format_exc()
            
            rospy.logerr_throttle(1.0, error_msg)
            rospy.logerr_throttle(1.0, f"完整错误追踪:\n{stack_trace}")
            self._publish_zero_velocity()

    def _handle_state_machine(self, mode, lost_flag, garage_detected, 
                            curvature, lookahead_pt, vehicle_pt, garage_corners, distance):
        """状态机核心逻辑"""
   
        # 记录状态变化
        old_state = self.current_state
        
        # 状态转换逻辑

        # 正常巡线模式
        if self.current_state == self.State.NORMAL: 
            # 修改后的条件：检测到车库 且 已经避障过
            if garage_detected and self._get_avoidance_history():
            #if self._get_avoidance_history():             
                rospy.loginfo("检测到车库且已完成避障，切换到停车模式")
                self.current_state = self.State.PARKING # 切换到停车模式
            elif garage_detected and not self._get_avoidance_history():
                # 检测到车库但还没避障过，记录日志但不切换状态
                rospy.loginfo_throttle(1.0, "检测到车库，但尚未完成避障，继续巡线...")

        # 停车模式
        elif self.current_state == self.State.PARKING: 
            if self.parking_finish_flag: # 停车控制器所返回的标志位
                rospy.loginfo("停车完成，任务结束")
                self.current_state = self.State.COMPLETED
                self.parking_completed = True # 设置停车完成标志（状态机标志）

        # 任务完成状态
        elif self.current_state == self.State.COMPLETED:
            # 任务完成状态，关闭程序
            pass
        
        # 状态变化日志
        if old_state != self.current_state:
            rospy.loginfo(f"状态切换: {self._state_to_string(old_state)} -> {self._state_to_string(self.current_state)}")
        
        # 根据当前状态执行相应控制
        self._execute_state_control(mode, lost_flag, garage_detected, 
                                  curvature, lookahead_pt, vehicle_pt, garage_corners, distance)

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
        target_yaw = self.normalize_angle(target_yaw) - 10  
        
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
            
            self.line_follow_pub.publish(twist)
            rate.sleep()
        self._publish_zero_velocity()     
        rospy.loginfo(f"旋转完成，当前角度: {math.degrees(self.current_yaw):.2f}°")


    def normalize_angle(self, angle):
        """ 将角度标准化到[-π, π] """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle 

    def _execute_state_control(self, mode, lost_flag, garage_detected, 
                            curvature, lookahead_pt, vehicle_pt, garage_corners, distance):
        """根据当前状态执行相应的控制逻辑"""
        # 状态机为NORMAL时
        if self.current_state == self.State.NORMAL:
            try:
                global INROUNDABOUT  # 声明使用全局变量
                global NEED_ROUNDABOUT  # 声明使用全局变量
                global LOOKAHEAD_DISTANCE  # 声明使用全局变量

                # 获取丢线状态信息
                lost_status = self._get_lost_line_status()
                ready_park = self._get_avoidance_history()
                
                # 国赛（环岛）处理逻辑
                if NEED_ROUNDABOUT:

                    # 检查距离上次intersection_flag更新的时间是否超过2秒
                    current_time = rospy.Time.now()
                    time_since_start = (current_time - self.task_start_time).to_sec() 
                    time_since_last_intersection = (current_time - self.last_intersection_time).to_sec()
                    yaw_diff = self.current_yaw - self.start_yaw
                    can_trigger_control = time_since_last_intersection >= 2.7 and time_since_start >=7
                    
                    if (lost_flag or lost_status['is_lost']) and can_trigger_control: 
                        # 打印time_since_start
                        my_print(f"Time since start: {time_since_start:.2f}") 

                        if self.intersection_flag in [2, 3, 4] or (self.intersection_flag == 1):  # 都是使用先直行再旋转
                            cmd = self._handle_lost_line_situation(mode)
                            # 打印 time_since_last_intersection
                            rospy.loginfo(f"距离上次路口更新时间: {time_since_last_intersection:.2f} 秒")
                            # 如果在等待稳定阶段且找到线路
                            if lost_status['is_waiting'] and not lost_flag:
                                with self.lost_line_lock:
                                    rospy.loginfo("等待稳定完成且重新找到线路，恢复正常巡线")
                                    self._reset_lost_line_status()

                                self.intersection_flag += 1
                                INROUNDABOUT = 1  # 更新标志位，表示正在环岛
                                self.last_intersection_time = rospy.Time.now()  # 记录更新时间
                                rospy.logwarn(f"更新路口编号为: {self.intersection_flag}")
                                
                                # 根据intersection_flag和mode选择参数
                                if self.intersection_flag == 1:
                                    # 第1个路口根据mode选择参数
                                    param_key = 1.0 if mode == 0 else 1.1
                                    params = FINAL_INTERSECTION_PARAMS[param_key]
                                elif self.intersection_flag == 4:
                                    # 第4个路口使用选拔赛的参数
                                    param_key = 1.0 if mode == 0 else 1.1
                                    params = INTERSECTION_PARAMS[param_key]
                                    LOOKAHEAD_DISTANCE *= (10/6)  # 增大LOOKAHEAD_DISTANCE
                                    NEED_ROUNDABOUT = 0  # 完成环岛，切换到选拔赛模式
                                    self.need_roundabout_switch_time = rospy.Time.now()  # 记录NEED_ROUNDABOUT变为0的时间
                                    my_print("第4个路口，增大LOOKAHEAD_DISTANCE, 切换到选拔赛模式")
                                else:
                                    # 第2、3个路口直接使用对应参数
                                    params = FINAL_INTERSECTION_PARAMS[self.intersection_flag]
                                    if self.intersection_flag == 3:
                                        # 第3个路口需要减小LOOKAHEAD_DISTANCE
                                        LOOKAHEAD_DISTANCE *= 0.6
                                        my_print("第3个路口，减小LOOKAHEAD_DISTANCE") 
                                # 设置参数
                                self.set_lost_line_forward_params(*params)
                    else:
                        # 正常巡线控制
                        cmd = self.calculate.calculate_control(
                            curvature=curvature,
                            lookahead_pt=lookahead_pt,
                            vehicle_pt=vehicle_pt,
                            ready_park=ready_park
                        )

                # 选拔赛(非环岛)处理逻辑
                else:
                    current_time = rospy.Time.now() 
                    yaw_diff = self.current_yaw - self.start_yaw

                    if self.need_roundabout_switch_time is None:
                        time_since_start = (current_time - self.task_start_time).to_sec()
                        can_trigger_control = time_since_start >= 7.0 
                    else:
                        time_since_roundabout_switch = (current_time - self.need_roundabout_switch_time).to_sec() 
                        time_since_start = (current_time - self.task_start_time).to_sec()
                        can_trigger_control = time_since_start >= 7.0 and time_since_roundabout_switch > 2.0 
                        
                    #打印当前位姿self.current_yaw与self.start_yaw的差值
#                    my_print(f"当前位姿与起始位姿的差值: {self.current_yaw - self.start_yaw:.2f}")                                                   
                    if (lost_flag or lost_status['is_lost']) and can_trigger_control:  
                        if lost_flag or lost_status['is_lost']:
                            # 打印time_since_start
                            my_print(f"Time since start: {time_since_start:.2f}")
                                                
                            # 直接调用处理函数自动处理前进和旋转阶段
                            cmd = self._handle_lost_line_situation(mode)
                            
                            # 如果在等待稳定阶段且找到线路
                            if lost_status['is_waiting'] and not lost_flag:
                                with self.lost_line_lock:
                                    rospy.loginfo("等待稳定完成且重新找到线路，恢复正常巡线")
                                    self._reset_lost_line_status()
                                    # 切换巡线模式
                                    if self.shared_state.lane_mode.value == 0 and not self.need_roundabout:
                                        self.shared_state.lane_mode.value = 1
                                        my_print("巡线模式切换： 左 变为 右")
                                    elif self.shared_state.lane_mode.value == 1 and not self.need_roundabout:
                                        self.shared_state.lane_mode.value = 0
                                        my_print("巡线模式切换： 右，变为 左")    
              
                    else:
                        # 正常巡线控制
                        cmd = self.calculate.calculate_control(
                            curvature=curvature,
                            lookahead_pt=lookahead_pt,
                            vehicle_pt=vehicle_pt,
                            ready_park=ready_park
                        )

                self.line_follow_pub.publish(cmd)
                
            except Exception as e:
                rospy.logerr(f"正常巡线控制失败: {str(e)}")
                self._publish_zero_velocity()
        
        # 状态机为停车模式
        elif self.current_state == self.State.PARKING:
            try:
                cmd, finish_flag = self.calculate.perform_garage_parking_control(distance)
                self.line_follow_pub.publish(cmd)

                self.parking_finish_flag = finish_flag

            except Exception as e:
                rospy.logerr(f"停车控制失败: {str(e)}")
                self._publish_zero_velocity()
                
         # 任务完成模式
        elif self.current_state == self.State.COMPLETED:
            try:
                # 创建一个发布者用于发送结束消息
                ret_pub = rospy.Publisher('/laneline/result', String, queue_size=10)
                rate = rospy.Rate(10)  # 10Hz的发布频率
                
                # 第二步：循环发布零速度停车
                rospy.loginfo("开始发布零速度停车指令")
                self._publish_zero_velocity()
                # 发布结束消息
                msg = String()
                msg.data = "end"
                ret_pub.publish(msg)
                rospy.logwarn_throttle(2.0, "发送结束消息")
                rate.sleep()
                    
            except Exception as e:
                rospy.logerr(f"任务完成控制失败: {str(e)}")
                self._publish_zero_velocity()
            finally:
                # 触发程序关闭
                self._trigger_shutdown()
 

    def _handle_roundabout_situation(self, mode):
        """
        先旋转再直行
        仅用于处理国赛中岔路口1，2的特殊控制逻辑，先旋转在直行"""
        current_time = time.time()
        cmd = Twist()
	
        with self.lost_line_lock:
            # 如果刚开始丢线，记录开始时间并进入旋转阶段
            if not (self.is_forwarding_for_line or self.is_rotating_for_line or self.is_waiting_for_stabilization):
                self.rotation_start_time = current_time
                self.is_rotating_for_line = True
                rospy.logwarn("这里是环岛岔路口1，2的特殊过渡控制")

            # 第一阶段：旋转
            if self.is_rotating_for_line:
                rotation_elapsed = current_time - self.rotation_start_time
                
                if rotation_elapsed < self.lost_line_rotation_duration:
                    # 旋转阶段控制
                    cmd.linear.x = 0.0
                    
                    # 根据巡线模式确定旋转方向
                    if mode == 0:  # 左线模式
                        cmd.angular.z = - self.lost_line_rotation_speed  # 向右转
                    elif mode == 1:  # 右线模式
                        cmd.angular.z =  self.lost_line_rotation_speed  # 向左转
                    rospy.loginfo_throttle(0.5, 
                        f"旋转搜索中... {rotation_elapsed:.1f}s/{self.lost_line_rotation_duration}s")
                else:
                    # 旋转时间结束，切换到前进阶段
                    self.is_rotating_for_line = False
                    self.is_forwarding_for_line = True
                    self.forward_start_time = current_time
                    rospy.loginfo("旋转完成，开始前进搜索")

            # 第二阶段：前进
            elif self.is_forwarding_for_line:
                forward_elapsed = current_time - self.forward_start_time
                
                if forward_elapsed < self.lost_line_forward_duration:
                    # 前进阶段控制
                    cmd.linear.x = self.lost_line_forward_speed
                    cmd.angular.z = 0.0
                    rospy.loginfo_throttle(0.5, 
                        f"前进中... {forward_elapsed:.1f}s/{self.lost_line_forward_duration}s")
                else:
                    # 前进时间结束，进入等待稳定阶段
                    if not self.is_waiting_for_stabilization:
                        self.is_waiting_for_stabilization = True
                        self.wait_start_time = current_time
                        rospy.loginfo("前进搜索完成，开始等待稳定")
                    
                    # 等待稳定期间停止运动
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

        return cmd

    def _handle_lost_line_situation(self, mode):
        """先直行再旋转
        选拔赛中用于处理岔路口1的特殊控制逻辑，先直行再旋转
        国赛中用于处理到达岔路口1，2，3，4的特殊控制逻辑"""
        current_time = time.time()
        cmd = Twist()
	
        with self.lost_line_lock:
            # 如果刚开始丢线，记录开始时间
            # 如果刚开始丢线，记录开始时间并进入前进阶段
            if not (self.is_forwarding_for_line or self.is_rotating_for_line or self.is_waiting_for_stabilization):
                self.forward_start_time = current_time
                self.is_forwarding_for_line = True
                rospy.logwarn("开始前进")

            # 第一阶段：前进
            if self.is_forwarding_for_line:
                forward_elapsed = current_time - self.forward_start_time
                
                if forward_elapsed < self.lost_line_forward_duration:
                    # 前进阶段控制
                    cmd.linear.x = self.lost_line_forward_speed
                    if mode == 0 :         
                        cmd.linear.y = self.lost_line_translation_speed     # 向左平移
                        cmd.angular.z = self.lost_line_rotate_speed        # 向左转
                    elif mode == 1:  
                        cmd.linear.y = -self.lost_line_translation_speed    # 向右平移
                        cmd.angular.z = - self.lost_line_rotate_speed     # 向右转

                    rospy.loginfo_throttle(0.5, 
                        f"前进/平移中... {forward_elapsed:.1f}s/{self.lost_line_forward_duration}s")
                else:
                    # 前进时间结束，切换到旋转阶段
                    self.is_forwarding_for_line = False
                    self.is_rotating_for_line = True
                    self.lost_line_start_time = current_time
                    rospy.loginfo("前进搜索完成，开始旋转")

            # 第二阶段：旋转
            elif self.is_rotating_for_line:
                rotation_elapsed = current_time - self.lost_line_start_time
                
                if rotation_elapsed < self.lost_line_rotation_duration:
                    # 旋转阶段控制
                    cmd.linear.x = 0.0 
                    
                    # 根据巡线模式确定旋转方向
                    if mode == 0:  # 左线模式
                        cmd.angular.z = self.lost_line_rotation_speed  # 向左转
                    elif mode == 1:  # 右线模式
                        cmd.angular.z = - self.lost_line_rotation_speed  # 向右转                        
                    rospy.loginfo_throttle(0.5, 
                        f"强制旋转搜索中... {rotation_elapsed:.1f}s/{self.lost_line_rotation_duration}s")
                else:
                    # 旋转时间已到，进入等待稳定阶段
                    if not self.is_waiting_for_stabilization:
                        self.is_waiting_for_stabilization = True
                        self.wait_start_time = current_time
                        rospy.loginfo("强制旋转完成，开始等待稳定")
                    
                    # 等待稳定期间停止运动
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

        return cmd

    def _handle_stabilization_wait(self):
        """处理等待稳定阶段的控制逻辑"""
        cmd = Twist()
        cmd.linear.x = 0.0  # 停止移动
        cmd.angular.z = 0.0  # 停止旋转
        return cmd

    def set_stabilization_wait_duration(self, duration):
        """设置稳定等待持续时间"""
        with self.lost_line_lock:
            self.stabilization_wait_duration = max(0.1, duration)  # 最小0.1秒
            rospy.loginfo(f"稳定等待持续时间设置为: {self.stabilization_wait_duration}s") 

    def set_lost_line_forward_params(self, forward_duration, translation_speed, rotate_speed, forward_speed, rotation_duration, rotation_speed, ):
        """设置_handle_lost_line_situation参数
        包括：前进时间段的平移速度，前进速度以及旋转时间，旋转速度
        rotation>0表示向左转，rotation<0表示向右转
        """
        with self.lost_line_lock:
            self.lost_line_forward_duration = max(0.1, forward_duration)  # 最小0.1秒
            self.lost_line_translation_speed = translation_speed # 第一阶段的平移速度
            self.lost_line_rotate_speed = rotate_speed # 第一阶段的旋转速度
            self.lost_line_forward_speed = forward_speed # 第一阶段的直线速度

            self.lost_line_rotation_duration = max(0.0, rotation_duration)  # 最小0.5秒
            self.lost_line_rotation_speed = rotation_speed # 第二阶段的旋转速度

            rospy.loginfo(f"环岛参数设置: 前进时间={forward_duration}s, 速度={forward_speed}m/s, "
                        f"旋转时间={rotation_duration}s, 速度={rotation_speed}rad/s")

    def set_roundabout_params(self, forward_duration, forward_speed, rotation_duration, rotation_speed):
        """设置_handle_roundabout_situation处理参数"""
        with self.lost_line_lock:
            self.lost_line_forward_duration = max(0.1, forward_duration)
            self.lost_line_forward_speed = forward_speed
            self.lost_line_rotation_duration = max(0.5, rotation_duration)
            self.lost_line_rotation_speed = rotation_speed
            rospy.loginfo(f"环岛参数设置: 前进时间={forward_duration}s, 速度={forward_speed}m/s, "
                        f"旋转时间={rotation_duration}s, 速度={rotation_speed}rad/s")


    def _reset_lost_line_status(self):
        """重置所有丢线相关状态标志"""
        self.is_forwarding_for_line = False
        self.is_rotating_for_line = False
        self.is_waiting_for_stabilization = False
        self.forward_start_time = None
        self.lost_line_start_time = None
        self.wait_start_time = None

    def _get_lost_line_status(self):
        """获取丢线状态信息(包含前进阶段)"""
        with self.lost_line_lock:
            if (self.is_rotating_for_line or self.is_forwarding_for_line or 
            self.lost_line_start_time or self.forward_start_time):
                current_time = time.time()
                            
                status = {
                    'is_lost': True,
                    'is_forwarding': self.is_forwarding_for_line,
                    'is_rotating': self.is_rotating_for_line,
                    'is_waiting': self.is_waiting_for_stabilization
                }
                
                # 前进阶段信息
                if self.is_forwarding_for_line and self.forward_start_time:
                    forward_elapsed = current_time - self.forward_start_time
                    status.update({
                        'forward_elapsed': forward_elapsed,
                        'forward_duration': self.lost_line_forward_duration
                    })
                
                # 旋转阶段信息
                if self.is_rotating_for_line and self.lost_line_start_time:
                    rotation_elapsed = current_time - self.lost_line_start_time
                    status.update({
                        'rotation_elapsed': rotation_elapsed,
                        'rotation_duration': self.lost_line_rotation_duration
                    })
                
                # 等待阶段信息
                if self.is_waiting_for_stabilization and self.wait_start_time:
                    wait_elapsed = current_time - self.wait_start_time
                    status.update({
                        'wait_elapsed': wait_elapsed,
                        'wait_duration': getattr(self, 'stabilization_wait_duration', 0.5)
                    })
                
                return status
            else:
                return {'is_lost': False, 'is_forwarding': False, 'is_rotating': False, 'is_waiting': False}


    def _validate_frame(self, frame):
        """验证帧的有效性"""
        if not isinstance(frame, np.ndarray):
            rospy.logerr("帧数据不是numpy数组")
            return False
        if frame.size == 0:
            rospy.logwarn("收到空帧")
            return False
        if frame.shape != self.expected_shape:
            rospy.logerr(f"图像尺寸异常！期望{self.expected_shape}，实际{frame.shape}")
            return False
        return True

    def _trigger_shutdown(self):
        """触发程序关闭"""
        if not self.parking_completed:
            return
            
        rospy.loginfo("停车任务完成，5秒后关闭程序...")
        
        # 延迟关闭，给系统时间完成最后的操作
        def delayed_shutdown():
            time.sleep(5.0)
            rospy.loginfo("程序正常关闭")
            # 设置共享状态标志
            # 关闭ROS节点
            rospy.signal_shutdown("Mission Completed Successfully")
        
        # 在新线程中执行延迟关闭
        shutdown_thread = threading.Thread(target=delayed_shutdown)
        shutdown_thread.daemon = True
        shutdown_thread.start()

    def _state_to_string(self, state):
        """状态枚举转字符串"""
        state_names = {
            self.State.NORMAL: "正常巡线",
            self.State.PARKING: "停车入库",
            self.State.COMPLETED: "任务完成"
        }
        return state_names.get(state, f"未知状态({state})")

    def _publish_zero_velocity(self):
        """发布零速度指令（安全停止）"""
        cmd = Twist()
        self.line_follow_pub.publish(cmd)

    def cleanup(self):
        """资源释放"""
        if hasattr(self, 'line_follow_pub'):
            self.line_follow_pub.unregister()



# ================= 底盘控制进程 =================
def run_chassis_control(shared_state, global_lock):
    """底盘控制进程入口"""
    my_print("底盘控制进程启动，等待激活", "info")
    rospy.init_node('chassis_controller_node')
    
    controller = ChassisController(shared_state, global_lock)
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        if not shared_state.enable_event.wait(timeout=0.1):
            continue
            
        try:
            # 安全检查（带锁访问共享变量flag_task）
            with global_lock:
                if not shared_state.flag_task.value:
                    controller._publish_zero_velocity()
                    continue
                    
            # 主控制流程
            current_cmd, src = controller._select_active_cmd()
            controller._publish_cmd(current_cmd, src)
                
        except Exception as e:
            rospy.logerr_throttle(1.0, f"控制异常: {str(e)}")
            with global_lock:
                shared_state.enable_event.clear()
                
        rate.sleep()

class ChassisController:
    def __init__(self, shared_state, global_lock):
        self.shared_state = shared_state
        self.cmd_lock = global_lock

        self.last_line_cmd = Twist()
        self.last_avoidance_cmd = Twist()

        self.avoidance_status = Value('b', False)  # 使用原子变量替代锁

        # ROS初始化
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/line_follow_cmd', Twist, self._line_cmd_cb)
        rospy.Subscriber('/avoidance_cmd', Twist, self._avoidance_cmd_cb)
        rospy.Subscriber('/avoidance_status', Bool, self._avoidance_status_cb)

    def _line_cmd_cb(self, msg):
        with self.cmd_lock:# 带锁访问
            self.last_line_cmd = msg

    def _avoidance_cmd_cb(self, msg):
        with self.cmd_lock:
            self.last_avoidance_cmd = msg

    def _avoidance_status_cb(self, msg):
        self.avoidance_status.value = msg.data
        # 当避障结束时，清除残留的避障指令
        if not msg.data:
            with self.cmd_lock:
                self.last_avoidance_cmd = Twist()

    def _select_active_cmd(self):
        with self.cmd_lock:
            # 综合状态标志和指令有效性
            if self.avoidance_status.value:
                return self.last_avoidance_cmd, "特殊控制（环岛、避障）"
            return self.last_line_cmd, "巡线"

    def _publish_cmd(self, cmd, src):
        self.cmd_pub.publish(cmd)
#        rospy.loginfo_throttle(1.0, f"目前速度控制源: {src} 速度: {cmd.linear.x:.2f}m/s")
        
class LowPassFilter:
    """一阶低通滤波器"""
    def __init__(self, time_constant):
        self.time_constant = time_constant  # 时间常数，越大滤波效果越强
        self.last_value = None
        
    def filter(self, current_value, dt):
        if self.last_value is None:
            self.last_value = current_value
            return current_value
            
        alpha = dt / (self.time_constant + dt)
        filtered_value = self.last_value + alpha * (current_value - self.last_value)
        self.last_value = filtered_value
        return filtered_value


# ================= 解析计算小车速度并返回twist =================
class calculate_speed:
    """
       有三种控制行为：巡线和恢复和停车控制，需要根据车道线是否有效/是否检测到车库角点来选择行为
       包含根据前视点和曲率计算巡线速度twist的函数calculate_control
       以及控制恢复行为计算twist的函数generate_recovery_cmd
       以及控制停车行为的函数perform_garage_parking_control
    """
    def __init__(self):
        # ========== 可调参数 ==========
        half_track_width = TRACK_WIDTH / 2  # 车道宽度的一半：0.195 米
        self.pixel_to_meter = half_track_width / MOVE_OFFSET  # 0.195 / 146

        # 速度限制
        self.max_angular_speed = MAX_ANGULAR_SPEED  # 最大角速度
        self.min_angular_speed = MIN_ANGULAR_SPEED  # 最小角速度

        self.max_linear_speed = MAX_LINEAR_SPEED  # 最大线速度
        self.min_linear_speed = MIN_LINEAR_SPEED  # 最小线速度
        
        self.parking_speed = 0.13 #入库速度

        # 转向控制参数
        self.kp_yaw = KP_YAW            # 航向P增益
        self.kd_yaw = KD_YAW            # 航向D增益
        self.ki_yaw = KI_YAW            # 航向I增益（抗稳态误差）
        self.max_integral_error = 1.0
        self.min_integral_error = 0.0      
        self.integral_yaw_error = 0.0    # 航向误差积分项

        self.yaw_error_threshold = YAW_ERROR_THRESHOLD  # 航向误差死区(rad)，如果觉得微小的误差影响不大，可以适当调大，反之可以调小
        self.last_yaw_error = 0.0

        # 曲率适应参数
        self.curvature_speed_factor = CURVATURE_SPEED_FACTOR  # 曲率对速度的影响系数（系数越大前向速度会越小）

        # 速度平滑参数
        self.max_acceleration = 0.5  # 最大加速度 m/s²
        self.max_deceleration = 2.0  # 最大减速度 m/s²
        self.last_linear_speed = 0.0
        
        # 创建低通滤波器，时间常数为0.2秒
        self.speed_filter = LowPassFilter(time_constant=0.1)
        self.last_time = rospy.Time.now()

    def calculate_control(self, curvature, lookahead_pt, vehicle_pt, ready_park):
        twist = Twist()
        
        # 先判断前视点是否为空，如果为空则不进行控制直接返回一个零速度
        if not lookahead_pt:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return twist

        try:
            # === 1. 前向速度计算（曲率自适应） ===
            speed_factor = 1.0 - self.curvature_speed_factor * min(abs(curvature), 1.0)
            target_speed = np.clip(
                self.max_linear_speed * speed_factor,
                self.min_linear_speed,
                self.max_linear_speed
            )

            # 计算时间间隔
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # === 2. 速度平滑处理 ===
            
            # 2.1 变化率限制
            speed_diff = target_speed - self.last_linear_speed
            if speed_diff > 0:  # 加速
                max_change = self.max_acceleration * dt
                speed_diff = min(speed_diff, max_change)
            else:  # 减速
                max_change = self.max_deceleration * dt
                speed_diff = max(speed_diff, -max_change)
            
            limited_speed = self.last_linear_speed + speed_diff
            
            # 2.2 低通滤波
            filtered_speed = self.speed_filter.filter(limited_speed, dt)
            
            # 最终速度赋值
            twist.linear.x = filtered_speed
            self.last_linear_speed = filtered_speed

            # === 3. 角速度控制 ===
            lx, ly = lookahead_pt
            vx, vy = vehicle_pt

            dx = lx - vx  # 横向距离（图像x方向）
            dy = vy - ly  # 前向距离（图像y方向，注意y向下为正）
            
            # 计算目标偏航角（相对于图像y轴向上方向）
            # 当前车辆航向假设为沿y轴向上方向（0度）
            # 前视点的目标角度就是相对于y轴向上的偏转角
            target_yaw = np.arctan2(dx, dy)  # 注意这里是 arctan2(x, y)
            
            # 当前航向角为0（沿y轴向上），所以航向误差就是目标角度
            yaw_error = - target_yaw
            
            # 死区处理
            if abs(yaw_error) < self.yaw_error_threshold:
                yaw_error = 0.0
                
            # PID计算
            error_diff = yaw_error - self.last_yaw_error
            self.integral_yaw_error += yaw_error
            
            # 积分项限幅
            self.integral_yaw_error = np.clip(
                self.integral_yaw_error,
                -self.max_integral_error,
                self.max_integral_error
            )
            
            # 计算角速度控制量
            angular_control = (
                self.kp_yaw * yaw_error +
                self.kd_yaw * error_diff +
                self.ki_yaw * self.integral_yaw_error
            )
            
            # 限幅输出
            twist.angular.z = np.clip(angular_control, -self.max_angular_speed, self.max_angular_speed)
            
            # 记录当前误差用于下次计算
            self.last_yaw_error = yaw_error
            
            if ready_park:
                # 避障之后即将进入停车入库，需要将速度降低
                twist.linear.x = 0.20
                twist.angular.z *= 0.8
            # 添加日志打印，每秒打印一次
            rospy.loginfo_throttle(1.0, 
                "curvature=%.2f  lin.x=%.2f m/s, ang.z=%.2f rad/s | Yaw error: %.3f rad (%.1f°)", 
                curvature, twist.linear.x, twist.angular.z,
                yaw_error, math.degrees(yaw_error))

        except Exception as e:
            rospy.logerr("Control calculation error: %s", str(e))
            twist = Twist()  # 发生异常时停止
            
        return twist

    #================= 停车入库控制器 =================
    def perform_garage_parking_control(self, distance):
        """停车入库控制器,只有当左右车库角点都存在时才使用停车入库控制"""
        try:
            stop_distance = STOP_DISTANCE   # 停车距离阈值
            # 改进的停车判断条件：
            # 1. distance <= stop_distance 并且 不等于0
            if (distance <= stop_distance and distance != 0):
                
                my_print(f"满足停车条件 - 距离ang.z=0.00: {distance:.2f}")
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                return stop_cmd,  True
		
		
            else:
                if (distance > stop_distance and distance != 0):
                    self.parking_speed = 0.07
                #my_print(f"不满足停车条件 - 距离: {distance:.2f}")
                cmd = Twist()
                cmd.linear.x = self.parking_speed # 缓慢继续前进
                cmd.angular.z = 0.0
            return cmd,  False
            
        except Exception as e:
            # 详细的错误日志
            error_msg = f"停车控制详细错误: {str(e)}"
            print(f"Error details: garage_corners={garage_corners}")
            print(f"Error details: car_pt_in_bev_image={car_pt_in_bev_image}")
            rospy.logerr(error_msg) 
            return stop_cmd, False


#================= 避障进程 =================
def run_obstacle_avoidance(shared_state):
    """运行避障节点"""
    from escape_obstacle import ObstacleAvoidance
    signal.signal(signal.SIGINT, signal_handler)
    rospy.loginfo("避障节点启动成功！等待激活...")
    oa = ObstacleAvoidance(shared_state)
    try:
        oa.run()
    except rospy.ROSInterruptException:
        pass

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    oa.stop()
    sys.exit(0)
        
def _check_lane_valid(leftx, lefty, rightx, righty, mode):
    """检查是否丢线（	到达岔路口1号和3号）"""
    # 添加静态变量来记录连续次数
    if not hasattr(_check_lane_valid, "counter"):
        _check_lane_valid.counter = 0
    
    # 检查输入是否为空
    if len(leftx) == 0:
        leftx = [0]
    if len(rightx) == 0:
        rightx = [0]
    if len(lefty) == 0:
        lefty = [359]
    if len(righty) == 0:
        righty = [359]
    # print("=============================")
    # print(f"左线数量: {len(leftx)}, 右线数量: {len(rightx)}")
    # print(f"左x跨度: {np.max(leftx) - np.min(leftx) if len(leftx) > 0 else 0}, 右x跨度: {np.max(rightx) - np.min(rightx) if len(rightx) > 0 else 0}")
    # print(f"左y跨度: {np.max(lefty) - np.min(lefty) if len(lefty) > 0 else 0}, 右y跨度: {np.max(righty) - np.min(righty) if len(righty) > 0 else 0}")
#    my_print(f"左车道线x坐标最大值: {np.max(leftx) if len(leftx) > 0 else 0}")
#    my_print(f"右车道线x坐标最小值: {np.min(rightx) if len(rightx) > 0 else 0}")
    
    # print("=============================")
#    rospy.loginfo_throttle(1.0, 
#        "车道线有效性检查：目前模式：%s；左线数量：%d；右线数量：%d", 
#        mode_str, len(leftx), len(rightx))  

    # 计算跨度
    if mode == 1:
        y_span = max(righty) - min(righty)  # 右线y坐标跨度
        x_span = max(rightx) - min(rightx)  # 右线x坐标跨度
        x_min = min(rightx)                 # 右线x坐标最小值
        y_min = min(righty)                 # 右线y坐标最小值
    else:
        y_span = max(lefty) - min(lefty)   # 左线y坐标跨度
        x_span = max(leftx) - min(leftx)   # 左线x坐标跨度
        x_max = max(leftx)                 # 左线x坐标最大值
        y_min = min(lefty)                 # 左线y坐标最小值
    
    # 检查跨度条件
    if mode == 1:
        if y_span < 50 and x_span < 50 and x_min > 410 and y_min > 238:
            _check_lane_valid.counter += 1
        else:
            _check_lane_valid.counter = 0
    else:
        if y_span < 50 and x_span < 50 and x_max < 220 and y_min > 238:
            _check_lane_valid.counter += 1
        else:
            _check_lane_valid.counter = 0
    
    # 如果连续三次跨度小于65，返回1
    if _check_lane_valid.counter >= 2:
        _check_lane_valid.counter = 0  # 重置计数器
        return 0
    return 1

def _round_insection(leftx, lefty, rightx, righty, curvature, mode):
    """
    判断是否到达环岛的第二岔路口
    通过检查左右车道线的数量和跨度条件来判断
    
    Args:
        leftx: 左车道线x坐标列表
        lefty: 左车道线y坐标列表
        rightx: 右车道线x坐标列表
        righty: 右车道线y坐标列表
        curvature: 弯曲度
        mode: 模式，0表示检查右车道线，1表示检查左车道线
    
    Returns:
        int: 1表示到达岔路口，0表示未到达
    """
    # 检查输入是否为空
    if len(leftx) == 0:
        leftx = np.array([0])
    if len(rightx) == 0:
        rightx = np.array([0])
    if len(lefty) == 0:
        lefty = np.array([0])
    if len(righty) == 0:
        righty = np.array([0])
    
    # 确保所有输入都是numpy数组
    leftx = np.array(leftx)
    lefty = np.array(lefty)
    rightx = np.array(rightx)
    righty = np.array(righty)
    
    # 打印车道线信息
#    print(f"左车道线数量: {len(leftx)}")
#    print(f"左车道线x坐标跨度: {np.max(leftx) - np.min(leftx) if len(leftx) > 0 else 0}")
#    print(f"左车道线y坐标跨度: {np.max(lefty) - np.min(lefty) if len(lefty) > 0 else 0}")
#    print(f"右车道线数量: {len(rightx)}")
#    print(f"右车道线x坐标跨度: {np.max(rightx) - np.min(rightx) if len(rightx) > 0 else 0}")
#    print(f"右车道线y坐标跨度: {np.max(righty) - np.min(righty) if len(righty) > 0 else 0}")
    
    # 计算跨度
    left_x_span = np.max(leftx) - np.min(leftx) if len(leftx) > 0 else 0
    left_y_span = np.max(lefty) - np.min(lefty) if len(lefty) > 0 else 0
    right_x_span = np.max(rightx) - np.min(rightx) if len(rightx) > 0 else 0
    right_y_span = np.max(righty) - np.min(righty) if len(righty) > 0 else 0
    
    # 根据mode进行条件判断
    if mode == 1:
        # 检查左车道线
        if (len(leftx) > 170 and len(rightx) > 170 and  # 数量条件
            left_y_span > 150 and right_y_span > 150 and  # y跨度条件
            left_x_span < 50 and right_x_span < 50):  # x跨度条件
            return 1
    else:  # mode == 0
        # 检查右车道线
        if (len(leftx) > 170 and len(rightx) > 170 and  # 数量条件
            left_y_span > 150 and right_y_span > 150 and  # y跨度条件
            left_x_span < 50 and right_x_span < 50):  # x跨度条件
            return 1

    return 0




def has_left_and_right_corners(leftx, lefty, rightx, righty, garage_corners):
    """
    检查是否有左右两个车库角点，并确保左右车道线都有足够的像素点，
    优先选择下方的角点对（y坐标值更大），且左右角点的纵坐标差值不超过60
    
    Args:
        leftx: 左车道线x坐标数组
        lefty: 左车道线y坐标数组  
        rightx: 右车道线x坐标数组
        righty: 右车道线y坐标数组
        garage_corners: 车库角点列表，格式为 [(x, y, side), ...]
        
    Returns:
        bool: 满足条件则返回True，否则返回ang.z=0.00False
    """
                
    # 分别获取左右角点
    left_corners = [(x, y) for x, y, side in garage_corners if side == 'L']
    right_corners = [(x, y) for x, y, side in garage_corners if side == 'R']
    
    # 检查是否有左右两个角点
    has_left_corner = len(left_corners) > 0
    has_right_corner = len(right_corners) > 0
    
    if not (has_left_corner and has_right_corner):
        return False
    
    # 按y坐标降序排序，优先选择下方的角点（y坐标值更大）
    left_corners_sorted = sorted(left_corners, key=lambda corner: corner[1], reverse=True)
    right_corners_sorted = sorted(right_corners, key=lambda corner: corner[1], reverse=True)
    
    # print(f"[DEBUG] 左角点排序后: {left_corners_sorted}")
    # print(f"[DEBUG] 右角点排序后: {right_corners_sorted}")
    
    # 寻找下方的有效角点对
    valid_pair_found = False
    selected_left_corner = None
    selected_right_corner = None
    
    # 优先匹配下方的角点对
    for left_corner in left_corners_sorted:
        for right_corner in right_corners_sorted:
            left_x, left_y = left_corner
            right_x, right_y = right_corner
            
            # 检查y坐标差值是否在允许范围内
            y_diff = abs(left_y - right_y)
            if y_diff <= 40:
                # 找到有效的角点对，记录并退出
                selected_left_corner = left_corner
                selected_right_corner = right_corner
                valid_pair_found = True
                print(f"[DEBUG] 找到有效角点对: 左({left_x}, {left_y}) 右({right_x}, {right_y}), y差值: {y_diff}")
                break
        
        if valid_pair_found:
            break
    
    if not valid_pair_found:
        # print("[DEBUG] 未找到y坐标差值满足条件的角点对")
        return False
    
    # 检查左右车道线像素点数量是否足够（至少50个）
    left_points_enough = len(leftx) > 50 and len(lefty) > 50
    right_points_enough = len(rightx) > 50 and len(righty) > 50
    
    # print(f"[DEBUG] 左车道线点数: {len(leftx)}, 右车道线点数: {len(rightx)}")
    # print(f"[DEBUG] 左车道线足够: {left_points_enough}, 右车道线足够: {right_points_enough}")
    
    # 所有条件都满足才返回True
    result = left_points_enough and right_points_enough
    # print(f"[DEBUG] 最终结果: {result}")
    
    return result

# ====================== 主程序 ========================
if __name__ == '__main__':

    multiprocessing.set_start_method('spawn')
    
    import multiprocessing as mp
    import numpy as np
    
    # 使用Manager创建共享状态  
    manager = multiprocessing.Manager()  
    shared_state = manager.Namespace()  
    shared_state.flag_task = manager.Value('i', 0)  
    shared_state.lane_mode = manager.Value('i', 0)  
    shared_state.lock = manager.Lock()  
    shared_state.enable_event = manager.Event() 

    # ========== 锁分配说明 ==========
    # - shared_state.lock : 用于保护flag_task和lane_mode
    # - cmd_lock : 仅用于底盘控制指令同步

    # ========== 创建cmd指令锁 ==========
    cmd_lock = multiprocessing.Lock()

    # 创建进程列表
    processes = []
    # 创建需要监控的关键进程列表（不包括避障进程）
    critical_processes = []
    
    try:
        # ========== 启动子进程 ==========
        # 1. Control节点进程 (服务托管)
        control_process = Process(
            name='control_node_process',
            target=run_control_node,
            args=(shared_state,)
        )
        processes.append(control_process)
        critical_processes.append(control_process)  # 添加到关键进程监控列表

        # 2. 车道检测进程 (需共享状态)
        lane_process = Process(
            name='lane_detection_process',
            target=run_lane_detection,
            args=(shared_state,)
        )
        processes.append(lane_process)
        critical_processes.append(lane_process)  # 添加到关键进程监控列表

        # 3. 底盘控制进程 (需共享状态+指令锁)
        chassis_process = Process(
            name='chassis_control_process',
            target=run_chassis_control,
            args=(shared_state,cmd_lock)
        )
        processes.append(chassis_process)
        critical_processes.append(chassis_process)  # 添加到关键进程监控列表

        # 4. 避障进程 (需共享状态) - 不监控此进程
        avoidance_process = Process(
            name='obstacle_avoidance_process',
            target=run_obstacle_avoidance,
            args=(shared_state,)
        )
        processes.append(avoidance_process)
        # 注意：避障进程不添加到critical_processes列表中

        # 启动所有进程
        for p in processes:
            p.start()
            my_print(f"进程启动: {p.name}", "info")
            time.sleep(0.5)  # 间隔启动避免资源冲突
            if not p.is_alive():
                my_print(f"关键进程异常终止: {p.name}", "warn")
                raise RuntimeError(f"Critical process {p.name} died")            
            time.sleep(1)


        # ========== 主进程监控 ==========
        while True:
            # 检查子进程健康状态
            for p in critical_processes:
                if not p.is_alive():
                    my_print(f"进程异常终止: {p.name}", "warn")
                    raise RuntimeError(f"{p.name} died")

            time.sleep(1)


    except KeyboardInterrupt:
        my_print("收到终止信号，执行安全关闭...", "warn")
    except Exception as e:
        my_print("主进程异常", "error", str(e))
        
    finally:
        # ========== 资源清理 ==========
        # 1. 停止所有子进程
        for p in processes:
            if p.is_alive():
                my_print(f"正在终止进程: {p.name}", "info")
                p.terminate()
                p.join(timeout=1.0)
                if p.is_alive():
                    my_print(f"强制终止进程: {p.name}", "warn")
                    p.kill()
                else:
                    my_print(f"进程已安全终止: {p.name}", "info")
        
        manager.shutdown()  # 关闭manager内存
        my_print("系统安全关闭完成", "info")
