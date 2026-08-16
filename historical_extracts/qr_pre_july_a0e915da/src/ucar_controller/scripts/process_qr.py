#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import signal
import sys
import cv2
import threading
import json
import requests
from cv_bridge import CvBridge
# from pyzbar import pyzbar          # 原代码缺失此导入，补上

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ---------------------------- 全局共享变量 ----------------------------
class SharedVariables:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # 导航点定义（根据你的实际地图修改！）
        self.nav_point = {
            "last_point": [0.12, -0.92, 0.0, 0.0, 0.0, 1.0, 0.0],
            "room_edge": [-0.636, -0.973, 0.0, 0.0, 0.0, 1.0, 0.0],
            "room_center": [-0.80, -0.51, 0.0, 0.0, 0.0, 0.7071, 0.7071],
            "room_st": [-0.80, -0.92, 0.0, 0.0, 0.0, 1.0, 0.0]
              # "room_ce
            # "room_1": [-0.3, -0.92 0.0, 0.0, 0.0, 1.0, 0.0]
        }

        # 有效二维码内容（任务类别）
        self.valid_qr_keys = {
            'Dessert', 'Fruit', 'Vegetable'
        }

        # URL 解析结果存储
        self.latest_url_result = None

SV = SharedVariables()


# ---------------------------- 基础初始化 ----------------------------
def init_move_base():
    if not SV.move_base.wait_for_server(rospy.Duration(5)):
        rospy.logerr("无法连接 move_base action server...")
        sys.exit(1)
    rospy.loginfo("move_base action server 已连接")
    send_initialpose()

def send_initialpose():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0
    rospy.sleep(1)
    pub.publish(initial_pose)
    rospy.loginfo("已发布初始位姿 (0,0)")

def send_nav_point_and_wait(target_pose_list):
    if len(target_pose_list) != 7:
        rospy.logerr("导航点列表长度错误")
        return

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(*target_pose_list[:3]),
                                 Quaternion(*target_pose_list[3:]))
    SV.move_base.send_goal(goal)
    SV.move_base.wait_for_result()
    if SV.move_base.get_result():
        rospy.loginfo("导航成功")
    else:
        rospy.logwarn("导航失败")

def cancel_all_goals():
    SV.move_base.cancel_all_goals()

def signal_handler(sig, frame):
    cancel_all_goals()
    rospy.signal_shutdown("用户终止")
    sys.exit(0)


# ---------------------------- URL 解析结果处理 ----------------------------
def url_parsed_callback(msg):
    """
    处理从 qr_url_parser.py 发布的 URL 解析结果
    """
    try:
        result = json.loads(msg.data)
        SV.latest_url_result = result
        
        if result['status'] == 'success':
          
            rospy.loginfo(f"成功解析 URL: {result['url']}")
            if 'json_data' in result:
                rospy.loginfo(f"获取到 JSON 数据: {result['json_data']}")
                process_url_data(result['json_data'])
            elif 'text_data' in result:
                rospy.loginfo(f"获取到文本数据: {result['text_data'][:100]}...")
                pass
        elif result['status'] == 'error':
            # 静默处理解析错误，不输出
            # rospy.logwarn(f"URL 解析失败: {result['error']}")
            pass
        elif result['status'] == 'not_url':
            # 不输出非URL信息
            # rospy.loginfo(f"二维码内容不是 URL: {result['data']}")
            pass
            
    except json.JSONDecodeError as e:
        rospy.logerr(f"解析 URL 结果 JSON 失败: {e}")
    except Exception as e:
        rospy.logerr(f"处理 URL 结果时出错: {e}")


def process_url_data(json_data):
    """
    根据解析到的 JSON 数据执行相应任务（不输出数据细节）
    """
    try:
        if 'task_type' in json_data:
            task_type = json_data['task_type']
            if task_type == 'pickup':
                target_location = json_data.get('location', 'room_center')
                if target_location in SV.nav_point:
                    # rospy.loginfo(f"执行取货任务，导航到: {target_location}")
                    send_nav_point_and_wait(SV.nav_point[target_location])
                # else:
                #     rospy.logwarn(f"未知位置: {target_location}")
            elif task_type == 'deliver':
                # 可在此添加送货逻辑
                pass
            # 其他任务类型不输出日志
        
        if 'item_type' in json_data:
            item_type = json_data['item_type']
            if item_type in SV.valid_qr_keys:
                # rospy.loginfo(f"识别到有效物品类型: {item_type}")
                pass
            # else:
            #     rospy.logwarn(f"未知物品类型: {item_type}")
                
    except Exception as e:
        rospy.logerr(f"处理 URL 数据时出错: {e}")


# ---------------------------- 二维码旋转扫描 ----------------------------
def rotate_and_scan(angular_speed=0.5, timeout=60.0):
    """
    原地旋转并扫描二维码，返回第一个有效数据，或 None。
    不打印任何扫描到的二维码内容。
    """
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist = Twist()
    twist.angular.z = angular_speed

    bridge = CvBridge()
    latest_image = None
    image_lock = threading.Lock()

    def img_cb(msg):
        nonlocal latest_image
        try:
            with image_lock:
                latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    sub_img = rospy.Subscriber("/usb_cam/image_raw", Image, img_cb, queue_size=1)
    rospy.sleep(0.5)  # 等待图像稳定

    rospy.loginfo("开始旋转扫描二维码...")
    start = rospy.Time.now()
    rate = rospy.Rate(10)
    result = None

    while not rospy.is_shutdown():
        pub_vel.publish(twist)

        with image_lock:
            img = latest_image
        if img is not None:
            decoded = pyzbar.decode(img)
            for obj in decoded:
                data = obj.data.decode("utf-8")
                # 不输出扫描到的任何数据
                # rospy.loginfo(f"扫描到二维码内容: {data}")
                if data in SV.valid_qr_keys:
                    # rospy.loginfo(f"识别到有效任务: {data}")
                    result = data
                    break
        if result:
            break

        if (rospy.Time.now() - start).to_sec() > timeout:
            rospy.logwarn("扫描超时，未检测到有效二维码")
            break
        rate.sleep()

    pub_vel.publish(Twist())   # 停车
    sub_img.unregister()
    return result


# ---------------------------- 主任务 ----------------------------
def main_mission():
    # 第一步：直接导航到第一部分的最后一个点
    last = SV.nav_point["last_point"]
    rospy.loginfo("开始导航至第一部分巡航终点...")
    send_nav_point_and_wait(last)
    rospy.sleep(0.5)
    room_st=SV.nav_point["room_st"]
    send_nav_point_and_wait(room_st)
    rospy.sleep(0.5)

     # 更新房间起点为第一部分终点

    # 第二步：前往房间中心
    room = SV.nav_point["room_center"]
    rospy.loginfo("前往房间中心，准备扫描二维码...")
    send_nav_point_and_wait(room)
    rospy.sleep(0.5)

    # 第三步：旋转扫描二维码
    qr_data = rotate_and_scan(angular_speed=0.5, timeout=60.0)
    if qr_data:
        # 不输出任务目标确认信息
        # rospy.loginfo(f"任务目标已确认: {qr_data}")
        pass
    else:
        rospy.logerr("未能识别到有效二维码")


if __name__ == '__main__':
    rospy.init_node('qr_mission_novoice')
    signal.signal(signal.SIGINT, signal_handler)

    # 订阅 URL 解析结果
    url_sub = rospy.Subscriber('/qr_url_parsed', String, url_parsed_callback)

    rospy.loginfo("初始化 move_base ...")
    init_move_base()

    rospy.loginfo("开始执行二维码扫描任务（跳过第一部分巡航）")
    main_mission()

    rospy.loginfo("任务完成，节点保持运行...")
    rospy.spin()