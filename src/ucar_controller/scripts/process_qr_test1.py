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
          # 必须导入

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
        }

        # 不再需要 valid_qr_keys，已删除

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


# ---------------------------- 分段旋转扫描三个二维码 ----------------------------
def scan_three_qr_codes(angular_speed=0.5, rotate_duration=1.57, stop_duration=2.0, max_attempts=4, timeout=60.0):
    """
    每次旋转约90度，停车扫描二维码，解析URL获取子类名称，收集三个不同子类后退出。
    发布结果到 /qr_scan_results。
    返回子类名称列表。
    """
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    result_pub = rospy.Publisher('/qr_scan_results', String, queue_size=1)
    twist = Twist()
    bridge = CvBridge()
    collected_items = []          # 存储已收集的子类名称
    collected_urls = []           # 用于去重（也可用子类名去重）
    image_lock = threading.Lock()
    latest_image = None

    def img_cb(msg):
        nonlocal latest_image
        try:
            with image_lock:
                latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    sub_img = rospy.Subscriber("/usb_cam/image_raw", Image, img_cb, queue_size=1)
    rospy.sleep(0.5)  # 等待图像流稳定

    rospy.loginfo("开始分段旋转扫描三个二维码...")
    start_time = rospy.Time.now()

    for attempt in range(max_attempts):
        if rospy.is_shutdown():
            break

        # 1. 旋转90度（约1.57秒 @ 0.5 rad/s）
        rospy.loginfo(f"旋转第 {attempt+1} 次...")
        twist.angular.z = angular_speed
        pub_vel.publish(twist)
        rospy.sleep(rotate_duration)
        # 停止旋转
        pub_vel.publish(Twist())
        rospy.loginfo("停止，等待扫描...")
        rospy.sleep(stop_duration)  # 停车2秒，让摄像头稳定

        # 2. 抓取当前图像，尝试解码二维码
        with image_lock:
            img = latest_image
        if img is not None:
            decoded_list = pyzbar.decode(img)
            for obj in decoded_list:
                qr_data = obj.data.decode("utf-8")
                # 检查是否为URL（简单判断）
                if not (qr_data.startswith("http://") or qr_data.startswith("https://")):
                    rospy.logwarn(f"非URL内容，忽略: {qr_data}")
                    continue
                # 避免重复采集同一个二维码
                if qr_data in collected_urls:
                    continue

                # 3. 请求URL获取JSON
                try:
                    rospy.loginfo(f"请求URL: {qr_data}")
                    resp = requests.get(qr_data, timeout=5)
                    if resp.status_code == 200:
                        json_data = resp.json()
                        if json_data.get("code") == 200 and "result" in json_data:
                            item_name = json_data["result"]
                            # 子类名去重（避免不同URL但相同子类情况）
                            if item_name not in collected_items:
                                collected_items.append(item_name)
                                collected_urls.append(qr_data)
                                rospy.loginfo(f"采集到子类: {item_name}")
                                # 实时发布已收集的结果
                                result_msg = String()
                                result_msg.data = json.dumps(collected_items, ensure_ascii=False)
                                result_pub.publish(result_msg)
                            else:
                                rospy.logwarn(f"重复子类 {item_name}，忽略")
                        else:
                            rospy.logwarn(f"JSON格式错误或无result: {json_data}")
                    else:
                        rospy.logwarn(f"HTTP请求失败，状态码: {resp.status_code}")
                except Exception as e:
                    rospy.logerr(f"URL处理异常: {e}")

        # 4. 如果已集齐三个，退出循环
        if len(collected_items) >= 3:
            rospy.loginfo("已收集三个不同子类，停止扫描")
            break

        # 超时检查
        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logwarn("扫描超时")
            break

    # 停车并取消订阅
    pub_vel.publish(Twist())
    sub_img.unregister()
    rospy.loginfo(f"扫描结束，共收集子类: {collected_items}")
    return collected_items


# ---------------------------- 主任务 ----------------------------
def main_mission():
    # 第一步：导航到巡航终点
    last = SV.nav_point["last_point"]
    rospy.loginfo("开始导航至第一部分巡航终点...")
    send_nav_point_and_wait(last)
    rospy.sleep(0.5)

    # 导航到 room_st
    room_st = SV.nav_point["room_st"]
    send_nav_point_and_wait(room_st)
    rospy.sleep(0.5)

    # 第二步：前往物品领取区（这里用 room_center 作为示例，需替换为你实际坐标）
    pickup_point = SV.nav_point["room_center"]
    rospy.loginfo("前往物品领取区，准备扫描二维码...")
    send_nav_point_and_wait(pickup_point)
    rospy.sleep(0.5)

    # 第三步：分段旋转扫描三个二维码，获取子类名称列表
    item_list = scan_three_qr_codes(angular_speed=0.5, rotate_duration=1.57, stop_duration=2.0)
    if len(item_list) == 3:
        rospy.loginfo(f"成功获取三个子类: {item_list}")
        # 后续可在此调用大模型推理、语音播报等
    else:
        rospy.logerr(f"未集齐三个子类，仅获取到: {item_list}")


if __name__ == '__main__':
    rospy.init_node('qr_mission_novoice')
    signal.signal(signal.SIGINT, signal_handler)

    rospy.loginfo("初始化 move_base ...")
    init_move_base()

    rospy.loginfo("开始执行二维码扫描任务")
    main_mission()

    rospy.loginfo("任务完成，节点保持运行...")
    rospy.spin()