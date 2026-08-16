#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import signal
import sys
import threading

import actionlib
import rospy
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


# ---------------------------- 全局共享变量 ----------------------------
class SharedVariables:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # 导航点定义（导航逻辑保持不变）
        self.nav_point = {
            "last_point": [0.12, -0.92, 0.0, 0.0, 0.0, 1.0, 0.0],
            "room_edge": [-0.636, -0.973, 0.0, 0.0, 0.0, 1.0, 0.0],
            "room_center": [-0.80, -0.51, 0.0, 0.0, 0.0, 0.7071, 0.7071],
            "room_st": [-0.80, -0.92, 0.0, 0.0, 0.0, 1.0, 0.0],
        }

        self.qr_lock = threading.Lock()
        self.unique_urls = []
        self.parsed_results = {}
        self.new_url_event = threading.Event()
        self.new_parsed_event = threading.Event()
        self.scan_active = False
        self.current_yaw = 0.0
        self.odom_received = False


SV = SharedVariables()


# ---------------------------- 基础初始化 ----------------------------
def init_move_base():
    if not SV.move_base.wait_for_server(rospy.Duration(5)):
        rospy.logerr("无法连接 move_base action server...")
        sys.exit(1)
    rospy.loginfo("move_base action server 已连接")
    send_initialpose()


def send_initialpose():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,  queue_size=10) 

    center = SV.nav_point["room_center"]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()

    initial_pose.pose.pose.position.x = center[0]
    initial_pose.pose.pose.position.y = center[1]
    initial_pose.pose.pose.position.z = center[2]

    initial_pose.pose.pose.orientation.x = center[3]
    initial_pose.pose.pose.orientation.y = center[4]
    initial_pose.pose.pose.orientation.z = center[5]
    initial_pose.pose.pose.orientation.w = center[6]
    rospy.sleep(1)
    pub.publish(initial_pose)
    rospy.loginfo("已发布初始位姿 room_center")


def send_nav_point_and_wait(target_pose_list):
    if len(target_pose_list) != 7:
        rospy.logerr("导航点列表长度错误")
        return

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(
        Point(*target_pose_list[:3]),
        Quaternion(*target_pose_list[3:])
    )
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


# ---------------------------- 话题回调 ----------------------------
def is_url(data):
    return data.startswith("http://") or data.startswith("https://")


def qr_code_callback(msg):
    """订阅 /qr_code_result，只记录原始 URL 并查重。"""
    rospy.loginfo("收到原始二维码内容: [%s]", msg.data)
    qr_data = msg.data.strip()
    if not is_url(qr_data):
        rospy.logwarn("收到非 URL 二维码内容，忽略: %s", qr_data)
        return

    with SV.qr_lock:
        if not SV.scan_active:
            return
        if qr_data in SV.unique_urls:
            return

        SV.unique_urls.append(qr_data)
        SV.new_url_event.set()
        count = len(SV.unique_urls)

    rospy.loginfo("获取到新的二维码 URL [%d/3]: %s", count, qr_data)


def qr_url_parsed_callback(msg):
    """订阅 /qr_url_parsed，只保存 qr_url_parser.py 解析后的信息。"""
    try:
        result = json.loads(msg.data)
    except ValueError as e:
        rospy.logerr("解析 /qr_url_parsed JSON 失败: %s", e)
        return

    url = result.get("url") or result.get("data")
    if not url:
        return

    with SV.qr_lock:
        if SV.scan_active or url in SV.unique_urls:
            SV.parsed_results[url] = result
            SV.new_parsed_event.set()


# ---------------------------- 信息解读与结果发布 ----------------------------
def odom_callback(msg):
    q = msg.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    _, _, yaw = euler_from_quaternion(quat)

    with SV.qr_lock:
        SV.current_yaw = yaw
        SV.odom_received = True


def interpret_parsed_result(parsed_result):
    """把 /qr_url_parsed 的结果转换成任务需要的简洁信息。"""
    if not parsed_result:
        return None
    if parsed_result.get("status") != "success":
        return None

    json_data = parsed_result.get("json_data")
    if isinstance(json_data, dict):
        if "result" in json_data:
            return json_data["result"]
        if "item_type" in json_data:
            return json_data["item_type"]
        if "task_type" in json_data:
            return json_data["task_type"]

    text_data = parsed_result.get("text_data")
    if text_data:
        return text_data

    return None


def publish_scan_results(result_pub, urls, items):
    payload = {
        "urls": urls,
        "items": items,
    }
    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=False)
    result_pub.publish(msg)


# ---------------------------- 旋转扫描逻辑 ----------------------------
def reset_scan_state():
    with SV.qr_lock:
        SV.unique_urls = []
        SV.parsed_results = {}
        SV.new_url_event.clear()
        SV.new_parsed_event.clear()
        SV.scan_active = True


def wait_for_parsed_results(urls, timeout=10.0):
    start_time = rospy.Time.now()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        with SV.qr_lock:
            if all(url in SV.parsed_results for url in urls):
                break

        if (rospy.Time.now() - start_time).to_sec() > timeout:
            break

        SV.new_parsed_event.wait(0.1)
        SV.new_parsed_event.clear()
        rate.sleep()

    items = []
    with SV.qr_lock:
        for url in urls:
            item = interpret_parsed_result(SV.parsed_results.get(url))
            if item is not None and item not in items:
                items.append(item)

    return items


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def get_current_yaw(timeout=5.0):
    start_time = rospy.Time.now()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        with SV.qr_lock:
            if SV.odom_received:
                return SV.current_yaw

        if (rospy.Time.now() - start_time).to_sec() > timeout:
            rospy.logerr("Timed out waiting for /odom; cannot rotate precisely by 90 degrees.")
            return None

        rate.sleep()


def has_scanned_three_urls():
    with SV.qr_lock:
        if len(SV.unique_urls) >= 3:
            return list(SV.unique_urls[:3])
    return None


def rotate_90_degrees(pub_vel, angular_speed=0.5, tolerance=0.03, max_turn_time=15.0):
    start_yaw = get_current_yaw()
    if start_yaw is None:
        return False

    target_yaw = normalize_angle(start_yaw + math.pi / 2.0)
    turn_start_time = rospy.Time.now()
    max_speed = abs(angular_speed)
    min_speed = min(0.12, max_speed)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        with SV.qr_lock:
            current_yaw = SV.current_yaw

        error = normalize_angle(target_yaw - current_yaw)
        if abs(error) <= tolerance:
            break

        if (rospy.Time.now() - turn_start_time).to_sec() > max_turn_time:
            rospy.logwarn("90-degree rotation timed out before reaching target yaw.")
            pub_vel.publish(Twist())
            return False

        command_speed = min(max_speed, max(min_speed, abs(error) * 1.5))
        twist = Twist()
        twist.angular.z = command_speed if error > 0 else -command_speed
        pub_vel.publish(twist)
        rate.sleep()

    pub_vel.publish(Twist())
    return not rospy.is_shutdown()


def rotate_scan_until_three_urls(angular_speed=0.5, timeout=60.0):
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    reset_scan_state()
    rospy.loginfo("Start precise 90-degree rotation scan; stop 10 seconds after each turn.")
    urls = []

    try:
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            scanned_urls = has_scanned_three_urls()
            if scanned_urls:
                urls = scanned_urls
                break

            if (rospy.Time.now() - start_time).to_sec() > timeout:
                with SV.qr_lock:
                    urls = list(SV.unique_urls)
                rospy.logwarn("Scan timed out; only got %d unique URLs.", len(urls))
                break

            if not rotate_90_degrees(pub_vel, angular_speed=angular_speed):
                with SV.qr_lock:
                    urls = list(SV.unique_urls)
                break

            scanned_urls = has_scanned_three_urls()
            if scanned_urls:
                urls = scanned_urls
                break

            rospy.loginfo("Finished one precise 90-degree turn, stopping for 10 seconds.")
            rospy.sleep(10.0)
    finally:
        pub_vel.publish(Twist())
        with SV.qr_lock:
            SV.scan_active = False

    return urls


def scan_three_qr_codes(angular_speed=0.5, timeout=60.0):
    """
    旋转扫描二维码。
    /qr_code_result 用于获取原始 URL 并查重；
    /qr_url_parsed 用于获取 URL 解析后的信息；
    获取到 3 个不同 URL 后停止旋转，并发布汇总结果到 /qr_scan_results。
    """
    result_pub = rospy.Publisher('/qr_scan_results', String, queue_size=1)

    urls = rotate_scan_until_three_urls(angular_speed=angular_speed, timeout=timeout)
    items = wait_for_parsed_results(urls, timeout=10.0)
    publish_scan_results(result_pub, urls, items)

    rospy.loginfo("扫描结束，URL: %s", urls)
    rospy.loginfo("解析结果: %s", items)
    return items


# ---------------------------- 主任务 ----------------------------
def main_mission():
    # 第一步：导航到巡航终点
    # last = SV.nav_point["last_point"]
    # rospy.loginfo("开始导航至第一部分巡航终点...")
    # send_nav_point_and_wait(last)
    # rospy.sleep(0.5)

    # # 导航到 room_st
    # room_st = SV.nav_point["room_st"]
    # send_nav_point_and_wait(room_st)
    # rospy.sleep(0.5)

    # 第二步：前往物品领取区
    # pickup_point = SV.nav_point["room_center"]
    rospy.loginfo("前往物品领取区，准备扫描二维码...")
    # send_nav_point_and_wait(pickup_point)
    # rospy.sleep(0.5)

    # 第三步：旋转扫描三个二维码，解析结果由 /qr_url_parsed 提供
    item_list = scan_three_qr_codes(angular_speed=0.5)
    if len(item_list) == 3:
        rospy.loginfo("成功获取三个解析结果: %s", item_list)
    else:
        rospy.logerr("未获取完整三个解析结果，仅获取到: %s", item_list)


if __name__ == '__main__':
    rospy.init_node('qr_mission_novoice')
    signal.signal(signal.SIGINT, signal_handler)

    rospy.Subscriber('/qr_code_result', String, qr_code_callback, queue_size=10)
    rospy.Subscriber('/qr_url_parsed', String, qr_url_parsed_callback, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=10)

    rospy.loginfo("初始化 move_base ...")
    init_move_base()

    rospy.loginfo("开始执行二维码扫描任务")
    main_mission()

    rospy.loginfo("任务完成，节点保持运行...")
    rospy.spin()


