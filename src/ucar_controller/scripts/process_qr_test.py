#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import signal
import sys
import tf2_ros
import subprocess

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from std_srvs.srv import Empty


# ---------------------------- 全局共享变量 ----------------------------
class SharedVariables:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_buffer = None
        self.tf_listener = None

        # 导航点定义（前3个为 x,y,z，后4个为四元数 x,y,z,w，表示朝向）
        self.nav_point = {
                "s0": [0.09, -0.04, 0.0, 0.0, 0.0, 0.0, 1.0],
                "s0_twist": [0.124, -0.0166, 0.0, 0.0, 0.0, -0.7071, 0.7071],
                "s1": [0.16, -0.50, 0.0, 0.0, 0.0, -0.7071, 0.7071],
                "s2":  [0.12, -0.92, 0.0, 0.0, 0.0, -0.7071, 0.7071],
                "s2_twist":  [0.12, -0.92, 0.0, 0.0, 0.0,1.0 ,0.0 ],
        }

SV = SharedVariables()

# ---------------------------- 初始化 ----------------------------
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
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = 0.0
    initial_pose.pose.pose.orientation.w = 1.0
    rospy.sleep(1)
    pub.publish(initial_pose)
    rospy.loginfo("已发送初始位姿")

def init_tf_listener():
    SV.tf_buffer = tf2_ros.Buffer()
    SV.tf_listener = tf2_ros.TransformListener(SV.tf_buffer)
    rospy.sleep(1.0)
    rospy.loginfo("TF 监听器已初始化")


# ---------------------------- 工具函数 ----------------------------
def reset_navigation():
    """取消当前导航目标并清除代价地图"""
    # 取消当前导航任务
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
    cancel_msg = GoalID()
    cancel_pub.publish(cancel_msg)
    rospy.loginfo("Current navigation goal canceled.")
    
    # 重置代价地图
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps()
        rospy.loginfo("Costmaps cleared.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def kill_ros_node(node_name):
    """强制杀死指定的ROS节点"""
    try:
        command = f"rosnode kill {node_name}"
        subprocess.run(command, shell=True, check=True)
        rospy.loginfo(f"Successfully killed ROS node: {node_name}")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to kill ROS node {node_name}: {e}")

def send_nav_point_and_wait(target_pose_list, timeout=None):
    if len(target_pose_list) != 7:
        rospy.logerr(f"导航点列表长度错误: {len(target_pose_list)}")
        return

    name = "unknown"
    for k, v in SV.nav_point.items():
        if v == target_pose_list:
            name = k
            break

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(*target_pose_list[:3]),
                                 Quaternion(*target_pose_list[3:]))

    rospy.loginfo(f"前往导航点: {name}")
    SV.move_base.send_goal(goal)
    SV.move_base.wait_for_result()
    result = SV.move_base.get_result()

    if result:
        rospy.loginfo(f"成功到达 {name}")
    else:
        rospy.logwarn(f"首次尝试到达 {name} 失败，正在重置代价地图并重试...")
        reset_navigation()
        rospy.sleep(1)
        SV.move_base.send_goal(goal)
        SV.move_base.wait_for_result()
        result = SV.move_base.get_result()
        if result:
            rospy.loginfo(f"重试后成功到达 {name}")
        else:
            rospy.logerr(f"重试后仍无法到达 {name}")

def cancel_all_goals():
    SV.move_base.cancel_all_goals()
    rospy.loginfo("已取消所有导航目标")

def signal_handler(sig, frame):
    rospy.loginfo("收到中断信号，正在退出...")
    reset_navigation()          # 彻底清理
    rospy.signal_shutdown("用户终止")
    sys.exit(0)


# ---------------------------- 定点巡航任务 ----------------------------
def cruise_points(points):
    rospy.loginfo(f"开始定点巡航，共 {len(points)} 个点")
    for name in points:
        if rospy.is_shutdown():
            break
        if name not in SV.nav_point:
            rospy.logerr(f"导航点 '{name}' 不存在，跳过")
            continue
        send_nav_point_and_wait(SV.nav_point[name])
        rospy.sleep(0.5)
    rospy.loginfo("定点巡航完成")


# ---------------------------- 主程序 ----------------------------
if __name__ == '__main__':
    rospy.init_node('cruise_mode')
    signal.signal(signal.SIGINT, signal_handler)

    rospy.loginfo("初始化 move_base ...")
    init_move_base()

    rospy.loginfo("初始化 TF 监听器 ...")
    init_tf_listener()

    rospy.sleep(2)
    rospy.loginfo("开始巡航")

    patrol_path = [
        "s0","s0_twist",
        "s1",
        "s2","s2_twist",
    ]

    cruise_points(patrol_path)
    rospy.loginfo("任务结束，节点保持运行...")
    rospy.spin()