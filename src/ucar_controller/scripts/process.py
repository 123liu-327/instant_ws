#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import signal
import sys
import tf2_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import String
# ---------------------------- 全局共享变量 ----------------------------
class SharedVariables:
    def __init__(self):
        # ===== Action 通信：创建 move_base 的 Action 客户端 =====
        # move_base 是导航的核心 Action 服务器，通过它发送目标点并等待到达
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # ===== TF 监听器：用于获取机器人实时坐标变换 =====
        self.tf_buffer = None      # 存储 TF 数据
        self.tf_listener = None    # 订阅 /tf 话题，填充 buffer

        # 导航点定义（前3个为 x,y,z，后4个为四元数 x,y,z,w，表示朝向）
        self.nav_point = {
            "s0":           [1.54, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]   ,        # 朝前
            "s0_twist_45":  [1.54, 0.0, 0.0, 0.0, 0.0, -0.3827, 0.9239],    # 右转45度
            "s0_twist":     [1.54, 0.0, 0.0, 0.0, 0.0, -0.706, 0.706],    # 右转90度

            "s1":           [1.68, -0.395, 0.0, 0.0, 0.0, -0.706, 0.706], # 朝右
            "s1_twist_45":  [1.68, -0.395, 0.0, 0.0, 0.0, -0.3827, 0.9239], # 左转45度（回正一半）
            "s1_twist":     [1.68, -0.395, 0.0, 0.0, 0.0, 0.0, 1.0],        # 朝前

            "s2":           [2.07, -0.45, 0.0, 0.0, 0.0, 0.0, 1.0],         # 朝前
            "s2_twist_45":  [2.07, -0.45, 0.0, 0.0, 0.0, 0.3827, 0.9239],   # 右转45度
            "s2_twist":     [2.07, -0.45, 0.0, 0.0, 0.0, 0.707, 0.707],     # 右转90度

            "s3":           [2.07, -0.04, 0.0, 0.0, 0.0, 0.707, 0.707],       # 朝右
            "s3_twist_45":  [2.07, -0.04, 0.0, 0.0, 0.0, 0.3827, 0.9239],     # 左转45度
            "s3_twist":     [2.07, -0.04, 0.0, 0.0, 0.0, 0.0, 1.0],           # 朝前

           # "s40":      [2.39,0.039,0.0,0.0, 0.0, 1.0],
            "s4":           [3.07, -0.02, 0.0, 0.0, 0.0, 0.0, 1.0],           # 朝前
            "s4_twist_45":  [3.07, -0.02, 0.0, 0.0, 0.0, -0.3827, 0.9239],    # 右转45度
            "s4_twist":     [3.07, -0.02, 0.0, 0.0, 0.0, -0.707, 0.707],      # 右转90度

            "s50":  [3.07, -0.55, 0.0, 0.0, 0.0, -0.707, 0.707],
           "s51":[3.1, -0.55, 0.0, 0.0, 0.0, -0.707, 0.707],
            "s5":           [3.08, -0.92, 0.0, 0.0, 0.0, -0.707, 0.707], 
           "s5_twist_45":  [3.08, -0.92, 0.0, 0.0, 0.0, -0.9239, 0.3827],  # 右转45度（朝右下）
            "s5_twist":     [3.08, -0.92, 0.0, 0.0, 0.0, 1.0, 0.0],

            "s60":[2.0, -0.96, 0.0, 0.0, 0.0, 1.0, 0.0],
            "s61":[1.6, -0.973, 0.0, 0.0, 0.0, 1.0, 0.0],
            "s6":           [1.14,-0.949,0.0,0.0,0.0,1.0,0.0],
           "s6_twist_45":  [1.14, -0.949, 0.0, 0.0, 0.0, 0.9239,0.3827],   # 右转45度（朝右下）
            "s6_twist":  [1.14, -0.949, 0.0, 0.0, 0.0, 0.707, 0.707],      # 再右转45度（朝右）

            "s7":           [1.1, -0.51, 0.0, 0.0, 0.0, 0.707, 0.707],      # 朝右
            "s7_twist_45":  [1.1, -0.51, 0.0, 0.0, 0.0, 0.9239,0.3827],    
            "s7_twist":     [1.1, -0.51, 0.0, 0.0, 0.0, 1.0,0.0],          

            "s8":           [0.12, -0.51, 0.0, 0.0, 0.0, 1.0,0.0],          # 朝前
           "s8_twist_45":  [0.12, -0.51, 0.0, 0.0, 0.0,-0.9239,0.3827 ],   # 右转45度
            "s8_twist":     [0.12, -0.51, 0.0, 0.0, 0.0, -0.707, 0.707],     # 右转45度（朝右）

            "s9":           [0.12, -0.92, 0.0, 0.0, 0.0, -0.707, 0.707],     # 朝右
           "s9_twist_45":  [0.12, -0.92, 0.0, 0.0, 0.0, -0.9239,0.3827],   # 左转45度
            "s9_twist":     [0.12, -0.92, 0.0, 0.0, 0.0, 1.0,0.0],          #
            # 可继续添加更多点位..
             "room_st":      [-0.80, -0.92, 0.0, 0.0, 0.0, 1.0,    0.0],     # 朝 180 度
          "room_st_twist_45": [-0.80, -0.92, 0.0, 0.0, 0.0, 0.9239, 0.3827],  # 转到 135 度
        "room_center":      [-0.80, -0.51, 0.0, 0.0, 0.0, 0.7071, 0.7071],  # 最后点，朝 90 度,
            
        }

SV = SharedVariables()


# ---------------------------- 初始化 ----------------------------
def init_move_base():
    # 等待 move_base Action 服务器启动（超时 5 秒）
    if not SV.move_base.wait_for_server(rospy.Duration(5)):
        rospy.logerr("无法连接 move_base action server...")
        sys.exit(1)
    rospy.loginfo("move_base action server 已连接")
    # 连接成功后发送初始位姿
    send_initialpose()

def send_initialpose():
    # ===== Topic 通信：创建一个发布者，向 /initialpose 话题发送消息 =====
    # 此话题用于告诉 AMCL 机器人在 map 坐标系中的初始位置和朝向
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
    initial_pose.pose.pose.orientation.w = 1.0   # 朝向 X 轴正方向
    rospy.sleep(1)  # 等待话题建立连接
    pub.publish(initial_pose)   # 发布消息
    rospy.loginfo("已发送初始位姿")

def init_tf_listener():
    # ===== TF 通信：初始化 TF 监听器，自动订阅 /tf 话题 =====
    # tf2_ros.Buffer 用于缓存坐标变换树
    # tf2_ros.TransformListener 负责接收 /tf 消息并填入 buffer
    SV.tf_buffer = tf2_ros.Buffer()
    SV.tf_listener = tf2_ros.TransformListener(SV.tf_buffer)
    rospy.sleep(1.0)  # 等待 buffer 填充足够的变换数据
    rospy.loginfo("TF 监听器已初始化")


# ---------------------------- 工具函数 ----------------------------
def send_nav_point_and_wait(target_pose_list, timeout=None):
    """
    使用 Action 发送一个导航目标点并等待到达
    """
    if len(target_pose_list) != 7:
        rospy.logerr(f"导航点列表长度错误: {len(target_pose_list)}")
        return

    # 查找点位名称（用于日志）
    name = "unknown"
    for k, v in SV.nav_point.items():
        if v == target_pose_list:
            name = k
            break

    # 构造 MoveBaseGoal 消息
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(*target_pose_list[:3]),
                                 Quaternion(*target_pose_list[3:]))

    # ===== Action 通信：发送目标点给 move_base =====
    rospy.loginfo(f"前往导航点: {name}")
    SV.move_base.send_goal(goal)

    # 等待导航完成（阻塞直到成功或失败）
    SV.move_base.wait_for_result()
    result = SV.move_base.get_result()
    if result:
        rospy.loginfo(f"成功到达 {name}")
    else:
        rospy.logwarn(f"未能到达 {name}")

def cancel_all_goals():
    # 取消当前所有导航目标
    SV.move_base.cancel_all_goals()
    rospy.loginfo("已取消所有导航目标")

def signal_handler(sig, frame):
    # Ctrl+C 时的清理函数
    rospy.loginfo("收到中断信号，正在退出...")
    cancel_all_goals()
    rospy.signal_shutdown("用户终止")
    sys.exit(0)


# ---------------------------- 定点巡航任务 ----------------------------
def cruise_points(points):
    """
    按顺序依次前往指定名称的导航点
    """
    rospy.loginfo(f"开始定点巡航，共 {len(points)} 个点")
    for name in points:
        if rospy.is_shutdown():
            break
        if name not in SV.nav_point:
            rospy.logerr(f"导航点 '{name}' 不存在，跳过")
            continue
        send_nav_point_and_wait(SV.nav_point[name])
        rospy.sleep(0.5)  # 每点到达后短暂停留
    rospy.loginfo("定点巡航完成")


# ---------------------------- 主程序 ----------------------------
if __name__ == '__main__':
    rospy.init_node('cruise_mode')
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化 move_base（连接 Action 服务器 + 发送初始位姿 Topic）
    rospy.loginfo("初始化 move_base ...")
    init_move_base()

    # 初始化 TF 监听器
    rospy.loginfo("初始化 TF 监听器 ...")
    init_tf_listener()

    # ===== Topic 通信：等待唤醒信号 (std_msgs/String) =====
    # 阻塞直到 /wakeup 话题收到一条消息
    # rospy.loginfo("等待 /wakeup 话题唤醒信号...")
    # rospy.wait_for_message("/wakeup", String, timeout=None)
    rospy.sleep(2)
    rospy.loginfo("收到唤醒，开始巡航")

    # 定义巡航路径
    patrol_path = ["s0","s0_twist_45","s0_twist","s1", "s1_twist_45", "s1_twist","s2", "s2_twist_45","s2_twist",
                   "s3", "s3_twist_45","s3_twist","s4", "s4_twist_45","s4_twist","s50","s51","s5","s5_twist_45","s5_twist","s60","s61",
                   "s6","s6_twist_45","s6_twist","s7","s7_twist_45","s7_twist","s8","s8_twist_45","s8_twist","s9","s9_twist_45","s9_twist",
                   "room_st","room_st_twist_45","room_center"]  # 可根据需要调整巡航点顺序和数量

    # 执行定点巡航（内部使用 Action 发送每个目标点）
    cruise_points(patrol_path)

    rospy.loginfo("任务结束，节点保持运行...")
    rospy.spin()

