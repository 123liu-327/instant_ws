#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf.transformations as tf

class ImuToEuler:
    def __init__(self):
        rospy.init_node('imu_to_euler', anonymous=True)
        
        # 订阅IMU话题，根据实际话题名称修改
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # 初始化变量
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        rospy.loginfo("IMU to Euler angle converter initialized")

    def imu_callback(self, msg):
        # 从IMU消息中获取四元数
        orientation = msg.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        
        # 将四元数转换为欧拉角（roll, pitch, yaw）
        try:
            (self.roll, self.pitch, self.yaw) = tf.euler_from_quaternion(quaternion)
            
            # 将弧度转换为度（可选）
            roll_deg = self.roll * 180.0 / 3.141592653589793
            pitch_deg = self.pitch * 180.0 / 3.141592653589793
            yaw_deg = self.yaw * 180.0 / 3.141592653589793
            
            # 打印结果
            rospy.loginfo_throttle(1.0, 
                "Euler angles [deg]: Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
                roll_deg, pitch_deg, yaw_deg)
                
        except Exception as e:
            rospy.logerr("Error converting quaternion to Euler angles: %s", str(e))

if __name__ == '__main__':
    try:
        converter = ImuToEuler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass