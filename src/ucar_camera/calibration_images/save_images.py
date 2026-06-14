#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaver:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('image_saver', anonymous=True)
        
        # 设置保存路径（默认home目录下的calibration_images文件夹）
        self.save_dir = os.path.expanduser('/home/ucar/instant_ws/src/ucar_camera/calibration_images')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # 初始化CV Bridge（用于ROS Image和OpenCV转换）
        self.bridge = CvBridge()
        
        # 订阅摄像头话题
        self.image_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.image_callback)
        
        # 控制保存频率（每隔2秒）
        self.last_save_time = time.time()
        self.save_interval = 2.0  # 单位：秒
        
        rospy.loginfo("Image saver started. Press Ctrl+C to stop.")

    def image_callback(self, msg):
        try:
            # 将ROS Image消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检查是否达到保存间隔
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_interval:
                # 生成文件名（时间戳）
                timestamp = rospy.Time.now().to_sec()
                filename = os.path.join(self.save_dir, f"image_{timestamp:.3f}.jpg")
                
                # 保存图像
                cv2.imwrite(filename, cv_image)
                rospy.loginfo(f"Saved: {filename}")
                
                # 更新最后保存时间
                self.last_save_time = current_time
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        saver = ImageSaver()
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        pass