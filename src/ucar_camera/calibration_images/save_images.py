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
        
        # 保存路径可通过私有参数 ~save_dir 配置。
        default_save_dir = '/home/ucar/instant_ws/src/ucar_camera/calibration_images'
        configured_save_dir = rospy.get_param('~save_dir', default_save_dir)
        self.save_dir = os.path.abspath(
            os.path.expandvars(os.path.expanduser(configured_save_dir))
        )
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        
        # 初始化CV Bridge（用于ROS Image和OpenCV转换）
        self.bridge = CvBridge()
        
        # 订阅摄像头话题
        self.image_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.image_callback)
        
        # 保存间隔可通过私有参数 ~save_interval 配置，单位为秒。
        self.save_interval = float(rospy.get_param('~save_interval', 2.0))
        if self.save_interval <= 0:
            raise ValueError('~save_interval must be greater than 0')

        # 总采集时长可通过 ~record_duration 配置；0 表示不限制时长。
        self.record_duration = float(rospy.get_param('~record_duration', 0.0))
        if self.record_duration < 0:
            raise ValueError('~record_duration must be greater than or equal to 0')
        self.record_start_time = None
        self.last_save_time = time.time()

        duration_text = (
            'unlimited' if self.record_duration == 0
            else f'{self.record_duration:.3f}s'
        )
        
        rospy.loginfo(
            "Image saver started. save_dir=%s, save_interval=%.3fs, "
            "record_duration=%s. Press Ctrl+C to stop.",
            self.save_dir,
            self.save_interval,
            duration_text,
        )

    def image_callback(self, msg):
        try:
            current_time = time.time()
            if self.record_start_time is None:
                self.record_start_time = current_time

            if (
                self.record_duration > 0
                and current_time - self.record_start_time >= self.record_duration
            ):
                rospy.loginfo(
                    "Recording finished after %.3f seconds. Saved images are in %s",
                    self.record_duration,
                    self.save_dir,
                )
                rospy.signal_shutdown('record duration reached')
                return

            # 将ROS Image消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检查是否达到保存间隔
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
