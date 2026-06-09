#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from std_msgs.msg import String

def tts_callback(msg):
    text = msg.data
    rospy.loginfo("正在播报: %s", text)
    # 使用系统原生工具进行测试，无需 SDK 鉴权
    # 确保你的小车安装了 espeak: sudo apt-get install espeak
    os.system(f'espeak -v zh "{text}"')

def listener():
    rospy.init_node('tts_test_node', anonymous=True)
    rospy.Subscriber('/factory/tts_text', String, tts_callback)
    rospy.loginfo("TTS 测试节点已启动，等待指令...")
    rospy.spin()

if __name__ == '__main__':
    listener()