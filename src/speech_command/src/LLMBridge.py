#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class LLMBridge:
    def __init__(self):
        rospy.init_node('llm_bridge', anonymous=True)
        
        # 订阅听写结果 (假设你的 speech_command 节点发布的 topic 是这个)
        rospy.Subscriber('/speech_to_text', String, self.tts_callback)
        
        # 发布播报指令
        self.tts_pub = rospy.Publisher('/factory/tts_text', String, queue_size=10)
        
        rospy.loginfo("大模型逻辑桥接已启动...")

    def call_llm(self, text):
        """在这里调用你的大模型 API"""
        # 模拟大模型处理
        rospy.loginfo(f"正在向大模型发送: {text}")
        
        # --- 在此替换你的大模型调用逻辑 ---
        # response = my_llm.chat(text)
        response = f"我已经收到你的指令：{text}，正在执行任务。"
        # --------------------------------
        
        return response

    def tts_callback(self, msg):
        user_input = msg.data
        if not user_input:
            return
            
        rospy.loginfo(f"用户: {user_input}")
        
        # 获取大模型回复
        reply = self.call_llm(user_input)
        
        # 发送给 TTS 进行播报
        self.tts_pub.publish(reply)

if __name__ == '__main__':
    try:
        bridge = LLMBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass