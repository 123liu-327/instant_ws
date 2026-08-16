#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR Code URL Parser Node
监听 /qr_code_result 话题，解析二维码中的网址，返回 JSON 格式结果
"""

import rospy
import requests
import json
from std_msgs.msg import String
from std_msgs.msg import String as JsonString  # 使用 String 发布 JSON

class QRCodeURLParser:
    def __init__(self):
        rospy.init_node('qr_url_parser', anonymous=True)
        
        # 订阅二维码结果话题
        self.qr_sub = rospy.Subscriber('/qr_code_result', String, self.qr_callback)
        
        # 发布解析结果话题
        self.result_pub = rospy.Publisher('/qr_url_parsed', JsonString, queue_size=10)
        
        # 设置请求超时
        self.timeout = 10  # 秒
        
        rospy.loginfo("QR Code URL Parser initialized")
        rospy.loginfo("Listening to /qr_code_result")
        rospy.loginfo("Publishing to /qr_url_parsed")

    def qr_callback(self, msg):
        """处理二维码检测结果"""
        qr_data = msg.data.strip()
        rospy.loginfo(f"Received QR code data: {qr_data}")
        
        # 检查是否是网址
        if self.is_url(qr_data):
            try:
                # 发送 HTTP 请求
                response = requests.get(qr_data, timeout=self.timeout)
                response.raise_for_status()  # 检查 HTTP 错误
                
                # 尝试解析 JSON
                try:
                    json_data = response.json()
                    result = {
                        "status": "success",
                        "url": qr_data,
                        "json_data": json_data
                    }
                except json.JSONDecodeError:
                    # 如果不是 JSON，返回文本
                    result = {
                        "status": "success",
                        "url": qr_data,
                        "text_data": response.text
                    }
                
                # 发布结果
                result_msg = JsonString()
                result_msg.data = json.dumps(result, ensure_ascii=False, indent=2)
                self.result_pub.publish(result_msg)
                
                rospy.loginfo(f"Successfully parsed URL: {qr_data}")
                
            except requests.exceptions.RequestException as e:
                # 请求失败
                error_result = {
                    "status": "error",
                    "url": qr_data,
                    "error": str(e),
                    "error_type": "request_error"
                }
                result_msg = JsonString()
                result_msg.data = json.dumps(error_result, ensure_ascii=False, indent=2)
                self.result_pub.publish(result_msg)
                
                rospy.logwarn(f"Failed to request URL {qr_data}: {e}")
                
        else:
            # 不是网址
            result = {
                "status": "not_url",
                "data": qr_data,
                "message": "QR code data is not a valid URL"
            }
            result_msg = JsonString()
            result_msg.data = json.dumps(result, ensure_ascii=False, indent=2)
            self.result_pub.publish(result_msg)
            
            rospy.loginfo(f"QR code data is not a URL: {qr_data}")

    def is_url(self, string):
        """简单检查是否是网址"""
        return string.startswith(('http://', 'https://'))

if __name__ == '__main__':
    try:
        parser = QRCodeURLParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass