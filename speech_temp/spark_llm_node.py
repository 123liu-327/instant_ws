#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import requests
from std_msgs.msg import String

class SparkDualEnvPlannerNode:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('spark_dual_env_planner_node', anonymous=True)
        
        # 1. 填入你验证通过的密钥与 HTTP 接口参数
        self.api_key = "8898fd00d09d56be8985fedd123dc1ae"
        self.api_secret = "MjdhYzI0ZWQwYjIyM2VmOWlyZWVhYjg4" 
        self.url = "https://spark-api-open.xf-yun.com/x2/chat/completions"
        
        # 严格限制大模型的系统提示词
        self.system_prompt = """
你是智能无人工厂的中央调度员。你的任务是根据给定的【目标大类】和【备选货品列表】，从列表中筛选出绝对属于该目标大类的唯一货品。
核心要求：
1. 你的输出必须是标准的 JSON 格式，绝对不能包含任何标点符号、解释性文字、或者是 ```json 这样的 Markdown 标记。
2. 禁止进行任何形式的推理过程或深度思考，直接输出最终的 JSON 结果。
格式要求：{"selected_item": "选出的货品名称", "target_warehouse": "对应的标准车间名称"}
"""

        # 标准车间绝对映射表（防止模型随性发挥起错名字扣 15 分）
        self.warehouse_mapping = {
            "食品加工类": "食品加工车间", "食品类": "食品加工车间", "食品大类": "食品加工车间", "食品加工车间": "食品加工车间",
            "日用品类": "日用品加工车间", "日用品": "日用品加工车间", "日用品大类": "日用品加工车间", "日用品加工车间": "日用品加工车间",
            "电子产品类": "电子产品生产车间", "电子产品": "电子产品生产车间", "电子产品大类": "电子产品生产车间", "电子产品生产车间": "电子产品生产车间"
        }

        # 2. ROS 1 通信接口
        # 【订阅】来自主控状态机或视觉节点的触发 Topic
        # 期望的消息格式（JSON字符）： {"real_category": "食品加工类", "sim_category": "日用品类", "items": ["香蕉", "T恤衫", "手机"]}
        self.sub_trigger = rospy.Subscriber('/factory/llm_trigger', String, self.trigger_callback)
        
        # 【发布】最终拼装好的完整双环境语音播报文本给 TTS 节点
        self.pub_tts = rospy.Publisher('/factory/tts_text', String, queue_size=10)
        
        # 【发布】目标车间（包含真实和仿真车间，方便底层逻辑做分流导航）
        self.pub_target = rospy.Publisher('/factory/target_warehouses', String, queue_size=10)
        
        rospy.loginfo("🚀 [双环境] 星火大模型控制节点已成功就位，等待主控或视觉节点触发...")

    def call_spark_x(self, target_category, items_list):
        """单次大模型调用封装"""
        user_msg = f"【目标大类】：{target_category}\n【备选货品列表】：{items_list}\n请找出属于该大类的物品并按要求返回 JSON。"
        
        headers = {
            "Authorization": f"Bearer {self.api_key}:{self.api_secret}",
            "Content-Type": "application/json"
        }
        payload = {
            "model": "spark-x",
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_msg}
            ],
            "temperature": 0.1,
            "max_tokens": 150
        }
        
        try:
            response = requests.post(self.url, headers=headers, json=payload, timeout=20)
            if response.status_code == 200:
                res_json = response.json()
                content_str = res_json['choices'][0]['message']['content']
                clean_str = content_str.replace("```json", "").replace("```", "").strip()
                data = json.loads(clean_str)
                return data.get("selected_item"), data.get("target_warehouse")
            else:
                rospy.logerr(f"大模型响应失败，状态码: {response.status_code}")
                return None, None
        except Exception as e:
            rospy.logerr(f"请求大模型时发生异常: {e}")
            return None, None

    def trigger_callback(self, msg):
        """核心处理逻辑：当收到包含双环境大类和扫码物品的信号时触发"""
        rospy.loginfo("📥 [LLM] 接收到双环境分拣决策请求...")
        try:
            # 解析上游节点发过来的整合数据
            input_data = json.loads(msg.data)
            real_category = input_data.get("real_category") # 真实大类
            sim_category = input_data.get("sim_category")   # 仿真大类
            items_list = input_data.get("items")             # 扫码拿到的3个物品名字
            
            if not real_category or not sim_category or not items_list:
                rospy.logwarn("⚠️ 接收到的触发数据字段不完整，拒绝推理。")
                return

            # --- 线程线 1：推理真实环境对应的物品与车间 ---
            rospy.loginfo(f"🔍 正在进行 [真实环境-{real_category}] 推理...")
            real_item, real_wh = self.call_spark_x(real_category, items_list)
            real_warehouse_clean = self.warehouse_mapping.get(real_wh, self.warehouse_mapping.get(real_category, "未知车间"))

            # --- 线程线 2：推理仿真环境对应的物品与车间 ---
            rospy.loginfo(f"🔍 正在进行 [仿真环境-{sim_category}] 推理...")
            sim_item, sim_wh = self.call_spark_x(sim_category, items_list)
            sim_warehouse_clean = self.warehouse_mapping.get(sim_wh, self.warehouse_mapping.get(sim_category, "未知车间"))

            if real_item and sim_item:
                # --- 严格按照官方【说明1 - 完成标志与输出要求】合成最终文本（逗号、句号错一个扣15分！） ---
                broadcast_text = (
                    f"取得{real_item}属于{real_category}应放置在{real_warehouse_clean}，"
                    f"仿真环境中取得{sim_item}属于{sim_category}应放置在{sim_warehouse_clean}。"
                )
                
                # 打印到终端方便你调试查验
                rospy.loginfo("================================================================")
                rospy.loginfo(f"📢 拼装完成的官方规范播报文本如下:")
                print(broadcast_text)
                rospy.loginfo("================================================================")
                
                # 1. 发布给语音播报节点
                self.pub_tts.publish(String(data=broadcast_text))
                
                # 2. 发布车间结果给决策层/导航层
                target_result = {
                    "real_warehouse": real_warehouse_clean,
                    "sim_warehouse": sim_warehouse_clean
                }
                self.pub_target.publish(String(data=json.dumps(target_result)))
                rospy.loginfo("🚀 数据成功广播给语音节点及主控节点！")
            else:
                rospy.logerr("❌ 双线推理中存在失败，未生成语音指令。")

        except Exception as e:
            rospy.logerr(f"处理触发回调时发生严重异常: {e}")

if __name__ == '__main__':
    try:
        node = SparkDualEnvPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass