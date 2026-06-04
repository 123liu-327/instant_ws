#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import requests
from std_msgs.msg import String

class SparkDualEnvPlannerNode:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('spark_dual_env_planner_node', anonymous=True)
        
        # 2. 原封不动抄写你上面完全正确的单线初始化配置
        self.api_key = "8898fd00d09d56be8985fedd123dc1ae"
        self.api_secret = "MjdhYzI0ZWQwYjIyM2VmOWIyZWVhYjg4" 
        self.url = "https://spark-api-open.xf-yun.com/x2/chat/completions"
        
        self.system_prompt = """
你是智能无人工厂的中央调度员。你的任务是根据给定的【目标大类】和【备选货品列表】，从列表中筛选出绝对属于该目标大类的唯一货品。

【核心要求】：
1. 你的输出必须是标准的 JSON 格式，绝对不能包含任何标点符号、解释性文字、或者是 ```json 这样的 Markdown 标记。
2. 禁止进行任何形式的推理过程或深度思考（不要返回 reasoning_content），直接输出最终的 JSON 结果。

格式要求：{"selected_item": "选出的货品名称", "target_warehouse": "对应的标准车间名称"}
"""
        # 赛规严格映射表
        self.warehouse_mapping = {
            "食品加工类": "食品加工车间", "食品类": "食品加工车间", "食品大类": "食品加工车间", "食品加工车间": "食品加工车间",
            "日用品类": "日用品加工车间", "日用品": "日用品加工车间", "日用品大类": "日用品加工车间", "日用品加工车间": "日用品加工车间",
            "电子产品类": "电子产品生产车间", "电子产品": "电子产品生产车间", "电子产品大类": "电子产品生产车间", "电子产品生产车间": "电子产品生产车间"
        }

        # 3. ROS 通信接口
        self.sub_trigger = rospy.Subscriber('/factory/llm_trigger', String, self.trigger_callback)
        self.pub_tts = rospy.Publisher('/factory/tts_text', String, queue_size=10)
        self.pub_target = rospy.Publisher('/factory/target_warehouses', String, queue_size=10)
        
        rospy.loginfo("🚀 [双环境] 节点已完全复刻纯 HTTP 逻辑，就位等待触发...")

    def call_by_http(self, target_category, items_list):
        """完全复制你上面100%成功的网络请求函数，一个字不改"""
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
            response = requests.post(self.url, headers=headers, json=payload, timeout=30)
            return response
        except requests.exceptions.Timeout:
            rospy.logerr("❌ 请求超时！星火大模型没有在指定时间内响应。")
            return None
        except Exception as e:
            rospy.logerr(f"HTTP 请求异常: {e}")
            return None

    def parse_single_env(self, target_category, raw_response_text):
        """单路数据安全清洗防火墙"""
        try:
            res_json = json.loads(raw_response_text)
            content_str = res_json['choices'][0]['message']['content']
            clean_str = content_str.replace("```json", "").replace("```", "").strip()
            data = json.loads(clean_str)
            
            item = data.get("selected_item")
            warehouse = data.get("target_warehouse")
            
            standard_warehouse = self.warehouse_mapping.get(warehouse)
            if not standard_warehouse:
                standard_warehouse = self.warehouse_mapping.get(target_category, "未知车间")
            return item, standard_warehouse
        except Exception as e:
            rospy.logerr(f"解析失败: {e}")
            return None, None

    def trigger_callback(self, msg):
        """收到触发后，用你成功的方法跑两次"""
        rospy.loginfo("📥 收到双环境分拣决策请求...")
        try:
            input_data = json.loads(msg.data)
            real_category = input_data.get("real_category") 
            sim_category = input_data.get("sim_category")   
            items_list = input_data.get("items")             
            
            # 1. 跑第一路：真实环境
            rospy.loginfo(f"📡 正在发起 [真实环境-{real_category}] 请求...")
            res_real = self.call_by_http(real_category, items_list)
            
            # 2. 跑第二路：仿真环境
            rospy.loginfo(f"📡 正在发起 [仿真环境-{sim_category}] 请求...")
            res_sim = self.call_by_http(sim_category, items_list)
            
            if res_real and res_real.status_code == 200 and res_sim and res_sim.status_code == 200:
                real_item, real_wh = self.parse_single_env(real_category, res_real.text)
                sim_item, sim_wh = self.parse_single_env(sim_category, res_sim.text)
                
                if real_item and sim_item:
                    # 拼装双环境官方要求的完美句式
                    broadcast_text = (
                        f"取得{real_item}属于{real_category}应放置在{real_wh}，"
                        f"仿真环境中取得{sim_item}属于{sim_category}应放置在{sim_wh}。"
                    )
                    
                    print("\n================== ✅ 双线数据完美闭环 ==================")
                    print(broadcast_text)
                    print("========================================================\n")
                    
                    # 广播给其他节点
                    self.pub_tts.publish(String(data=broadcast_text))
                    self.pub_target.publish(String(data=json.dumps({"real_warehouse": real_wh, "sim_warehouse": sim_wh})))
            else:
                rospy.logerr(f"请求失败！状态码：真实环境={res_real.status_code if res_real else 'None'}, 仿真环境={res_sim.status_code if res_sim else 'None'}")
                
        except Exception as e:
            rospy.logerr(f"触发逻辑处理异常: {e}")

if __name__ == '__main__':
    try:
        node = SparkDualEnvPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass