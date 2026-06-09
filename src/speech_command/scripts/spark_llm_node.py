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
        
        # ─── 💾 新增：核心“多源蓄水池”缓冲区变量 ───
        self.raw_voice_text = ""   # 存储 C++ 抛过来的裁判语音原话
        self.scanned_items = []     # 存储陆续扫描到的 3 个二维码物品名称
        
        # 2. 原封不动保留你完全正确的单线初始化配置
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
            "食品加工类": "食品加工车间", "食品类": "食品加工车间", "食品大类": "食品加工车间", "食品加工车间": "食品加工车ain",
            "日用品类": "日用品加工车间", "日用品": "日用品加工车间", "日用品大类": "日用品加工车间", "日用品加工车间": "日用品加工车间",
            "电子产品类": "电子产品生产车间", "电子产品": "电子产品生产车间", "电子产品大类": "电子产品生产车间", "电子产品生产车间": "电子产品生产车间"
        }

        # 3. 🔌 更新后的 ROS 通信接口
        # 【耳听】：订阅 C++ 传过来的裁判原句
        self.sub_voice = rospy.Subscriber('/factory/voice_raw_text', String, self.voice_callback)
        # 【眼看】：订阅视觉节点发过来的单个二维码 JSON（截图中的格式：{"name": "芯片"}）
        self.sub_qr = rospy.Subscriber('/factory/qr_item', String, self.qr_callback)
        
        # 【嘴说与底盘】：决策完后的广播话题
        self.pub_tts = rospy.Publisher('/factory/tts_text', String, queue_size=10)
        self.pub_target = rospy.Publisher('/factory/target_warehouses', String, queue_size=10)
        
        rospy.loginfo("🧠 [双环境大脑合流] 节点已完全就位，正在监听起跑线语音与分拣区 3 处扫码...")

    def voice_callback(self, msg):
        """ 【第一步】：起跑线听任务（接收裁判原话） """
        self.raw_voice_text = msg.data
        rospy.loginfo(f"👂 [蓄水池] 成功存入听觉原话: '{self.raw_voice_text}'")

    def qr_callback(self, msg):
        """ 【第二步】：物品区连续看 3 个物品（单路触发漏斗） """
        try:
            # 严格解析截图格式：{"name": "芯片"}
            qr_data = json.loads(msg.data)
            item_name = qr_data.get("name")
            
            # 去重累加机制：防止因为距离近导致同一个码被视觉节点高频连发扫入
            if item_name and item_name not in self.scanned_items:
                self.scanned_items.append(item_name)
                rospy.loginfo(f"👀 [蓄水池] 视觉扫码成功 [{len(self.scanned_items)}/3]: {item_name}")
            
            # 💡 终极合流扳机：只有当 3 个物品全部集齐时，才触发大模型决策
            if len(self.scanned_items) == 3:
                if self.raw_voice_text:
                    rospy.loginfo("⚡ [条件全齐] 听觉与 3 处视觉数据全部合流！开始唤醒大模型核心...")
                    self.execute_combined_decision()
                else:
                    rospy.logwarn("⚠️ 扫齐了 3 个物品，但起跑线尚未收到裁判命令，大脑挂起等待...")
                    
        except Exception as e:
            rospy.logerr(f"❌ 蓄水池解析视觉二维码 JSON 失败: {e}")

    def execute_combined_decision(self):
        """ 【第三步】：调用星火大模型，多路并行决策 """
        rospy.loginfo("📥 开始处理双环境分拣大模型决策...")
        try:
            # 🧠 兼容性漏斗映射：直接让星火从裁判一整句话中智能捕捉提取真实环境与仿真环境大类！
            # 即使裁判说：“请分拣食品类大类和日用品大类”，星火也拥有极强的泛化提取能力
            real_category = self.raw_voice_text  
            sim_category = self.raw_voice_text   
            items_list = self.scanned_items  # 这里已经凑齐了完整的数组，例如 ["芯片", "苹果", "牙刷"]
            
            # 1. 跑第一路：真实环境大模型请求
            rospy.loginfo(f"📡 正在发起 [真实环境] 大模型请求...")
            res_real = self.call_by_http(real_category, items_list)
            
            # 2. 跑第二路：仿真环境大模型请求
            rospy.loginfo(f"📡 正在发起 [仿真环境] 大模型请求...")
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
                    
                    # 广播给其他节点（C++ 语音节点会通过 /factory/tts_text 抓取到这里并当场念出来）
                    self.pub_tts.publish(String(data=broadcast_text))
                    self.pub_target.publish(String(data=json.dumps({"real_warehouse": real_wh, "sim_warehouse": sim_wh})))
                else:
                    rospy.logerr("❌ 大模型返回的数据经过防火墙清洗后存在空值，请检查 Prompt 约束！")
            else:
                rospy.logerr(f"❌ 请求失败！状态码：真实环境={res_real.status_code if res_real else 'None'}, 仿真环境={res_sim.status_code if res_sim else 'None'}")
                
        except Exception as e:
            rospy.logerr(f"大模型触发决策执行异常: {e}")
        finally:
            # 🔴 关键安全重置：不论成功失败，立刻清空视觉蓄水池，为下一轮比赛或重新测试做好基础准备
            self.scanned_items = []

    def call_by_http(self, target_category, items_list):
        """原封不动保留的核心网络请求函数"""
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
        """原封不动保留的单路数据安全清洗防火墙"""
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

if __name__ == '__main__':
    try:
        node = SparkDualEnvPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass