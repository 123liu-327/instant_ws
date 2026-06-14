#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import requests
from std_msgs.msg import String, Int32 # 导入 Int32 用于接收状态

class SparkDualEnvPlannerNode:
    def __init__(self):
        rospy.init_node('spark_dual_env_planner_node', anonymous=True)
        # 必须添加这两个初始化，否则代码运行时会找不到变量
        self.system_prompt = "你是智能分拣调度员。请根据【目标大类】和【备选货品列表】，找出最匹配的物品，按 JSON 返回：{\"selected_item\": \"名称\", \"target_warehouse\": \"车间\"}"
        self.pub_target = rospy.Publisher('/factory/target_warehouse', String, queue_size=10)
        self.is_awake = False  # 新增：唤醒状态锁，默认为 False
        
        # 状态机初始化: WAITING -> SCANNING -> DECIDING
        self.task_state = "WAITING" 
        self.raw_voice_text = ""
        self.scanned_items = []
        
        # API 配置 (保持不变)
        self.api_key = "8898fd00d09d56be8985fedd123dc1ae"
        self.api_secret = "MjdhYzI0ZWQwYjIyM2VmOWIyZWVhYjg4" 
        self.url = "https://spark-api-open.xf-yun.com/x2/chat/completions"
        
        # 映射表 (已修复拼写)
        self.warehouse_mapping = {
            "食品加工类": "食品加工车间", "食品类": "食品加工车间", "食品大类": "食品加工车间", 
            "食品加工车间": "食品加工车间",
            "日用品类": "日用品加工车间", "日用品": "日用品加工车间", "日用品大类": "日用品加工车间", 
            "日用品加工车间": "日用品加工车间",
            "电子产品类": "电子产品生产车间", "电子产品": "电子产品生产车间", "电子产品大类": "电子产品生产车间", 
            "电子产品生产车间": "电子产品生产车间"
        }

        # ROS 接口
        self.sub_voice = rospy.Subscriber('/factory/voice_raw_text', String, self.voice_callback)
        self.sub_state = rospy.Subscriber('/factory/task_state', Int32, self.state_callback) # 新增状态订阅
        self.sub_qr = rospy.Subscriber('/factory/qr_item', String, self.qr_callback)
        
        self.pub_tts = rospy.Publisher('/factory/tts_text', String, queue_size=10)
        
        rospy.loginfo("🧠 [状态机大脑] 已激活，等待 C++ 指令...")

    def state_callback(self, msg):
        """接收 C++ 的状态同步：1=SCANNING"""
        if msg.data == 1:
            self.task_state = "SCANNING"
            self.scanned_items = [] # 每次进入扫码模式，清空旧物品
            rospy.loginfo("🚥 [同步] 收到 C++ 指令，进入 SCANNING 状态")

    def voice_callback(self, msg):
        """语音处理：严格遵循唤醒-待命逻辑"""
        raw_text = msg.data
        
        # 1. 检查是否包含唤醒词
        if "小飞小飞" in raw_text:
            self.is_awake = True
            self.raw_voice_text = "" # 清空之前的旧指令
            #self.pub_tts.publish(String(data="小飞在呢，请下达任务。"))
            rospy.loginfo("✅ [唤醒] 系统已激活，等待具体指令...")
            self.raw_voice_text = raw_text
            rospy.loginfo(f"👂 [接收任务] 存入指令: {self.raw_voice_text}")
            self.pub_tts.publish(String(data="收到任务，正在准备分拣。"))
            return # 唤醒后直接返回，等待下一次语音输入真正的任务

        # 2. 如果未处于唤醒状态，直接无视所有输入
        if not self.is_awake:
            rospy.loginfo(f"🔇 [静默] 忽略非唤醒指令: {raw_text}")
            return

        # 3. 如果已经唤醒，则将当前输入作为任务指令
        
        
        # 可以在这里根据需要决定是否自动重置唤醒状态 (例如任务完成后)

    def qr_callback(self, msg):
        if self.task_state != "SCANNING": return # 状态锁
        
        try:
            qr_data = json.loads(msg.data)
            item_name = qr_data.get("name")
            if item_name and item_name not in self.scanned_items:
                self.scanned_items.append(item_name)
                rospy.loginfo(f"👀 [扫码] 进度 {len(self.scanned_items)}/3: {item_name}")
            
            if len(self.scanned_items) == 3:
                self.task_state = "DECIDING"
                rospy.loginfo("⚡ [条件齐备] 触发大模型决策")
                self.execute_combined_decision()
        except: pass

    # ... (后续 execute_combined_decision 等函数逻辑不变)

    def execute_combined_decision(self):
        rospy.loginfo("⚡ [决策] 正在启动双环境并行调度...")
        
        # 1. 语义拆解：获取干净的类别
        task_info = self.extract_task_categories(self.raw_voice_text)
        real_cat = task_info.get("real", "食品大类") # 提供默认值防止空指针
        sim_cat = task_info.get("sim", "日用品大类")
        
        # 2. 传入清洗后的变量，发起并行请求
        rospy.loginfo(f"📡 发起决策: 真实[{real_cat}], 仿真[{sim_cat}]")
        res_real = self.call_by_http(real_cat, self.scanned_items)
        res_sim = self.call_by_http(sim_cat, self.scanned_items)
        
        try:
            if res_real and res_real.status_code == 200 and res_sim and res_sim.status_code == 200:
                # 3. 使用清洗后的 real_cat/sim_cat 进行解析
                real_item, real_wh = self.parse_single_env(real_cat, res_real.text)
                sim_item, sim_wh = self.parse_single_env(sim_cat, res_sim.text)
                
                if real_item and sim_item:
                    broadcast_text = f"取得{real_item}放入{real_wh}，仿真环境取得{sim_item}放入{sim_wh}。"
                    print(f"\n✅ 决策结果: {broadcast_text}\n")
                    
                    self.pub_tts.publish(String(data=broadcast_text))
                    # 确保 self.pub_target 已在 __init__ 初始化
                    self.pub_target.publish(String(data=json.dumps({"real_warehouse": real_wh, "sim_warehouse": sim_wh})))
                else:
                    rospy.logerr("❌ 大模型返回数据异常，请检查 Prompt")
            else:
                rospy.logerr("❌ 网络请求失败")
        except Exception as e:
            rospy.logerr(f"决策执行异常: {e}")
        finally:
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
        
    def extract_task_categories(self, raw_text):
        """
        专门负责：把裁判的啰嗦话变成机器能懂的 JSON (提取 A2, B2)
        """
        prompt = f"从以下指令中提取任务类别，直接返回 JSON，格式如: {{\"real\": \"食品大类\", \"sim\": \"日用品大类\"}}。指令：{raw_text}"
        
        headers = {
            "Authorization": f"Bearer {self.api_key}:{self.api_secret}",
            "Content-Type": "application/json"
        }
        payload = {
            "model": "spark-x",
            "messages": [{"role": "user", "content": prompt}],
            "temperature": 0.0 # 严谨模式，不需要创意
        }
        
        try:
            response = requests.post(self.url, headers=headers, json=payload, timeout=10)
            res_json = json.loads(response.text)
            content = res_json['choices'][0]['message']['content']
            clean_str = content.replace("```json", "").replace("```", "").strip()
            return json.loads(clean_str)
        except Exception as e:
            rospy.logerr(f"指令提取失败: {e}")
            return {"real": "未知", "sim": "未知"}

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

    