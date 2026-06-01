import json
import requests

class SparkFactoryHttpPlanner:
    def __init__(self):
        # 1. 严格使用你最新截图（带20万额度）的密钥
        self.api_key = "8898fd00d09d56be8985fedd123dc1ae"
        # ⚠️请注意：检查你刚才复制的 Secret 尾部有没有丢字符，要和控制台完全一致
        self.api_secret = "MjdhYzI0ZWQwYjIyM2VmOWIyZWVhYjg4" 
        
        # 2. 严格使用截图里的 X2 HTTP 接口地址
        self.url = "https://spark-api-open.xf-yun.com/x2/chat/completions"
        
        # 💡 优化系统提示词：强力压制深度思考，强迫模型直接秒吐JSON，杜绝超时
        self.system_prompt = """
你是智能无人工厂的中央调度员。你的任务是根据给定的【目标大类】和【备选货品列表】，从列表中筛选出绝对属于该目标大类的唯一货品。

【核心要求】：
1. 你的输出必须是标准的 JSON 格式，绝对不能包含任何标点符号、解释性文字、或者是 ```json 这样的 Markdown 标记。
2. 禁止进行任何形式的推理过程或深度思考（不要返回 reasoning_content），直接输出最终的 JSON 结果。

格式要求：{"selected_item": "选出的货品名称", "target_warehouse": "对应的标准车间名称"}
"""

    def call_by_http(self, target_category, items_list):
        user_msg = f"【目标大类】：{target_category}\n【备选货品列表】：{items_list}\n请找出属于该大类的物品并按要求返回 JSON。"
        
        headers = {
            "Authorization": f"Bearer {self.api_key}:{self.api_secret}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": "spark-x", # 👈 已经确定的正确 model 名字
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_msg}
            ],
            "temperature": 0.1,
            "max_tokens": 150 # 👈 新增限制：告诉大模型别长篇大论，速度会变快很多！
        }
        
        try:
            # 👈 修正：将 timeout 提高到 30 秒，容忍网络抖动
            response = requests.post(self.url, headers=headers, json=payload, timeout=30)
            return response
        except requests.exceptions.Timeout:
            print("❌ 请求超时！星火大模型没有在指定时间内响应，请检查当前网络状态。")
            return None
        except Exception as e:
            print(f"HTTP 请求异常: {e}")
            return None
        
    def parse_and_broadcast_safely(self, target_category, raw_response_text):
        """💡 新增的强规则映射函数：硬编码死守赛规格式，自动擦除不合规的车间名"""
        try:
            # 1. 解析最外层 HTTP 响应
            res_json = json.loads(raw_response_text)
            content_str = res_json['choices'][0]['message']['content']
            
            # 2. 解析内层大模型返回的 JSON
            clean_str = content_str.replace("```json", "").replace("```", "").strip()
            data = json.loads(clean_str)
            
            item = data.get("selected_item")
            warehouse = data.get("target_warehouse")
            
            # 🛑 赛规严格映射表：把大模型随性发挥的名字（如"日用品类"）强行掰正为规则要求的"车间"
            warehouse_mapping = {
                "食品加工类": "食品加工车间", "食品类": "食品加工车间", "食品大类": "食品加工车间", "食品加工车间": "食品加工车间",
                "日用品类": "日用品加工车间", "日用品": "日用品加工车间", "日用品大类": "日用品加工车间", "日用品加工车间": "日用品加工车间",
                "电子产品类": "电子产品生产车间", "电子产品": "电子产品生产车间", "电子产品大类": "电子产品生产车间", "电子产品生产车间": "电子产品生产车间"
            }
            
            # 执行纠错逻辑
            standard_warehouse = warehouse_mapping.get(warehouse)
            if not standard_warehouse:
                # 兜底防火墙：如果大模型吐出的车间无法识别，直接用当前的目标大类强刷标准名称
                standard_warehouse = warehouse_mapping.get(target_category, "未知车间")
                
            # 3. 严格按照比赛【子任务 1 规则图片】要求的格式拼接（一字不差）
            broadcast_text = (
                f"取得{item}属于{target_category}应放置在{standard_warehouse}，"
                f"仿真环境中取得{item}属于{target_category}应放置在{standard_warehouse}。"
            )
            
            print("\n================== ✅ 智能车决策数据流处理成功 ==================")
            print(f"📦 筛选出的唯一货品: {item}")
            print(f"🏭 修正后的标准车间: {standard_warehouse}")
            print(f"📢 语音合成节点（TTS）即将完全照抄播报的文本:")
            print(broadcast_text)
            print("================================================================\n")
            
            return item, standard_warehouse, broadcast_text

        except Exception as e:
            print(f"❌ 防火墙解析失败！错误原因: {e}")
            return None, None, None


if __name__ == "__main__":
    planner = SparkFactoryHttpPlanner()
    print("🚀 正在发起纯 HTTP POST 协议请求...")
    
    test_category = "日用品类"
    test_items = ["棉被", "苹果", "笔记本电脑"]
    
    res = planner.call_by_http(test_category, test_items)
    
    if res is not None:
        print(f"状态码 (Status Code): {res.status_code}")
        
        if res.status_code == 200:
            # 👈 串联逻辑：把 call_by_http 拿到的 res.text 传给防火墙函数做最终的数据清洗与纠错
            planner.parse_and_broadcast_safely(test_category, res.text)
        else:
            print(f"服务器返回错误内容: {res.text}")