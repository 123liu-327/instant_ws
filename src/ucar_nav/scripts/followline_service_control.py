# followline_service_control.py
#!/usr/bin/env python3

import rospy
from ucar_nav.srv import EnableLaneFollow , EnableLaneFollowResponse 
from multiprocessing import Event
"""
该节点用于巡线部分任务相关节点（包括避障）的启停
当向该节点发送服务请求时，会更新共享状态中的flag_task和lane_mode，并触发enable_event，并更新避障任务的状态

"""


class LaneControlServer:
    def __init__(self, shared_state):
        """
        :param shared_state: 包含以下属性的对象:
            - lock: 跨进程锁
            - flag_task: Value('i')
            - lane_mode: Value('i')
            - enable_event: Event()
        """
        rospy.init_node('followline_service_control')
        rospy.loginfo("这里是巡线控制台！服务端已就绪！等待触发中...")
        self.shared_state = shared_state
        
        self.service = rospy.Service(
            '/enable_lane_follow',
            EnableLaneFollow,
            self.handle_request
        )

    def handle_request(self, req):
        with self.shared_state.lock: # 使用锁保护共享状态

            # 根据请求参数更新共享状态，包括flag_task和lane_mode以及enable_event
            self.shared_state.flag_task.value = int(req.enable)
            self.shared_state.lane_mode.value = req.mode
            self.shared_state.enable_event.set() if req.enable else self.shared_state.enable_event.clear()

            # 请求为True时，触发避障任务，更新参数enable_event
            rospy.set_param("/enable_avoidance", req.enable)
            if req.enable:
                rospy.loginfo("巡线任务event已触发！更新flag_task为%d, lane_mode为%d", self.shared_state.flag_task.value, self.shared_state.lane_mode.value)
        return EnableLaneFollowResponse(True, "Success")

def run_control_node(shared_state):
    """独立运行函数，供主进程调用"""
    server = LaneControlServer(shared_state)
    rospy.spin()

import sys
sys.modules[__name__].run_control_node = run_control_node
