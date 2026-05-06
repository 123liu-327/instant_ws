#!/usr/bin/env python3

import os
from dec_msgs.msg import Detection,DetectionArray
import traceback
import sys
import logging
from std_msgs.msg import String
# 1. 设置环境变量
os.environ['ROSCONSOLE_FORMAT'] = '${severity} ${message}'
os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = ''  # 禁用ROS默认日志配置

# 2. 手动配置logging使用标准级别
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)

# 3. 强制定义标准日志级别
logging.addLevelName(logging.DEBUG, "DEBUG")
logging.addLevelName(logging.INFO, "INFO")
logging.addLevelName(logging.WARNING, "WARNING")
logging.addLevelName(logging.ERROR, "ERROR")
logging.addLevelName(logging.CRITICAL, "CRITICAL")

# 4. 确保在导入ROS模块前设置好日志


import numpy as np
import cv2
from rknnlite.api import RKNNLite

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Model and Detection parameters
RKNN_MODEL = '/home/ucar/rknn-toolkit2-1.6.0/rknn_toolkit_lite2/examples/resnet18/best.rknn' 

logging.basicConfig(level=logging.INFO)  # 手动设置日志级别
import rospy
# Detection thresholds
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
# Default image size for the model input
DEFAULT_IMG_SIZE = 640

# Class names for detection
CLASSES = ("des1", "des2", "des3", "fru1", "fru2", "fru3", "veg1", "veg2", "veg3")

class YOLOv5RKNNNode:
    def __init__(self):
        rospy.init_node('detect_node', anonymous=False)
        
        # 初始化RKNN
        self.rknn_lite = RKNNLite(verbose=False)
   #     print('RKNN model platform:', self.rknn_lite.get_sdk_version())
        self.load_rknn_model()

        
        # 初始化CV桥
        self.bridge = CvBridge()
        # 处理频率控制
        self.rate = rospy.Rate(10)        
        # 发布者和订阅者
        self.image_pub = rospy.Publisher('/yolov5/detection_result', Image, queue_size=1)

       # self.start_sub = rospy.Subscriber('/detection_start',String,None)
        while not rospy.is_shutdown():
            msg=rospy.wait_for_message("/detection_start",String)
            if(msg.data=='start'):
                break
            self.rate.sleep()
        
        self.image_sub = rospy.Subscriber('/ucar_camera/image_raw', Image, self.image_callback)
        self.dec_pub= rospy.Publisher("/detection_result",DetectionArray,queue_size=10)    

    def load_rknn_model(self):
        """加载RKNN模型并初始化运行时环境"""
        rospy.loginfo('--> Loading RKNN model')
        ret = self.rknn_lite.load_rknn(RKNN_MODEL)
        if ret != 0:
            rospy.logerr(f'Load RKNN model {RKNN_MODEL} failed. Error code: {ret}')
            rospy.signal_shutdown("RKNN load failed")
            return
        
        rospy.loginfo('RKNN model loaded successfully.')
        
        # 初始化运行时环境
        rospy.loginfo('--> Initializing runtime environment')
        try:
            # 关键修复：添加兼容性参数
            ret = self.rknn_lite.init_runtime(
                core_mask=RKNNLite.NPU_CORE_0_1_2
            )
            if ret != 0:
                rospy.logwarn("使用备用兼容模式...")
                # 尝试简化兼容模式
                ret = self.rknn_lite.init_runtime(
                  
                )
                if ret != 0:
                    rospy.logerr(f'运行时环境初始化失败. 错误代码: {ret}')
                    rospy.signal_shutdown("RKNN init failed")
                    return
            rospy.loginfo('运行时环境初始化成功.')
        except Exception as e:
            rospy.logerr(f"初始化RKNN运行时错误: {str(e)}")
            # 获取详细平台信息
            try:
                platform_info = subprocess.check_output("cat /proc/device-tree/model", shell=True)
                rospy.loginfo(f"硬件平台: {platform_info.decode().strip()}")
            except:
                rospy.logwarn("无法获取硬件平台信息")
            rospy.signal_shutdown("RKNN初始化异常")
    
    def image_callback(self, msg):
        """处理传入的图像消息"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo(f"Converted OpenCV image shape: {cv_image.shape}")
            # 处理图像并获取检测结果
            result_image = self.process_image(cv_image)
            
            # 发布结果图像
            if result_image is not None:
                result_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
                self.image_pub.publish(result_msg)
            rospy.loginfo("receive image")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")
            rospy.logerr(traceback.format_exc())
    
    def process_image(self, original_image):
        """
        处理单张图像并返回带检测结果的图像
        Args:
            original_image: OpenCV图像 (BGR格式)
        Returns:
            带检测框的图像 (BGR格式)
        """
        # 记录开始时间
        start_time = rospy.Time.now()
        
        # 预处理
        img_letterboxed, ratio, (dw, dh) = self.letterbox(
            original_image, 
            new_shape=( DEFAULT_IMG_SIZE, DEFAULT_IMG_SIZE )
        )
        
        # 转换颜色空间和格式
        img_rgb = cv2.cvtColor(img_letterboxed, cv2.COLOR_BGR2RGB)
        img_chw = img_rgb.transpose(2, 0, 1)
        img_input_np = np.expand_dims(img_chw, 0)
        img_for_inference = np.ascontiguousarray(img_input_np.astype(np.float32))
        
        # 推理
        outputs = self.rknn_lite.inference(inputs=[img_for_inference], data_format=['nchw'])
        
        if outputs is None:
            rospy.logerr("RKNN inference failed, outputs is None")
            return original_image
        
        # 后处理
        input_data_list = []
        num_anchors_per_head = 3
        
        for i in range(len(outputs)):
            # 获取当前输出的形状
            output_shape = outputs[i].shape
            
            # 根据输出维度进行不同处理
            if len(output_shape) == 3:
                # 3D输出: [grid_h, grid_w, channels]
                input_data_list.append(outputs[i])
            elif len(output_shape) == 4:
                # 4D输出: [batch, channels, grid_h, grid_w]
                # 重塑为 [grid_h, grid_w, num_anchors, attributes]
                reshaped_output = outputs[i].reshape([num_anchors_per_head, -1] + list(output_shape[-2:]))
                transposed_output = np.transpose(reshaped_output, (2, 3, 0, 1))
                input_data_list.append(transposed_output)
            else:
                rospy.logwarn(f"Unexpected output shape {output_shape} for output {i}")
                continue
    
    # 使用调整后的输出进行后处理
        boxes, detected_classes, scores = self.yolov5_post_process(input_data_list, DEFAULT_IMG_SIZE)
        
        # 创建结果图像的副本
        result_image = original_image.copy()
        
        if boxes is not None and len(boxes) > 0:
            
            # 将边界框坐标转换回原始图像空间
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] - dw) / ratio[0]  # x_min, x_max
            boxes[:, [1, 3]] = (boxes[:, [1, 3]] - dh) / ratio[1]  # y_min, y_max
            
            # 裁剪边界框到图像边界内
            boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, original_image.shape[1] - 1)
            boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, original_image.shape[0] - 1)
            print(boxes)
            # 在图像上绘制检测结果
            self.draw_detections(result_image, boxes, scores, detected_classes)
            decArray=[]
            i=0

            # 记录检测到的对象
            detected_objs = ", ".join([f"{CLASSES[cl]}:{s:.2f}" for cl, s in zip(detected_classes, scores)])
            rospy.logdebug(f"Detected {len(boxes)} objects: {detected_objs}")
            
        else:
            rospy.logdebug("No objects detected")
        for box in boxes:
            decArray.append(Detection())
            decArray[i].x_min=box[0]
            decArray[i].y_min=box[1]
            decArray[i].x_max=box[2]
            decArray[i].y_max=box[3]
            decArray[i].class_name=CLASSES[detected_classes[i]]        
            i+=1
        self.dec_pub.publish(decArray)
        # 计算处理时间
        proc_time = (rospy.Time.now() - start_time).to_sec()
        fps = 1.0 / proc_time if proc_time > 0 else 0
        
        # 在图像上添加FPS信息
        cv2.putText(result_image, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return result_image
    
    # 以下是原有的辅助函数，保持原样但添加为类方法
    def xywh2xyxy(self, x):
        y = np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2
        y[:, 1] = x[:, 1] - x[:, 3] / 2
        y[:, 2] = x[:, 0] + x[:, 2] / 2
        y[:, 3] = x[:, 1] + x[:, 3] / 2
        return y

    def process(self, input_tensor, mask, anchors, current_img_size):
        current_anchors = np.array([anchors[i] for i in mask])
        grid_h, grid_w = map(int, input_tensor.shape[0:2])

        box_confidence = input_tensor[..., 4]
        box_confidence = np.expand_dims(box_confidence, axis=-1)
        box_class_probs = input_tensor[..., 5:]
        box_xy = input_tensor[..., :2] 
        box_wh = input_tensor[..., 2:4] 

        box_xy = box_xy * 2 - 0.5

        col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
        col = col.reshape(grid_h, grid_w, 1, 1).repeat(current_anchors.shape[0], axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(current_anchors.shape[0], axis=-2)
        grid = np.concatenate((col, row), axis=-1)

        box_xy += grid
        box_xy *= int(current_img_size / grid_h)

        box_wh = pow(box_wh * 2, 2)
        box_wh = box_wh * current_anchors

        box = np.concatenate((box_xy, box_wh), axis=-1)
        return box, box_confidence, box_class_probs

    def filter_boxes(self, boxes, box_confidences, box_class_probs):
        boxes = boxes.reshape(-1, 4)
        box_confidences = box_confidences.reshape(-1)
        box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

        _box_pos = np.where(box_confidences >= OBJ_THRESH)
        boxes = boxes[_box_pos]
        box_confidences = box_confidences[_box_pos]
        box_class_probs = box_class_probs[_box_pos]

        class_max_score = np.max(box_class_probs, axis=-1)
        classes = np.argmax(box_class_probs, axis=-1)
        _class_pos = np.where(class_max_score >= OBJ_THRESH)

        boxes = boxes[_class_pos]
        classes = classes[_class_pos]
        scores = (class_max_score * box_confidences)[_class_pos]

        return boxes, classes, scores

    def nms_boxes(self, boxes, scores):
        x_min = boxes[:, 0]
        y_min = boxes[:, 1]
        x_max = boxes[:, 2]
        y_max = boxes[:, 3]

        areas = (x_max - x_min) * (y_max - y_min)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x_min[i], x_min[order[1:]])
            yy1 = np.maximum(y_min[i], y_min[order[1:]])
            xx2 = np.minimum(x_max[i], x_max[order[1:]])
            yy2 = np.minimum(y_max[i], y_max[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 1e-5)
            h1 = np.maximum(0.0, yy2 - yy1 + 1e-5)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-5)
            inds = np.where(ovr <= NMS_THRESH)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep

    def yolov5_post_process(self, input_data_list, current_img_size):
        masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
        anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
                   [59, 119], [116, 90], [156, 198], [373, 326]]

        boxes, classes, scores = [], [], []
        for input_tensor, mask in zip(input_data_list, masks):
            b, c, s = self.process(input_tensor, mask, anchors, current_img_size)
            b, c, s = self.filter_boxes(b, c, s)
            boxes.append(b)
            classes.append(c)
            scores.append(s)

        if not any(b.size > 0 for b in boxes):
            return None, None, None
            
        boxes = np.concatenate([b for b in boxes if b.size > 0])
        classes = np.concatenate([c for c in classes if c.size > 0])
        scores = np.concatenate([s for s in scores if s.size > 0])

        if boxes.shape[0] == 0:
            return None, None, None

        boxes = self.xywh2xyxy(boxes)

        nboxes, nclasses, nscores = [], [], []
        for c_idx in set(classes):
            inds = np.where(classes == c_idx)
            b_class = boxes[inds]
            s_class = scores[inds]

            if b_class.shape[0] > 0:
                keep = self.nms_boxes(b_class, s_class)
                if len(keep) > 0:
                    nboxes.append(b_class[keep])
                    nclasses.append(np.full(len(keep), c_idx, dtype=classes.dtype))
                    nscores.append(s_class[keep])
        
        if not nboxes:
            return None, None, None

        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)

        FINAL_CONF_THRESH = 0.5
        mask = scores >= FINAL_CONF_THRESH
        boxes = boxes[mask]
        classes = classes[mask]
        scores = scores[mask]
      
        if len(scores) == 0:
            return None, None, None

        # best_idx = np.argmax(scores)
        # boxes = boxes[best_idx:best_idx+1]
        # classes = classes[best_idx:best_idx+1]
        # scores = scores[best_idx:best_idx+1]

        return boxes, classes, scores

    def draw_detections(self, image, boxes, scores, class_indices):
        for box, score, cl_idx in zip(boxes, scores, class_indices):
            x_min, y_min, x_max, y_max = map(int, box)
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
            label = f'{CLASSES[cl_idx]} {score:.2f}'
            cv2.putText(image, label,
                        (x_min, y_min - 6 if y_min > 10 else y_min + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)

    def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114), auto=False, scaleFill=False, scaleup=True, stride=32):
        # Resize and pad image while meeting stride-multiple constraints
        # This is a more generic letterbox function, adapted from YOLOv5 utils.
        # The original script had a simpler version. Using the simpler one from prompt for now.
        # Using the simpler version from the original prompt to minimize changes:
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        # The original script used (0,0,0) for padding. YOLOv5 often uses (114,114,114).
        # Sticking to (0,0,0) from the original script.
        padding_color = (0,0,0) # color for padding, was (0,0,0) in original script

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=padding_color)
        return im, ratio, (dw, dh)

    
    def shutdown_hook(self):
        """ROS关闭时的清理工作"""
        rospy.loginfo("Shutting down YOLOv5 RKNN detector")
        self.rknn_lite.release()
        rospy.loginfo("RKNN resources released.")

if __name__ == '__main__':
    try:

        detector_node = YOLOv5RKNNNode()
        rospy.on_shutdown(detector_node.shutdown_hook)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLOv5 RKNN detector node terminated.")