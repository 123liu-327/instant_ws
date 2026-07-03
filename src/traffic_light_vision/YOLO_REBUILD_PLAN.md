# 红绿灯视觉包 YOLOv5s-RKNN 重制与迁移

## 当前状态

生产视觉路径已经从HSV/轮廓法改为四状态YOLOv5s-RKNN：

```text
/ucar_camera/image_raw
  -> YOLOv5s FP16 RKNN (RK3588)
  -> 7帧置信度投票
  -> /traffic_light/state
  -> Stop / Middle / Left / Right
```

固定类别顺序为 `red, straight, left, right`。运行代码、launch、数据审计工具和训练配置已经落入本包。训练数据和最终RKNN模型不在当前工作区，因此模型训练、转换一致性验证和实车验收仍需执行。

## 新老方案对比

| 项目 | 旧工程 | 第一版视觉包 | 当前重制方案 |
|---|---|---|---|
| 识别 | YOLO红/绿两类 | HSV和轮廓 | YOLO四状态直检 |
| 决策 | 统计1秒数量 | 连续同类帧 | 置信度加权投票 |
| 部署 | 独立RKNN包 | CPU OpenCV | 视觉包内RKNN后端 |
| 安全 | 平票可能隐式走右侧 | 无任务使能 | 使能即停车，unknown不选路 |

旧方案仅复用了RKNN生命周期、三检测头解码、NMS和NPU多核初始化。旧物品类别、`/detect_results`消息、固定亮度减半、图像镜像、强制拉伸和硬编码模型路径均未迁移。

## 必须按顺序执行

### 1. 审计已有四类数据集

数据集目录必须符合：

```text
dataset/
  images/{train,val,test}/
  labels/{train,val,test}/
```

执行：

```bash
python3 tools/audit_yolo_dataset.py /实际数据集路径 --report audit.json
```

报告必须为 `ok: true`。相邻视频帧不可跨训练/验证/测试集，类别ID必须与本包一致。

### 2. 训练PT基线

使用YOLOv5 v7.0和COCO预训练的`yolov5s.pt`：

```bash
python3 train.py \
  --weights yolov5s.pt \
  --data /本包路径/training/traffic_light.yaml \
  --hyp /本包路径/training/hyp.traffic_light.yaml \
  --img 640 --batch 16 --epochs 150 --patience 30 --seed 42 \
  --noautoanchor
```

当前运行后端固定使用YOLOv5默认九组anchors，因此基线训练必须带`--noautoanchor`。不要启用水平或垂直翻转。先解决数据问题，再调整时序阈值。

### 3. 验证并导出三检测头ONNX

测试集要求`mAP@0.5 >= 0.95`且每类召回率不低于0.95。导出模型必须是静态`1x3x640x640`输入，并暴露三个原始检测头。四类模型的每层输出通道固定为：

```text
3 anchors * (x, y, w, h, objectness + 4 classes) = 27
```

普通的单一`[1, 25200, 9]`拼接输出不符合当前RKNN后端契约。

### 4. 转换和验证RKNN

使用RKNN Toolkit2 1.6.0，目标平台`rk3588`，首版不量化，生成FP16模型。预处理必须使用letterbox填充114和BGR转RGB，不得强制拉伸、镜像或固定降低亮度。

```bash
python3 tools/convert_onnx_to_rknn.py best_three_heads.onnx \
  models/traffic_light_yolov5s_fp16.rknn
```

在同一批100张图片上比较PT、ONNX、RKNN：类别一致率至少99%，平均框IoU至少0.90，平均置信度差不超过0.05。通过后复制为：

```text
models/traffic_light_yolov5s_fp16.rknn
```

### 5. 离线和上车验证

```bash
roslaunch traffic_light_vision traffic_light_detector.launch
rostopic pub -1 /traffic_light/enable std_msgs/Bool "data: true"
```

依次完成图片/rosbag回放、架空轮、低速实车测试。红灯或unknown不得产生方向命令；每次使能只允许产生一次方向命令。RK3588端到端速度应不低于15 FPS。

## ROS接口与状态机

- 输入图像：`/ucar_camera/image_raw`
- 任务使能：`/traffic_light/enable`
- 稳定状态：`/traffic_light/state`
- 调试图像：`/traffic_light/debug_image`
- 巡线指令：`/follow_begin`

收到`enable=true`后节点立即发布`Stop`并清空历史。红灯保持使能和停车；`straight/left/right`达到投票条件后分别发布`Middle/Left/Right`，随后自动解除使能。模型缺失、NPU初始化失败或推理异常均直接报错退出，不回退HSV。

## 后续优化

只有FP16端到端低于15 FPS时才评估INT8。量化模型必须重新完成100张一致性验证，未达到相同验收门槛不得替换FP16生产版本。误检和漏检样本按模型版本归档并加入下一轮训练。
