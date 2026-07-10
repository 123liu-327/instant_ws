# 交通信号灯检测逻辑

本包订阅相机图像，用 RK3588 NPU 上的 YOLOv5/RKNN 模型识别交通灯，并在识别结果稳定后向车辆发布通行指令。

## 类别编号：不可改变的模型约定

模型来自 `traffic_light_yolo_dataset/traffic_light.yaml`，类别编号为：

| 类别 ID | 模型标签 | 含义 | 车辆指令 |
| --- | --- | --- | --- |
| 0 | `red` | 红灯 | `Stop` |
| 1 | `straight` | 绿灯直行 | `Middle` |
| 2 | `right` | 绿灯右转 | `Right` |
| 3 | `left` | 绿灯左转 | `Left` |

因此，`rknn_backend.py` 中的 `CLASS_NAMES` 和 launch 文件中的 `class_names` 必须都是 `[red, straight, right, left]`。旧配置曾写成 `[red, straight, left, right]`，会把类别 ID 2 和 3 的结果解释反，表现为左、右转指令颠倒。

## 数据流与决策流程

```text
/ucar_camera/image_raw (sensor_msgs/Image, BGR)
  -> 仅在 /traffic_light/enable=true 后处理
  -> 等比例缩放和边缘填充到 640 x 640（letterbox）
  -> BGR 转 RGB，增加 batch 维，形成 [1, 640, 640, 3] 的 NHWC uint8 输入
  -> RKNNLite / RK3588 NPU 推理
  -> 三个 YOLOv5 输出头解码（80x80、40x40、20x20）
  -> 置信度筛选、按类别 NMS、坐标映射回原图
  -> 有 red 时优先选 red；否则选择置信度最高目标
  -> 时序确认
  -> /traffic_light/state 和 /follow_begin
```

### 单帧后处理

1. 将三个输出头统一为 `H x W x 3 x (5 + 4)`；每个锚框有坐标、目标置信度和四类概率。
2. 单框分数为 `objectness x 最大类别概率`，默认低于 `0.55` 的候选框丢弃。
3. 用九个 YOLOv5 anchors 解码边界框；同类别、IoU 大于默认 `0.45` 的重叠框由 NMS 去除。
4. 若同一帧既有红灯也有方向灯，红灯优先，保证安全停车。

### 时序确认与车辆动作

节点默认不启动检测。收到一次以下消息后才进入检测状态，并立即向 `/follow_begin` 发布 `Stop`：

```bash
rostopic pub -1 /traffic_light/enable std_msgs/Bool "data: true"
```

随后采用滑动窗口确认，默认参数如下：

| 项目 | 默认值 | 行为 |
| --- | ---: | --- |
| `vote_window` | 7 帧 | 保存最近结果 |
| `red_confirm_frames` | 2 帧 | 连续两帧红灯即稳定为 `red` |
| `direction_min_votes` | 5 帧 | 方向灯至少出现五次 |
| `direction_min_avg_confidence` | 0.65 | 方向灯这五帧的平均置信度至少为 0.65 |

- 稳定结果为 `red`：发布 `Stop`。
- 稳定结果为 `straight`、`right` 或 `left`：分别发布 `Middle`、`Right`、`Left`，随后自动解除检测，避免同一信号灯重复下发指令。
- 没有稳定结果：`/traffic_light/state` 发布 `unknown`，车辆保持此前在进入检测时下发的 `Stop`。

## 调试与安全测试

先关闭实际车辆指令：

```bash
roslaunch traffic_light_vision traffic_light_detector.launch publish_commands:=false
```

查看最终状态和带框图像：

```bash
rostopic echo /traffic_light/state
rqt_image_view /traffic_light/debug_image
```

用左右转样图各测试一次：类别 ID 2 的框和状态必须显示 `right`，类别 ID 3 必须显示 `left`。若更换了 `.rknn` 模型，先确认它仍由同一份 YAML（`0 red, 1 straight, 2 right, 3 left`）训练和转换；否则必须按新模型的实际类别编号同步更新三处配置。
