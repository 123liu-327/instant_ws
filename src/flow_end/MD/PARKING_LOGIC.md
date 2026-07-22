# follow_test 当前停车逻辑说明

本文档对应 `src/follow_line_test.cpp` 的当前停车实现。当前版本已经恢复为旧版风格：检测到有效 L 角点后立即进入停车接管，不再忽略第一个角点，也不再等待第二个角点。

## 总体流程

1. `detectCorners()` 在左右边线角度数组中寻找 L 型角点。
2. `handleParkingCorner()` 判断 `Lpt1 / Right_L` 和 `Lpt0 / Left_L` 是否满足停车条件。
3. 默认 `parking_allow_either_l=true`，所以左右任意一侧 L 点都可以触发停车；优先尝试 `Right_L`，再尝试 `Left_L`。
4. 一旦检测到有效停车 L 点，直接打印 `[PARKING] Corner detected` 并进入停车接管。
5. 停车接管期间直接发布 `cmd_vel`，不再走普通巡线控制器。
6. 到达计划停车距离后发布零速度，并向 `end_topic` 发布 `STOP`。

## L 点检测

L 点检测发生在 `detectCorners()` 中：

- `Lpt0`：左侧线上的 L 型角点。
- `Lpt1`：右侧线上的 L 型角点。

当前基础检测条件：

```cpp
65deg < conf < 115deg
i < 0.8 / sample_dist
```

含义：

- `conf` 是当前点相对前后邻域的角度突变量。
- 65 到 115 度之间认为接近直角 L 点。
- `i < 0.8 / sample_dist` 表示只检测车前约 `0.8m` 范围内的角点。
- 日志里的 `id` 不是直接米制距离，但近似距离可以理解为 `id * sample_dist`。

## 停车触发条件

默认行为：

- `parking_allow_either_l=true`：不管当前是左巡线还是右巡线，`Lpt1` 或 `Lpt0` 都可以触发停车。
- `parking_allow_either_l=false`：恢复按路径侧限制，`path=right` 只看 `Lpt1`，`path=left` 只看 `Lpt0`。

右侧 L 点需要满足：

```cpp
Lpt1_found
Lpt1_rpts1s_id >= 1
Lpt1_rpts1s_id < rptsc1_num
rptsc1[im1][1] - rptsc1[Lid][1] > 15
rptsc1[ip1][0] - rptsc1[Lid][0] < -15
rptsc1[Lid][1] > RESULT_ROW - 80
```

左侧 L 点需要满足：

```cpp
Lpt0_found
Lpt0_rpts0s_id >= 1
Lpt0_rpts0s_id < rptsc0_num
rptsc0[im0][1] - rptsc0[Lid][1] > 15
rptsc0[ip0][0] - rptsc0[Lid][0] > 15
rptsc0[Lid][1] > RESULT_ROW - 80
```

这些条件的含义：

- `id >= 1`：过滤最靠近采样起点的噪声。
- `id < rptsc*_num`：保证角点索引有效。
- 前后点的 `x/y` 差值：判断角点形状是否真的像停车 L 角。
- `y > RESULT_ROW - 80`：要求角点接近图像底部，说明车已经靠近停车区。

## 停车距离和运动

进入停车接管后，代码用角点图像坐标估算目标距离：

```cpp
cx = RESULT_COL / 2
cy = RESULT_ROW + 10
target_dis = -(corner_dot[1] - cy) / pixel_per_meter
target_dis_x = -(corner_dot[0] - cx) / pixel_per_meter
```

当前总停车距离：

```cpp
parking_total_dist = abs(target_dis) + parking_extra_dist
```

默认参数：

- `parking_extra_dist = 0.215m`
- `parking_forward_speed = 0.15m/s`

停车接管时：

```cpp
linear.x = parking_forward_speed
linear.y = 0.0
angular.z = 0.0
```

当横向偏差较大时：

```cpp
if (abs(target_dis_x) >= 0.08) {
    linear.y = target_dis_x > 0 ? 0.1 : -0.1;
}
```

也就是说，停车期间固定低速前进，横向偏差超过 `0.08m` 时用 `±0.10m/s` 做微调。

## Launch 参数

当前和停车直接相关的参数：

```xml
<arg name="parking_enabled" default="true" />
<arg name="parking_allow_either_l" default="true" />
<arg name="parking_extra_dist" default="0.215" />
<arg name="parking_forward_speed" default="0.15" />
```

含义：

- `parking_enabled`：停车总开关，`false` 时完全不进入 L 点停车逻辑。
- `parking_allow_either_l`：是否允许左右任意 L 点触发停车。
- `parking_extra_dist`：走到视觉估计角点距离后，额外继续前进的距离。
- `parking_forward_speed`：停车接管阶段的前进速度。

示例：

```bash
roslaunch flow_end follow_test.launch parking_enabled:=true parking_extra_dist:=0.10 parking_forward_speed:=0.10
```

## 常见日志含义

```text
[PARKING] CornerDetect
```

周期性检测日志。表示停车逻辑正在运行，但当前帧还没有触发最终停车。

```text
L0=1(id=...) / L1=1(id=...)
```

表示视觉检测到了左/右 L 点，但还不一定满足停车触发条件。

```text
[PARKING] Corner detected
```

最终停车角点确认，开始停车接管。当前版本检测到有效 L 点后会直接进入这个阶段。

```text
[PARKING_PROGRESS] Approaching
```

停车接管中，正在按计划距离前进。

```text
[PARKING] Parking finished!
```

停车距离完成，发布零速度和 `STOP`。

当前版本不应再出现：

```text
[PARKING] First square corner ignored
[PARKING] Waiting second square corner
```

如果还能看到这两类日志，说明运行的不是当前编译出来的节点。

## 调参建议

1. 如果只有 `L0=0/L1=0`，说明视觉没有检测到 L 点。
2. 如果 `L0=1` 或 `L1=1` 但没有 `Corner detected`，说明 L 点没有通过底部/形状过滤。
3. 如果 `Corner detected` 后停车太早，减小 `parking_extra_dist`。
4. 如果停车太晚，增大 `parking_extra_dist`。
5. 如果停车过程太快或打滑，减小 `parking_forward_speed`。
