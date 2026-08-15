# follow_test 停车 S 弯调参指南

本文说明如何调整 `flow_end/follow_test` 检测到停车 L 角后的 S 弯效果。
所有示例都通过命令行临时覆盖参数，不需要修改 `follow_test.launch`。

## 1. 当前动作

默认停车模式为：

```text
parking_motion_mode=s_curve
```

检测到 L 角后，控制器根据目标的纵向距离 `x` 和横向偏差 `y` 规划两段对称圆弧：

1. 前半程朝目标侧转入。
2. 后半程反向转向，将车头回正。
3. 到达规划距离后，如果航向误差仍超过容差，则停止前进并短暂原地回正。

规划关系为：

```text
x = abs(视觉纵向距离) + parking_extra_dist
y = parking_lateral_cmd_sign × 视觉横向偏差
峰值航向角 theta = 2 × atan2(abs(y), x)
圆弧半径 R = (x² + y²) / (4 × abs(y))
```

因此：

- `x` 越小，同样横向偏差下转弯越急。
- `abs(y)` 越大，转弯越明显。
- `parking_max_angular_speed` 是执行限幅，不直接改变理论圆弧半径。
- 如果角速度不够，程序会自动降低前进速度，以保持规划曲率。

## 2. 参数作用

| 参数 | 当前默认值 | 主要作用 | 调大后的表现 |
|---|---:|---|---|
| `parking_extra_dist` | `0.315 m` | 增加停车纵向距离，同时影响 S 弯半径 | 路径更长、更缓，最终停车位置更靠前 |
| `parking_forward_speed` | `0.15 m/s` | 期望前进速度 | 动作更快；角速度受限时可能不会达到该速度 |
| `parking_max_angular_speed` | `0.35 rad/s` | 限制停车最大角速度 | 允许更快地执行急弯，减少自动降速 |
| `parking_yaw_kp` | `1.5` | 航向参考跟踪强度 | 跟转和回正更积极，过大可能摆动或突变 |
| `parking_yaw_tolerance_deg` | `3.0°` | 最终航向完成容差 | 更容易结束，但停车朝向误差可能更大 |
| `parking_lateral_deadband` | `0.03 m` | 横向偏差小于此值时直行停车 | 更多小偏差会被忽略，不再规划 S 弯 |
| `parking_lateral_cmd_sign` | `1.0` | 适配底盘和图像的左右方向 | 只应在实际转向方向完全相反时改成 `-1.0` |
| `parking_timeout` | `6.0 s` | 单次停车动作总超时 | 给低速急弯和末端回正更多时间 |
| `parking_odom_timeout` | `0.5 s` | 里程计最大允许断流时间 | 只影响故障判定，不改变转弯效果 |

`parking_lateral_speed` 只在兼容模式 `parking_motion_mode=lateral` 中使用，S 弯模式下不生效。

## 3. 不修改 launch 的调参方法

### 3.1 启动时临时覆盖

下面的参数只对本次启动有效：

```bash
roslaunch flow_end follow_test.launch \
  parking_motion_mode:=s_curve \
  parking_forward_speed:=0.12 \
  parking_max_angular_speed:=0.30 \
  parking_yaw_kp:=1.3
```

关闭节点后，参数自动恢复为 launch 中的默认值。

### 3.2 节点运行中修改下一次停车参数

`follow_test` 会在收到 `/follow_begin` 指令时重新读取私有参数。因此应先停止本轮，再设置参数并重新开始：

```bash
rostopic pub -1 /follow_begin std_msgs/String "data: 'Stop'"

rosparam set /follow_test/parking_forward_speed 0.12
rosparam set /follow_test/parking_max_angular_speed 0.30
rosparam set /follow_test/parking_yaw_kp 1.3

rostopic pub -1 /follow_begin std_msgs/String "data: 'Right'"
```

运行中已经进入 `PARKING` 后再设置参数，不会改变当前这一次停车动作，只会在下一次开始指令时生效。

查看当前生效参数：

```bash
rosparam get /follow_test/parking_motion_mode
rosparam get /follow_test/parking_forward_speed
rosparam get /follow_test/parking_max_angular_speed
rosparam get /follow_test/parking_yaw_kp
rosparam get /follow_test/parking_yaw_tolerance_deg
```

## 4. 推荐调参顺序

每次只改一个参数，并保存 `[PARKING_PLAN]`、`[PARKING_PROGRESS]` 和最终停车日志。

### 第一步：确认左右方向

先架空车轮或把速度降到 `0.08～0.10 m/s`。如果目标在左侧而车辆先向右转，或目标在右侧而车辆先向左转，只切换方向符号：

```bash
rosparam set /follow_test/parking_lateral_cmd_sign -1.0
```

方向正确后不要再用这个参数调转弯幅度。

### 第二步：调整转向跟随

保持停车位置参数不变，先观察日志中的：

```text
yaw=(ref=...,actual=...,error=...)deg
```

- `actual` 长期追不上 `ref`：将 `parking_yaw_kp` 每次增加 `0.1～0.2`。
- 第二段回正时左右摆动：将 `parking_yaw_kp` 每次降低 `0.1～0.2`。
- 角速度经常达到上限且车辆自动变得很慢：可小幅提高 `parking_max_angular_speed`。

初调建议范围：

```text
parking_yaw_kp:             1.0 ～ 1.8
parking_max_angular_speed:  0.20 ～ 0.35 rad/s
```

未完成架空轮和低速验证前，不建议超过 `0.50 rad/s`。

### 第三步：调整曲线形状和停车前后位置

航向已经能稳定跟随后，再调整 `parking_extra_dist`：

- 转弯太急、车身摆动明显：增加 `0.02～0.05 m`。
- 希望 S 弯更明显、更紧凑：减少 `0.02～0.05 m`，但停车位置也会随之提前。
- 停车整体过于靠前：减少该值。
- 停车整体过于靠后：增加该值。

这个参数同时影响停车纵向位置和曲线半径，调整后必须重新检查最终位置。

### 第四步：调整速度

几何和航向稳定后再提高 `parking_forward_speed`，建议每次增加 `0.02 m/s`。

如果 `[PARKING_PLAN]` 中的 `cmd_v` 明显小于配置的 `parking_forward_speed`，说明当前曲率需要的角速度超过上限，控制器正在主动降速。此时继续提高前进速度不会让车实际更快，应先判断是否可以安全提高 `parking_max_angular_speed`，或者增加 `parking_extra_dist` 让曲线变缓。

## 5. 常见现象处理

| 现象 | 优先检查 | 建议 |
|---|---|---|
| 一开始向错误方向转 | `parking_lateral_cmd_sign` | 在 `1.0` 和 `-1.0` 之间切换 |
| 实际航向明显追不上参考航向 | `parking_yaw_kp`、角速度是否顶到上限 | 先小幅增加 `yaw_kp`；顶限时再考虑增加最大角速度 |
| 第二段回正过冲、左右摆 | `parking_yaw_kp` | 每次降低 `0.1～0.2` |
| 转弯动作太猛 | `parking_yaw_kp`、`parking_max_angular_speed` | 先降低 `yaw_kp`，必要时再降低角速度上限 |
| 转弯时速度异常慢 | `[PARKING_PLAN] cmd_v` | 角速度限幅正在降速；提高角速度上限或增加额外距离 |
| 最后经常原地回正 | 最终 `yaw_error` | 改善途中航向跟踪，不要优先放宽完成容差 |
| 小偏差也频繁 S 弯 | `parking_lateral_deadband` | 每次增加 `0.01 m`，同时检查停车横向误差 |
| 经常报 `PARKING_ABORTED_TIMEOUT` | 规划长度、实际速度、回正耗时 | 先排查里程计和速度，再适当增加总超时 |
| 报 `PARKING_ABORTED_ODOM` | `/odom` 频率和时间戳 | 不要通过放大超时掩盖真实断流 |

## 6. 日志判读

停车开始时：

```text
[PARKING_PLAN] mode=s_curve | target=(x,y)m | curved=... | radius=... |
peak_yaw=... | path=... | cmd_v=... | ff_wz=...
```

重点关注：

- `target`：本次视觉估计的纵向和横向目标。
- `radius`：越小代表理论转弯越急。
- `peak_yaw`：S 弯中点的最大航向偏角。
- `cmd_v`：限幅后实际计划速度。
- `ff_wz`：圆弧所需的基础角速度。

停车过程中：

```text
[PARKING_PROGRESS] ... yaw=(ref=...,actual=...,error=...)deg |
cmd=(linear.x,linear.y,angular.z)
```

S 弯模式正常情况下 `linear.y` 应始终为 `0`；前半程与后半程的 `angular.z` 符号应相反，最终 `yaw error` 应进入配置的角度容差。

## 7. 建议的三组临时配置

保守验证：

```bash
roslaunch flow_end follow_test.launch \
  parking_forward_speed:=0.10 \
  parking_max_angular_speed:=0.25 \
  parking_yaw_kp:=1.2
```

当前基准：

```bash
roslaunch flow_end follow_test.launch \
  parking_forward_speed:=0.15 \
  parking_max_angular_speed:=0.35 \
  parking_yaw_kp:=1.5
```

偏稳、减小回正摆动：

```bash
roslaunch flow_end follow_test.launch \
  parking_forward_speed:=0.12 \
  parking_max_angular_speed:=0.30 \
  parking_yaw_kp:=1.2 \
  parking_extra_dist:=0.345
```

正式实车验收时，应同时记录最终横向偏差和车头航向；目标分别是不超过 `0.03 m` 和 `3°`。
