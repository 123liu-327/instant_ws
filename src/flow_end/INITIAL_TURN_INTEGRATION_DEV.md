# 起步预转角速度积分算法开发说明

## 1. 修改目标

`follow_test` 在起步面对左/中/右岔路时，需要先让车身朝目标方向偏转，然后再进入正常巡线。

之前的实现使用：

```cpp
current_yaw - initial_turn_start_yaw
```

也就是用 IMU 姿态四元数解出的 yaw 差判断转过多少度。实际测试中可能出现 yaw 跳变、初始 yaw 不稳定、坐标轴方向和车体转向不一致等问题，导致日志显示已经转过很多角度，但小车物理上并没有转够。

因此本次修改改为原 `follow_line.cpp` 中环岛/转角状态使用过的算法：**对 IMU 角速度 `curent_wz` 做时间积分**。

## 2. 原工程参考逻辑

原 `follow_line.cpp` 的 `Round_step` 转角中使用了类似代码：

```cpp
rotated_angle = rotated_angle
    + (ros::Time::now() - Round_timer_dida).toSec() * curent_wz * 57.3;
```

含义是：

- `curent_wz` 来自 IMU 的 `angular_velocity.z`，单位是 `rad/s`
- `dt` 是两次循环之间的时间差，单位是秒
- `57.3` 约等于 `180 / pi`，用于把弧度转换成角度
- 累加后得到已经转过的角度，单位是度

这比直接用 yaw 差更接近原包已有的运动控制习惯。

## 3. 当前 follow_test 的状态流程

`follow_test` 的起步预转状态如下：

```text
IDLE
  |
  | 收到 /follow_begin: Left 或 Right
  v
ALIGNING_LEFT / ALIGNING_RIGHT
  |
  | 积分角度达到 initial_turn_angle_deg
  | 或目标线点数达到 initial_turn_rpts_threshold
  v
ALIGN_PAUSE
  |
  | 停顿 initial_turn_pause_sec 秒
  v
FOLLOWING
```

如果收到的是 `Middle`，不会进入预转角，直接进入 `FOLLOWING`。

## 4. 关键变量

```cpp
double initial_turn_integrated_angle_deg = 0.0;
ros::Time initial_turn_last_time;
bool initial_turn_has_last_time = false;
```

含义：

- `initial_turn_integrated_angle_deg`：当前预转阶段累计转过的角度
- `initial_turn_last_time`：上一次积分的时间戳
- `initial_turn_has_last_time`：是否已经初始化积分时间

每次收到 Left/Right 启动指令时，这些变量都会重新初始化，避免沿用上一次转角结果。

## 5. 积分计算

在 `handleInitialTurn()` 中，每一轮循环计算：

```cpp
double dt = (now - initial_turn_last_time).toSec();
initial_turn_last_time = now;
initial_turn_integrated_angle_deg += curent_wz * dt * 57.3;
```

程序使用：

```cpp
std::abs(initial_turn_integrated_angle_deg) >= initial_turn_angle_deg
```

判断是否转够角度。这里取绝对值，是因为不同 IMU 或底盘坐标方向下，左转/右转的 `wz` 正负可能相反，但只要累计角度大小达到目标，就认为预转完成。

## 6. 防异常时间差

代码中限制了异常 `dt`：

```cpp
if (dt < 0.0 || dt > 0.2) {
    dt = 0.0;
}
```

这是为了避免程序刚启动、系统时间跳变、回调卡顿时，一次性积分出过大的角度。

## 7. 调试日志

现在预转过程中的日志会输出：

```text
initial turn path=left integrated_angle=12.34 wz=0.351 target=30.00 selected_rpts=120 threshold=250
```

字段含义：

- `path`：当前目标路径，left 或 right
- `integrated_angle`：角速度积分得到的已转角度
- `wz`：IMU 当前 z 轴角速度
- `target`：目标预转角度
- `selected_rpts`：当前目标线点数
- `threshold`：目标线点数提前结束阈值

如果 `wz` 长时间接近 0，但 `/cmd_vel` 中有 `angular.z`，说明底盘没有真正转动或 IMU 没有反馈角速度。

## 8. 和常规巡线的关系

起步预转只在 `ALIGNING_LEFT` 或 `ALIGNING_RIGHT` 状态下接管 `/cmd_vel`：

```cpp
msg.linear.x = 0.0;
msg.angular.z = left ? +initial_turn_angular_speed : -initial_turn_angular_speed;
```

进入 `FOLLOWING` 后，控制权回到常规视觉巡线：

```cpp
error = -atan2f(dx, dy);
v = base_speed - abs(error) * base_speed;
msg.linear.x = v;
msg.angular.z = -error;
```

也就是说，预转角只是解决起步岔路朝向问题，不改变后续图像识别、左线/右线/中线选择和停车逻辑。
