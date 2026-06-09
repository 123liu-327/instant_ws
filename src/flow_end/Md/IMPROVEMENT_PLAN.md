# 巡线包深度分析 — 改进方案文档

> 基于对 `flow_end`、`line_follower`、`ucar_nav/ucar_followline.py` 三个巡线包的逐文件阅读。
> 生成日期：2026-06-06

---

## 目录

1. [P0 — Bug 修复](#1-p0--bug-修复)
2. [P1 — 架构与代码组织](#2-p1--架构与代码组织)
3. [P2 — 控制算法改进](#3-p2--控制算法改进)
4. [P2 — 图像处理改进](#4-p2--图像处理改进)
5. [P2 — 安全与鲁棒性](#5-p2--安全与鲁棒性)
6. [P3 — 代码质量](#6-p3--代码质量)
7. [P3 — 参数管理](#7-p3--参数管理)
8. [总结优先级排序](#8-总结优先级排序)

---

## 1. P0 — Bug 修复

### 1.1 `ucar_followline.py:909` — 停车距离计算错误

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py#L909)

**当前代码**：
```python
white_line_center_y = (white_line_start_y + white_line_start_y) // 2
```

**问题**：`white_line_start_y` 被加了两次，应该是 `white_line_end_y`。这导致白线中心点计算永远等于 `white_line_start_y`，停车判断距离完全失效。

**修复**：
```python
white_line_center_y = (white_line_start_y + white_line_end_y) // 2
```

---

### 1.2 `line_follower_node.cpp:565-619` — 微分项滤波不一致

**文件**：[`line_follower/src/line_follower_node.cpp`](../line_follower/src/line_follower_node.cpp#L565)

**当前代码**：
```cpp
static int lastError = 0;
// ...
int derivative = filteredError - lastError;  // line 600
// ...
lastError = filteredError;                    // line 619
```

**问题**：`lastError` 存的是滤波后的误差，但微分项的标准做法是跟踪原始误差。在滤波系数较小时会导致 D 项被严重衰减，转弯响应迟钝。

**修复**：
```cpp
static int lastRawError = 0;
// ...
int derivative = error - lastRawError;  // 使用原始误差
lastRawError = error;
```

---

### 1.3 `flow_end/src/process_image.cpp:116-118` — ROS_INFO 打印错误变量

**文件**：[`src/process_image.cpp`](src/process_image.cpp#L116-L118)

**当前代码**：
```cpp
ROS_INFO("rptsc0_num: %d\n", rpts1_num);   // 打印的是 rpts1_num 而非 rptsc0_num
ROS_INFO("rpts1_num: %d\n", rpts1_num);    // 同上
```

**问题**：变量名和实际打印内容不匹配，调试时严重误导。

**修复**：
```cpp
ROS_INFO("rptsc0_num: %d\n", rptsc0_num);
ROS_INFO("rptsc1_num: %d\n", rptsc1_num);
```

---

### 1.4 `flow_end` 角度转换使用魔数 `57.3`

**文件**：[`src/follow_line_test.cpp`](src/follow_line_test.cpp#L948)

**当前代码**：
```cpp
initial_turn_integrated_angle_deg += curent_wz * dt * 57.3;
```

**问题**：`57.3 ≈ 180/π`，是弧度的近似值。虽然在实际使用中问题不大，但使用标准常量更清晰、更精确。

**修复**：
```cpp
initial_turn_integrated_angle_deg += curent_wz * dt * 180.0 / M_PI;
```

---

### 1.5 `flow_end` PID 角度计算方向符号问题

**文件**：[`src/follow_line_test.cpp`](src/follow_line_test.cpp#L948)

**当前代码**：
```cpp
initial_turn_integrated_angle_deg += curent_wz * dt * 57.3;
```

**问题**：`curent_wz` 是带符号的角速度（正=左转，负=右转）。当 `motion_state == ALIGNING_RIGHT` 时 `curent_wz` 为负值，`initial_turn_integrated_angle_deg` 会累积负值。而后续比较 `std::abs(initial_turn_integrated_angle_deg) >= initial_turn_angle_deg` 使用的是绝对值，这本来是可行的。但在 Y 分支旋转 ([line 610](src/flow_end/src/follow_line_test.cpp#L610)) 中也用了 `std::abs(curent_wz)`，这意味着旋转方向由 `y_turn_angular_speed` 的符号显式决定，而不是依赖 IMU 的积分——这两种策略不一致，可能导致角度超调。

**建议**：统一使用 `std::abs(curent_wz)` 做积分，方向由状态机显式控制。

---

## 2. P1 — 架构与代码组织

### 2.1 `ucar_followline.py` 是 2753 行的巨型单体文件

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py)（2753 行）

**问题**：该文件包含所有功能模块：
- 图像预处理 (`PreProcess`)
- 车道线检测 (`Lanedetection`)
- 纯跟踪控制 (`pure_pursuit_control`)
- 速度计算 (`calculate_speed`)
- 状态机 (`LaneDetectionNode`)
- 底盘控制 (`ChassisController`)
- 环岛逻辑、避障接口

**建议拆分**：
```
ucar_nav/scripts/
├── lane_detection/
│   ├── __init__.py
│   ├── preprocess.py          # PreProcess 类
│   ├── lane_detector.py       # Lanedetection 类
│   ├── pure_pursuit.py        # 纯跟踪算法
│   └── corner_detect.py       # 角点检测
├── control/
│   ├── __init__.py
│   ├── speed_controller.py    # calculate_speed
│   ├── chassis_controller.py  # ChassisController
│   └── state_machine.py       # 状态机
├── utils/
│   ├── __init__.py
│   ├── lane_valid_check.py    # _check_lane_valid
│   └── roundabout_check.py    # _round_insection
└── lane_follow_node.py        # 仅 ROS 入口和调度 (~300行)
```

---

### 2.2 三套独立巡线实现并存

| 包 | 语言 | 状态 | 图像算法 | 当前用途 |
|---|---|---|---|---|
| `flow_end/follow_test` | C++ | 活跃开发 | 自适应阈值 + 迷宫法 | **主力** |
| `line_follower` | C++ | 遗留 | 简单颜色阈值 | 废弃 |
| `ucar_nav/ucar_followline.py` | Python | 国赛版本 | Canny + 迷宫法 | 国赛 |

**问题**：
- 三套代码没有共享任何算法模块
- `line_follower` 的算法质量远低于另外两个
- `ucar_followline.py` 和 `flow_end` 的迷宫法巡线逻辑完全相同但各自实现

**建议**：
1. **短期**：归档 `line_follower`，从 CMakeLists.txt 中移除，目录保留只读
2. **中期**：将 `ucar_followline.py` 中的环岛状态机和避障集成经验移植到 `flow_end` C++ 版本
3. **长期**：只保留一个主力巡线包

---

### 2.3 `flow_end` 中的全局变量泛滥

**文件**：[`src/follow_line_test.cpp`](src/follow_line_test.cpp#L28-L134)

**当前状态**：约 **60 个全局变量**定义在文件作用域：

```cpp
// 代表性的全局变量（共60+个）：
double change_un_Mat[3][3] = {...};
int point_map[RESULT_ROW][RESULT_COL][2];
uint8_t *PerImg_ip[RESULT_ROW][RESULT_COL];
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num = 0, rpts1_num = 0;
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num = 0, rpts1s_num = 0;
// ... 还有约 50 个
```

这些变量通过头文件 `follow.h` 中的 `extern` 声明在多个 `.cpp` 文件间共享。

**问题**：
- 无法实例化多个巡线器（如同时巡左线和右线）
- 单元测试完全不可行
- 修改一个函数可能影响全局状态，副作用不可控
- 多线程不安全（虽然目前是单线程，但未来如需多线程处理会出问题）

**建议修改方案**：

```cpp
// 新建 include/flow_end/lane_follow_context.h
namespace flow_end {

struct ImageProcessingState {
    image_t img_raw;
    float begin_x = 25;
    float begin_y = 400;
    int ipts0[POINTS_MAX_LEN][2];
    int ipts1[POINTS_MAX_LEN][2];
    int ipts0_num = 0, ipts1_num = 0;
    float thres = 30;
    float block_size = 7;
    float clip_value = 1;
    // ...
};

struct LanePoints {
    float rpts0[POINTS_MAX_LEN][2];
    float rpts1[POINTS_MAX_LEN][2];
    int rpts0_num = 0, rpts1_num = 0;
    float rpts0s[POINTS_MAX_LEN][2];
    float rpts1s[POINTS_MAX_LEN][2];
    int rpts0s_num = 0, rpts1s_num = 0;
    // ...
};

struct LaneFollowContext {
    ImageProcessingState image_state;
    LanePoints lane_points;
    // ... 所有状态封装在此
};

}  // namespace flow_end
```

然后逐步将所有函数改为接收 `LaneFollowContext&` 引用参数。

---

### 2.4 `flow_end` CMakeLists.txt 中的死代码

**文件**：[`CMakeLists.txt`](CMakeLists.txt)

**当前状态**：同时编译 `follow_end` 和 `follow_test` 两个可执行文件：

```cmake
add_executable(follow_end ${SOURCES})         # 旧版，包含环岛/激光雷达流程
add_executable(follow_test ...)               # 新版，当前主力
```

**问题**：
- `follow_end` 已不再使用，其源文件（`follow.cpp`、`Callback.cpp`、`follow_line.cpp`、`Laser_linear.cpp`）包含已废弃的环岛/激光雷达流程
- 每次 `catkin_make` 都浪费编译时间

**建议**：
1. 移除 `add_executable(follow_end ...)` 及相关源文件列表
2. 将 `follow.cpp`、`Callback.cpp`、`follow_line.cpp`、`Laser_linear.cpp` 移到 `src/legacy/` 目录归档
3. 同时清理 `include/flow_end/` 中这些源文件对应的头文件

---

### 2.5 `follow_line.cpp` 和 `follow_line_test.cpp` 存在大量重复代码

**文件**：
- [`src/follow_line.cpp`](src/follow_line.cpp)（旧版，含环岛逻辑）
- [`src/follow_line_test.cpp`](src/follow_line_test.cpp)（新版，当前主力）

**问题**：两个文件中的角点检测逻辑 (`detectCorners` / 内联角点检测循环) 几乎完全一致，约 60 行重复代码。如果修改角点检测参数，需要在两个地方同步修改。

**建议**：
1. 短期：确认 `follow_line.cpp` 已废弃后直接删除
2. 如果仍需保留，将角点检测提取为独立函数 `detectCorners()`，两个文件共同调用

---

## 3. P2 — 控制算法改进

### 3.1 两套 PID 控制器并存

**文件**：
- [`src/PID.cpp`](src/PID.cpp) — 老 PID
- [`src/follow_motion_controller.cpp`](src/follow_motion_controller.cpp) — 新运动控制器

**当前状态**：

| 使用场景 | PID 来源 | Kp | Ki | Kd |
|----------|----------|----|----|-----|
| 初始预转角 (`handleInitialTurn`) | `PID.cpp` 的 `pid` | 0.05 | 0.0 | 0.0 |
| Y 分支转角 (`Y_ALIGNING_*`) | `PID.cpp` 的 `pid` | 0.05 | 0.0 | 0.0 |
| 正常巡线控制 | `FollowMotionController` | 1.00 | 0.0 | 0.08 |

**问题**：
- 两套 PID 参数独立调参，容易混乱
- `FollowMotionController` 有完善的速度规划、路径平滑、丢线滑行，但初始转角没用上
- `PID.cpp` 中的 `linear_params` 和 `angular_params` 实例化后从未被使用（死代码）

**建议**：
1. 让初始转角也走 `FollowMotionController`（只需设 `base_speed=0`，`allow_lost_coast=false`）
2. 删除 `PID.cpp` 中未使用的 `linear_params`、`angular_params`、`linear_pid`、`angular_pid`
3. 保留 `PID.cpp` 中的 `pid` 仅用于特殊场景（如 `Laser_linear`），待 `Laser_linear` 废弃后一并删除

---

### 3.2 `FollowMotionController` 的 Ki 默认为 0

**文件**：[`include/flow_end/follow_motion_controller.h`](include/flow_end/follow_motion_controller.h#L32)

```cpp
double ki_yaw = 0.00;
```

**问题**：积分项默认为 0，意味着长时间直线巡线时无法纠正微小的系统性偏置（如相机安装不水平导致的固定偏角）。直线段越长，累积误差越大。

**建议**：
- 将 `ki_yaw` 默认值设为 0.02~0.05
- 配合现有的 `integral_error_threshold`（积分分离阈值）和 `integral_limit`（积分限幅），可安全使用

---

### 3.3 `line_follower` PID：死区后积分项未清零

**文件**：[`line_follower/src/line_follower_node.cpp`](line_follower/src/line_follower_node.cpp#L573-L593)

```cpp
// 死区处理
if (abs(filteredError) < deadzone_) {
    filteredError = 0;
} else {
    filteredError = (filteredError > 0) ? filteredError - deadzone_ : filteredError + deadzone_;
}

// 积分分离
if (abs(filteredError) < error_threshold_) {
    integral += filteredError;      // filteredError 可能是 0
} else {
    integral = 0;
}
```

**问题**：进入死区后 `filteredError=0`，积分项不会被清零而是保持不变。当小车再次偏出死区时，之前积累的积分项突然生效，会导致控制量突变（过冲）。

**修复**：
```cpp
if (abs(filteredError) < deadzone_) {
    filteredError = 0;
    integral = 0;  // 添加：死区内清零积分
}
```

---

### 3.4 `ucar_followline.py` 曲率计算数值稳定性

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py#L1168)

```python
curvature = 5.2 * dx / (dx**2 + dy**2)**0.75
```

**问题**：
- 当 `dy` 很小时（BEV 图像底部近点），`**0.75` 可能导致分母过小，曲率急剧放大
- 常量 `5.2` 没有物理意义，是经验调参产物

**建议**：使用标准纯跟踪曲率公式：
```python
L2 = dx**2 + dy**2
curvature = 2.0 * dx / max(L2, 1.0)  # 标准纯跟踪曲率，添加最小值保护
```

---

## 4. P2 — 图像处理改进

### 4.1 `flow_end/process_image.cpp` 中的硬编码魔数

**文件**：[`src/process_image.cpp`](src/process_image.cpp#L9-L18)

```cpp
int local_thres = 0;
int block_size = 7;          // 魔数：邻域大小
int half = block_size / 2;
int _d = 6;                  // 魔数：差分步长
int find_nums = 5;           // 魔数：搜索次数上限
if (after_bizhang_x != 0) {
    find_nums = 10;          // 避障后翻倍（注释说是"改变检索图像范围"）
}
```

**问题**：
- `_d = 6`：左右扫描时的差分步长，影响梯度检测灵敏度。光照条件变化时需要调整
- `find_nums`：避障后切换的逻辑写死在代码中，应该可配置
- `block_size = 7`：自适应邻域大小，不同分辨率/赛道宽度可能需要不同值

**建议**：改为 ROS param，可通过 launch 文件或 dynamic_reconfigure 动态调整：

```cpp
// 新增 param 声明
private_nh.param<int>("block_size", block_size, 7);
private_nh.param<int>("diff_step", diff_step, 6);
private_nh.param<int>("find_nums", find_nums, 5);
private_nh.param<int>("find_nums_after_obstacle", find_nums_after_obstacle, 10);
```

---

### 4.2 `line_follower` 颜色阈值过于简单

**文件**：[`line_follower/src/line_follower_node.cpp`](line_follower/src/line_follower_node.cpp#L52-L62)

```cpp
int getColorNumber(const Vec3b &pixel) {
    if (pixel[0] > line_threshold_ && pixel[1] > line_threshold_
        && pixel[2] > line_threshold_)
        return 255; // 线
    else
        return 1;   // 可通过区域
}
```

**问题**：
- 检测"接近白色的像素"= 车道线，对光照变化极度敏感
- 白色地面反光会被误检测为线（室内/室外白墙场景）
- 没有局部自适应策略——背光区域白线变灰时无法检测

**对比 `flow_end` 的自适应策略** [`src/Findline_Adaptive.cpp`](src/Findline_Adaptive.cpp)：
```cpp
float local_thres = 0;
for (int dy = -half; dy <= half; dy++)
    for (int dx = -half; dx <= half; dx++)
        local_thres += AT_CLIP(img, x + dx, y + dy);
local_thres /= block_size * block_size;
local_thres -= clip_value;
// 用 local_thres 而非全局阈值来判断像素是否是线
```

**建议**：`line_follower` 已不再使用，直接归档删除。如果将来还需要一个轻量巡线方案，应从 `flow_end` 的 `Findline_Adaptive` 提取核心算法。

---

### 4.3 `ucar_followline.py` 去畸变被注释掉

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py#L436)

```python
# 被注释掉了
# img = cv2.undistort(src=img, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs)
```

**问题**：相机内参和畸变系数仍然定义在全局变量中（line 177-186），但去畸变步骤被跳过了。对于广角相机（如 120° 视野），画面边缘的桶形畸变会显著影响后续 BEV 透视变换的精度。

**建议**：恢复去畸变调用，或者至少在 launch 文件中添加 `enable_undistort` 开关参数。

---

## 5. P2 — 安全与鲁棒性

### 5.1 `flow_end` 没有图像超时保护

**文件**：[`src/follow_line_test.cpp`](src/follow_line_test.cpp#L1143-L1156)

```cpp
int followLineTestOnce() {
    if (!run_car) return 0;
    cv::Mat local_frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        if (frame.empty()) return -1;
        local_frame = frame.clone();
    }
    // ... 直接用 local_frame，没有检查帧的时间戳
}
```

**问题**：如果摄像头断开或图像流卡住，`frame` 保持最后一帧不变（不为空），`followLineTestOnce()` 会用过期图像继续巡线。同时 `FollowMotionController` 的 `dt` 会异常增大（被 `dt > 0.2` 钳制），但不会报错或停车。

**建议**：
1. 在 `Callback_test.cpp` 的 `imageCallback` 中记录 `last_image_time`
2. 在 `followLineTestOnce()` 入口处检查：
```cpp
if ((ros::Time::now() - last_image_time).toSec() > 0.5) {
    ROS_ERROR("Image stream timeout! Stopping.");
    publishStop();
    run_car = false;
    return -1;
}
```

---

### 5.2 停车逻辑使用 `while(1)` 无限循环

**文件**：[`src/follow_line_test.cpp`](src/follow_line_test.cpp#L762)

```cpp
while (ros::ok()) {
    ros::spinOnce();
    // ... parking logic
    if (parking_moved >= parking_total_dist) {
        // 到达停车距离，break
        break;
    }
    pub.publish(local_msg);
    parking_rate.sleep();
}
```

**问题**：
- 如果里程计故障（`odom_dist` 不增长），循环永远不会退出
- 虽然 `parking_rate.sleep()` 阻塞了 33ms，但如果 `odom` 话题停止发布，`parking_moved` 不会更新
- 有 `parking_stuck` 检测（[line 800](src/flow_end/src/follow_line_test.cpp#L800)）但只是打印警告，不会触发退出

**建议**：添加最大时间和最大循环次数保护：
```cpp
const ros::Time parking_deadline = ros::Time::now() + ros::Duration(15.0);  // 最大15秒
const int max_parking_loops = 500;  // 500 * 33ms ≈ 16.5秒

while (ros::ok() && ros::Time::now() < parking_deadline && parking_loop_count < max_parking_loops) {
    // ...
}
if (parking_loop_count >= max_parking_loops) {
    ROS_ERROR("[PARKING] Timeout! Forcing stop.");
    publishStop();
}
```

---

### 5.3 `ucar_followline.py` 多进程架构缺少自动重启

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py#L2722-L2729)

```python
while True:
    for p in critical_processes:
        if not p.is_alive():
            raise RuntimeError(f"{p.name} died")
    time.sleep(1)
```

**问题**：
- 如果巡线进程因未捕获异常崩溃，主进程抛出 `RuntimeError` 进入 `finally` 块
- 底盘控制进程也会被终止 → 小车立刻失去控制
- 没有尝试重启崩溃的进程

**建议**：
```python
import time

MAX_RESTART_ATTEMPTS = 3
restart_counts = {p.name: 0 for p in critical_processes}

while True:
    for p in critical_processes:
        if not p.is_alive():
            if restart_counts[p.name] < MAX_RESTART_ATTEMPTS:
                rospy.logwarn(f"进程 {p.name} 崩溃，尝试重启 ({restart_counts[p.name]+1}/{MAX_RESTART_ATTEMPTS})")
                # 在重启前，底盘控制进程应执行安全停车
                restart_counts[p.name] += 1
                # ... 重启逻辑
            else:
                rospy.logerr(f"进程 {p.name} 多次重启失败，系统终止")
                raise RuntimeError(f"{p.name} died after {MAX_RESTART_ATTEMPTS} restarts")
    time.sleep(1)
```

---

### 5.4 `flow_end` 丢线后无渐进减速

**文件**：[`src/follow_line_test.cpp`](src/follow_line_test.cpp#L1186-L1193)

```cpp
if (rpts_num == 0) {
    zeroCount++;
    if (zeroCount >= 2) {
        zero_flag = true;  // 2帧后就完全停车
    }
}
```

**问题**：丢线 2 帧（约 66ms）后 `zero_flag=true`，在速度计算中直接置 `v=0`。这在偶发丢线（如地面反光、阴影）时会导致不必要的急刹。

**对比**：`FollowMotionController` 已经有 `lost_line_coast_sec`（丢线滑行时间）和 `lost_line_coast_speed_scale`（滑行速度比例），但当前代码中 `zero_flag` 逻辑绕过了这个滑行机制。

**建议**：移除 `zero_flag` 逻辑，将丢线处理完全交给 `FollowMotionController` 的 coasting 机制。或者在 `zero_flag` 生效前也先走 coasting，超过 coast 时间后再停车。

---

### 5.5 缺少传感器健康检查

三个巡线包都没有检查传感器数据是否新鲜：

| 传感器 | 话题 | 检查 |
|--------|------|------|
| 摄像头 | `/ucar_camera/image_raw` | ❌ 未检查帧时间戳 |
| IMU | `/imu` | ❌ 未检查数据新鲜度 |
| 里程计 | `/odom` | ❌ 仅用于辅助，未检查超时 |

**建议**：在 `flow_end/Callback_test.cpp` 中添加通用传感器超时检查：
```cpp
ros::Time last_image_time(0);
ros::Time last_imu_time(0);
const double SENSOR_TIMEOUT = 0.5;  // 500ms

// 在 imageCallback 中更新
last_image_time = msg->header.stamp;

// 在主循环中检查
bool sensors_healthy = 
    (ros::Time::now() - last_image_time).toSec() < SENSOR_TIMEOUT &&
    (ros::Time::now() - last_imu_time).toSec() < SENSOR_TIMEOUT;
```

---

## 6. P3 — 代码质量

### 6.1 注释语言混用与编码损坏

**文件**：多个 `flow_end` 源文件

**问题**：
- [`src/process_image.cpp`](src/process_image.cpp)：中文注释显示为乱码（`ԭͼ     ұ`、`环岛部分` 等），说明文件编码不是 UTF-8
- 同一文件中中英文混用：`// 平滑处理` vs `// Local background estimate`
- 部分函数完全没有注释（如 `corner_move`）

**建议**：
1. 统一使用 UTF-8 编码保存所有源文件
2. 统一注释语言为英文（便于团队协作）或系统性地使用中文
3. 为所有公开函数添加 Doxygen 风格注释

---

### 6.2 `ucar_followline.py` 存在大量被注释的旧代码

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py)

**示例**：
- Line 122-127：被注释掉的 `Laser_linear` 备选实现（约 180 行）
- Line 1103-1108：被注释掉的 `calculate_control` 调用
- Line 1120：被注释掉的 `_draw_twist_info` 调用
- Line 1297-1303：被注释掉的逆透视变换叠加逻辑

**问题**：总共约 200+ 行被注释的死代码。这些代码要么已经过时，要么通过 git history 可以找回。

**建议**：删除所有被注释的死代码。如果需要回溯，使用 `git log -p --follow <file>` 查看历史。

---

### 6.3 日志级别使用不当

**文件**：[`src/process_image.cpp`](src/process_image.cpp#L76-L118)

每个循环都打印：
```cpp
ROS_INFO("rpts0_num: %d\n", rpts0_num);       // line 76
ROS_INFO("rpts1_num: %d\n", rpts1_num);       // line 84
ROS_INFO("rpts0s_num: %d\n", rpts0s_num);     // line 96
ROS_INFO("rpts1s_num: %d\n", rpts1s_num);     // line 98
ROS_INFO("rptsc0_num: %d\n", rpts1_num);      // line 116
ROS_INFO("rpts1_num: %d\n", rpts1_num);       // line 118
```

**问题**：每帧图像都打印 6 条 INFO 日志。在 30Hz 巡线时，每秒产生 180 条日志，严重干扰真正需要关注的 WARN/ERROR 信息。

**建议**：
- 正常巡线状态日志改用 `ROS_DEBUG`
- 需要周期性查看的信息（如误差、速度）使用 `ROS_INFO_THROTTLE(1.0, ...)`
- 只有异常情况（丢线、传感器超时、停车触发）才用 `ROS_WARN` 或 `ROS_INFO`

在 `follow_line_test.cpp` 中已经较好地使用了 `ROS_WARN_THROTTLE`（如 line 1215-1228），应推广到所有文件。

---

### 6.4 `Laser_linear.cpp` 包含 180 行被注释的备选实现

**文件**：[`src/Laser_linear.cpp`](src/Laser_linear.cpp#L122-L301)

**问题**：`Laser_linear` 是环岛流程的一部分，已经被 `follow_test` 弃用。文件上半部分（~120 行）是当前代码，下半部分（~180 行）是完全被注释掉的备选实现。

**建议**：如果 `follow_end` 及其依赖的 `Laser_linear` 已废弃，直接删除整个文件。如果仍需保留，删除被注释的部分。

---

### 6.5 `CMakeLists.txt` 中 `include_directories` 包含不存在的路径

**文件**：[`CMakeLists.txt`](CMakeLists.txt#L24)

```cmake
include_directories(
  include
  include/flow_test    # 该目录不存在
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
```

**问题**：`include/flow_test` 目录不存在，但 CMake 不会报错（只是无效路径）。这可能是历史遗留。

**建议**：移除 `include/flow_test`。

---

## 7. P3 — 参数管理

### 7.1 `follow_test.launch` 参数过多

**文件**：[`launch/follow_test.launch`](launch/follow_test.launch)（220 行，约 80 个 ROS param）

**问题**：
- 核心参数（topic 名称、base_speed、parking 开关）和控制参数（control_*、y_*）混在一起
- 大量参数有合理的默认值，不需要在 launch 文件中显式写出
- 每次调参需要编辑 launch 文件然后重启节点

**建议**：

1. **核心 launch 文件**只保留 topic 名称和行为开关（约 30 行）：
```xml
<launch>
  <arg name="image_topic" default="/ucar_camera/image_raw" />
  <arg name="cmd_vel_topic" default="/cmd_vel" />
  <arg name="path_select" default="right" />
  <arg name="parking_enabled" default="true" />
  <arg name="base_speed" default="0.30" />
  <arg name="initial_turn_enabled" default="true" />
  <!-- 加载参数文件 -->
  <rosparam command="load" file="$(find flow_end)/config/default.yaml" />
  <node pkg="flow_end" type="follow_test" name="follow_test" output="screen">
    <!-- 仅覆盖核心参数 -->
  </node>
</launch>
```

2. **运动控制参数**下沉到 [`config/default.yaml`](config/default.yaml)（新建）：
```yaml
# 运动控制器参数
control_path_smooth_window: 2
control_path_ema_alpha: 0.35
control_error_filter_alpha: 0.65
control_yaw_deadband: 0.015
control_kp_yaw: 1.00
control_kd_yaw: 0.08
# ...

# 停车参数
parking_extra_dist: 0.315
parking_forward_speed: 0.15
parking_allow_either_l: true

# Y 分支参数
y_approach_dist: 0.20
y_turn_angle_deg: 45.0
# ...
```

3. **未来考虑**：添加 `dynamic_reconfigure` 支持，允许运行时调参。

---

### 7.2 `ucar_followline.py` 参数散落文件各处

**文件**：[`ucar_nav/scripts/ucar_followline.py`](../ucar_nav/scripts/ucar_followline.py)

| 位置 | 参数数量 | 类型 |
|------|---------|------|
| LINE 104-174 全局变量区 | ~70 个 | 二值化、迷宫法、纯跟踪、PID、停车 |
| `PreProcess.__init__` | 6 个 | 图像尺寸、俯仰角、地面范围 |
| `calculate_speed.__init__` | ~15 个 | PID 增益、曲率适应、加速度限制 |

**问题**：没有统一的配置文件。每次调参需要改 Python 代码并重启。且参数定义和参数使用间隔上千行，难以追踪。

**建议**：创建 `config/lane_follow.yaml`：
```yaml
image:
  width: 640
  height: 360
  binary_threshold: 180

perspective:
  pitch_deg: 18
  camera_height: 0.11
  ground_width: 0.78
  ground_depth: 0.50

maze_tracker:
  block_size: 3
  clip_value: 6
  max_steps: 200
  move_offset: 116

pure_pursuit:
  lookahead_distance: 115
  lateral_gain: 2.6

speed_control:
  max_linear_speed: 0.32
  max_angular_speed: 1.4
  kp_yaw: 0.7
  kd_yaw: 0.1
  curvature_speed_factor: 2.0
```

在代码中使用 `rospy.get_param()` 或 `rosparam` 加载。

---

## 8. 总结优先级排序

| 优先级 | 编号 | 问题 | 影响 | 预计工作量 |
|--------|------|------|------|-----------|
| 🔴 **P0** | 1.1 | 停车距离计算 bug | 停车功能不可靠 | 1 行修复 |
| 🔴 **P0** | 1.2 | 微分项滤波不一致 | 转向不稳定 | 3 行修复 |
| 🔴 **P0** | 1.3 | ROS_INFO 变量名错误 | 调试误导 | 2 行修复 |
| 🔴 **P0** | 1.4 | 角度转换魔数 | 精度损失 | 2 行修复 |
| 🔴 **P1** | 2.2 | 三套巡线并存 | 维护成本 3x | 1-2 天 |
| 🔴 **P1** | 2.3 | 60个全局变量 | 不可测试 | 2-3 天 |
| 🔴 **P1** | 2.1 | 2753行单体文件 | 改动风险大 | 1-2 天 |
| 🟡 **P2** | 5.1 | 无图像超时保护 | 安全性 | 0.5 天 |
| 🟡 **P2** | 5.2 | while(1) 无超时 | 安全性 | 0.5 天 |
| 🟡 **P2** | 5.4 | 丢线无渐进减速 | 乘坐体验 | 0.5 天 |
| 🟡 **P2** | 4.1 | 图像处理魔数 | 适应性差 | 0.5 天 |
| 🟡 **P2** | 3.1 | 两套 PID 并存 | 调参混乱 | 1 天 |
| 🟡 **P2** | 3.2 | Ki 默认为 0 | 直线偏航 | 调参 |
| 🟢 **P3** | 2.4 | 死代码 | 编译速度 | 0.5 天 |
| 🟢 **P3** | 6.1 | 注释编码损坏 | 可读性 | 0.5 天 |
| 🟢 **P3** | 6.2 | 注释死代码 | 可读性 | 0.5 天 |
| 🟢 **P3** | 6.3 | 日志级别 | 调试体验 | 0.5 天 |
| 🟢 **P3** | 7.1 | launch 参数过多 | 调参效率 | 0.5 天 |

---

## 附录：各包文件清单与用途

### flow_end（主力巡线包）

| 文件 | 用途 | 状态 |
|------|------|------|
| `src/follow_test.cpp` | ROS 入口，参数加载，主循环 | ✅ 活跃 |
| `src/Callback_test.cpp` | 图像/IMU/里程计/控制指令回调 | ✅ 活跃 |
| `src/follow_line_test.cpp` | 核心业务逻辑：状态机、停车、Y分支、控制路径选择 | ✅ 活跃 |
| `src/follow_motion_controller.cpp` | 运动控制器：路径平滑、PID、速度规划 | ✅ 活跃 |
| `src/process_image.cpp` | 图像处理管道：自适应阈值、左右线提取、点处理 | ✅ 活跃 |
| `src/Findline_Adaptive.cpp` | 迷宫法巡线（左右手法则） | ✅ 活跃 |
| `src/PID.cpp` | PID 控制器 | ✅ 活跃（部分冗余） |
| `src/Point_Process.cpp` | 点平滑、等距采样、角度估计、NMS | ✅ 活跃 |
| `src/track_line.cpp` | 左右巡线偏移计算 | ✅ 活跃 |
| `src/corner_move.cpp` | 角点目标位置计算 | ✅ 活跃 |
| `src/ImagePerspectiveInit.cpp` | 逆透视变换初始化 | ✅ 活跃 |
| `src/MatTransform.cpp` | cv::Mat 与二维数组转换 | ✅ 活跃 |
| `src/generateLookupTable.cpp` | 查找表生成 | ✅ 活跃 |
| `src/Signal.cpp` | SIGINT 信号处理 | ✅ 活跃 |
| `src/follow_line.cpp` | 旧版巡线（含环岛/激光雷达） | ❌ 废弃 |
| `src/follow.cpp` | 旧版入口 | ❌ 废弃 |
| `src/Callback.cpp` | 旧版回调 | ❌ 废弃 |
| `src/Laser_linear.cpp` | 旧版激光雷达避障 | ❌ 废弃 |
| `src/image_process_debug.cpp` | 图像处理调试工具 | ⚠️ 调试用 |

### line_follower（遗留巡线包）

| 文件 | 用途 | 状态 |
|------|------|------|
| `src/line_follower_node.cpp` | 图像处理 + 巡线 + PID 一体化（约 670 行） | ❌ 废弃 |
| `config/pid.cfg` | PID 参数配置文件 | ❌ 废弃 |
| `config/image.cfg` | 图像处理参数配置文件 | ❌ 废弃 |

### ucar_nav（Python 巡线系统）

| 文件 | 用途 | 状态 |
|------|------|------|
| `scripts/ucar_followline.py` | 主程序（2753 行）：图像处理、车道检测、纯跟踪、PID、状态机、环岛 | ⚠️ 国赛版 |
| `scripts/followline_service_control.py` | 控制节点服务 | ⚠️ 国赛版 |
| `launch/followline.launch` | 启动文件 | ⚠️ 国赛版 |
