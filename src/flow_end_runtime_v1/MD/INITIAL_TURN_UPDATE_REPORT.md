# follow_test 初始转角更新报告

## 一、更新目的

这次更新给 `follow_test` 增加了一个“起步预转角”逻辑。

主要解决的问题是：小车一开始遇到左、中、右三岔路时，摄像头可能同时看到多条线，即使发送 `Left` 或 `Right`，小车也容易被中间路线吸过去。

新的逻辑是：

```text
收到 Left  -> 先向左原地转一小段角度 -> 再进入左巡线
收到 Right -> 先向右原地转一小段角度 -> 再进入右巡线
收到 Middle -> 不预转，直接中线巡线
```

这样做的目的，是先让车身和摄像头朝向目标岔路，让视觉算法更容易识别目标方向的线。

## 二、小车状态

新增了一个运动状态机：

```cpp
enum class MotionState {
    IDLE,
    ALIGNING_LEFT,
    ALIGNING_RIGHT,
    FOLLOWING
};
```

各状态含义：

```text
IDLE
等待启动指令。

ALIGNING_LEFT
起步左转对准左岔路。

ALIGNING_RIGHT
起步右转对准右岔路。

FOLLOWING
正常巡线。
```

指令和状态关系：

```text
/follow_begin Left    -> ALIGNING_LEFT  -> FOLLOWING
/follow_begin Right   -> ALIGNING_RIGHT -> FOLLOWING
/follow_begin Middle  -> FOLLOWING
/follow_begin Stop    -> IDLE
```

## 三、初始转角逻辑

初始转角逻辑在：

```text
src/follow_line_test.cpp
```

核心函数是：

```cpp
handleInitialTurn()
```

每一帧的执行顺序是：

```text
读取相机图像
灰度化
process_image()
selectControlPath()
handleInitialTurn()
```

也就是说，转角过程中并不是盲转。小车一边转，一边仍然在识别图像里的线。

## 四、什么时候停止转角

转角停止有两个条件，满足任意一个就停止：

```cpp
turned_abs >= initial_turn_angle_deg
selected_count >= initial_turn_rpts_threshold
```

含义：

```text
1. 小车已经转过设定角度。
2. 或者目标方向的线点数量已经足够多。
```

左巡线时，判断的是左侧目标线点数：

```cpp
rptsc0e_num
```

右巡线时，判断的是右侧目标线点数：

```cpp
rptsc1e_num
```

这样设计的好处是：如果目标线已经识别得很好，就不用继续转到固定角度，避免转太多导致线跑出画面。

## 五、转角时如何控制底盘

转角阶段仍然使用现有底盘控制方式，也就是发布 `/cmd_vel`。

转角时不前进：

```cpp
msg.linear.x = 0.0;
```

左转：

```cpp
msg.angular.z = +initial_turn_angular_speed;
```

右转：

```cpp
msg.angular.z = -initial_turn_angular_speed;
```

转角结束后，会先发布停车速度，然后切换到 `FOLLOWING`。下一帧开始正常巡线。

## 六、新增参数

在 `follow_test.launch` 中新增了这些参数：

```xml
<arg name="initial_turn_enabled" default="true" />
<arg name="initial_turn_angle_deg" default="30.0" />
<arg name="initial_turn_angular_speed" default="0.35" />
<arg name="initial_turn_rpts_threshold" default="40" />
```

参数含义：

```text
initial_turn_enabled
是否启用起步预转角。

initial_turn_angle_deg
最大预转角度，单位是度。

initial_turn_angular_speed
预转角时使用的角速度，对应 /cmd_vel 的 angular.z。

initial_turn_rpts_threshold
目标线点数阈值。超过这个数量，就认为当前角度已经可以进入巡线。
```

启动时可以临时修改：

```bash
roslaunch flow_end follow_test.launch initial_turn_angle_deg:=45 initial_turn_rpts_threshold:=60
```

## 七、调试日志

转角过程中会打印：

```text
initial turn path=left yaw_delta=... target=... selected_rpts=... threshold=...
```

转角完成时会打印：

```text
initial turn finished path=left yaw_delta=... selected_rpts=... angle_ok=... line_ok=...
```

重点看这些字段：

```text
yaw_delta
当前已经转过的角度，来自 IMU yaw。

selected_rpts
当前目标线的点数。

angle_ok
是否因为达到设定角度而结束转角。

line_ok
是否因为目标线点数足够而提前结束转角。
```

## 八、建议调参

建议初始值：

```text
initial_turn_angle_deg=25~30
initial_turn_angular_speed=0.30~0.35
initial_turn_rpts_threshold=40
```

如果小车还是容易走中间路，可以增大角度：

```text
30 -> 40 -> 45
```

如果小车转太多，导致线跑出画面，可以减小角度或降低点数阈值：

```text
initial_turn_angle_deg:=25
initial_turn_rpts_threshold:=25
```

## 九、编译提醒

这次修改涉及 `.cpp` 和 `.h` 文件，所以需要重新编译：

```bash
catkin_make
source devel/setup.bash
```
