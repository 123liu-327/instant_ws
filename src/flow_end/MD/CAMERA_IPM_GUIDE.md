# flow_end 相机俯仰角与 IPM 操作流程

本文适用于 ROS 1 Noetic 和 `flow_end` 包，目标图像话题为
`/ucar_camera/image_raw`，标定及运行分辨率固定为 `640x480`。

## 1. 棋盘参数

当前棋盘的实际参数：

- 横向 12 个黑白方格
- 纵向 9 个黑白方格
- 单个方格边长 `0.020 m`

OpenCV 使用的是相邻方格之间的内角点数，不是方格数。因此所有命令都应使用：

```text
board_cols = 11
board_rows = 8
square_size_m = 0.020
```

不要把参数写成 `12x9`，也不要把棋盘纸张外框尺寸当作方格边长。

## 2. 编译和环境准备

在机器人或 ROS Noetic 系统中执行：

```bash
cd ~/instant_ws
catkin_make
source devel/setup.bash
rospack find flow_end
```

如果实际工作空间路径不是 `~/instant_ws`，替换为车上的真实路径。每次新开终端后都要执行：

```bash
source ~/instant_ws/devel/setup.bash
```

标定前停止巡线和底盘控制节点。标定工具只发布调试图像，不应发布
`/cmd_vel`，但仍建议将车辆架起或关闭电机使能。

## 3. 检查相机话题

先启动相机驱动，再检查话题和分辨率：

```bash
rostopic list | grep ucar_camera
rostopic hz /ucar_camera/image_raw
rostopic echo -n 1 /ucar_camera/image_raw/width
rostopic echo -n 1 /ucar_camera/image_raw/height
```

结果必须是宽 `640`、高 `480`。如果不是，先修改相机驱动输出尺寸；程序不会
把其他分辨率静默缩放为 `640x480`。

也可以检查画面：

```bash
rosrun rqt_image_view rqt_image_view /ucar_camera/image_raw
```

## 4. 标定相机内参

内参标定时手持棋盘，从不同距离、位置和角度覆盖整个画面。棋盘此时不需要平放
在地面。

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 11x8 \
  --square 0.020 \
  image:=/ucar_camera/image_raw \
  camera:=/ucar_camera \
  --no-service-check
```

操作顺序：

1. 缓慢移动棋盘，使左、右、上、下、中央和四角都有样本。
2. 改变棋盘与相机的距离及倾斜角度。
3. `CALIBRATE` 按钮可用后点击并等待计算完成。
4. 查看误差后点击 `SAVE`。

一般会生成 `/tmp/calibrationdata.tar.gz`。提取并放入 `flow_end/config`：

```bash
mkdir -p /tmp/flow_end_camera_calibration
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/flow_end_camera_calibration
cp /tmp/flow_end_camera_calibration/ost.yaml \
  "$(rospack find flow_end)/config/camera_intrinsics.yaml"
```

确认文件存在：

```bash
test -f "$(rospack find flow_end)/config/camera_intrinsics.yaml" && echo OK
```

接下来从方案 A 和方案 B 中选择一种。建议先运行方案 A 查看旧矩阵结果，再使用
方案 B 得到更可信的实际安装姿态。

## 5. 方案 A：保留旧矩阵并反解等效俯仰角

执行：

```bash
roslaunch flow_end recover_legacy_ipm.launch
```

如内参文件放在其他位置，可以明确传入：

```bash
roslaunch flow_end recover_legacy_ipm.launch \
  intrinsics_file:=/绝对路径/camera_intrinsics.yaml \
  output_file:="$(rospack find flow_end)/config/camera_ipm.yaml" \
  pixels_per_meter:=500.0
```

程序会输出：

- `pitch_down_deg`：旧矩阵在当前内参下的等效向下俯仰角
- `camera_height_m`：在 `500 px/m` 约定下的等效高度
- `axis_orthogonality_error`：两条地面轴的正交误差
- `axis_scale_mismatch_ratio`：两条地面轴的尺度不一致度

生成文件为：

```text
flow_end/config/camera_ipm.yaml
```

该方案会原样保存旧的 `H_bird_to_image`，并写入它的逆矩阵。若打印
`WARNING: the old matrix is not well described by a physical pinhole pose`，说明旧矩阵
可能来自人工四点拉伸、裁剪或翻转，反解角度只能当作近似值，不能当作真实机械
安装角。此时应执行方案 B。

脚本写完配置后正常退出，`roslaunch` 显示进程结束不代表失败。检查返回码和终端中
是否出现 `Legacy IPM recovery failed`。

## 6. 方案 B：地面棋盘重新计算真实姿态和矩阵

棋盘摆放要求：

- 棋盘必须完全平放在车辆所在的同一地面上。
- 棋盘横向边与车辆横轴平行。
- 棋盘中心线与车辆或相机中心线尽量重合。
- 棋盘不能翘曲，地面不能有明显坡度。
- 相机支架在采集期间不能晃动。

执行：

```bash
roslaunch flow_end calibrate_ground_pose.launch \
  board_cols:=11 \
  board_rows:=8 \
  square_size_m:=0.020 \
  pixels_per_meter:=500.0 \
  sample_count:=30
```

默认参数已经是 `11x8` 和 `0.020 m`，因此也可以直接运行：

```bash
roslaunch flow_end calibrate_ground_pose.launch
```

观察角点和鸟瞰预览：

```bash
rosrun rqt_image_view rqt_image_view \
  /flow_end_ground_pose_calibrator/checkerboard_overlay
```

另开终端观察：

```bash
rosrun rqt_image_view rqt_image_view \
  /flow_end_ground_pose_calibrator/bird_preview
```

程序会筛选重投影 RMS 不超过 `0.8 px` 的结果，并在连续 30 帧俯仰角标准差不超过
`0.3 deg` 后写入 `config/camera_ipm.yaml`。如果一直不能完成：

- 保证棋盘完整出现在图像中且没有强反光。
- 增加环境照明并擦拭镜头。
- 检查棋盘实际方格是否确实为 `20 mm`。
- 检查棋盘是否平放、相机支架是否振动。
- 确认内参和当前相机分辨率都为 `640x480`。

## 7. 静态验收

先只启动图像调试节点，不启动正式巡线控制：

```bash
roslaunch flow_end image_process_debug.launch \
  image_topic:=/ucar_camera/image_raw \
  camera_calibration_file:="$(rospack find flow_end)/config/camera_ipm.yaml"
```

查看调试图像：

```bash
rosrun rqt_image_view rqt_image_view /flow_end/image_process_debug
```

验收标准：

- 节点启动日志能打印俯仰角、相机高度、尺寸、矩阵行列式和有效映射比例。
- 不出现配置缺失、矩阵奇异、有效映射比例过低或图像尺寸不匹配错误。
- 鸟瞰图中的两条平行车道线应基本平行。
- `pixels_per_meter=500` 时，`0.42 m` 车道宽度应约为 `210 px`，允许约
  `210 +/- 10 px`。
- 图像中不应有大面积异常拉伸、翻转或所有内容集中在一个角落。
- 调试阶段确认没有节点发布非零 `/cmd_vel`。

检查速度话题：

```bash
rostopic echo /cmd_vel
```

## 8. 正式运行

静态验收通过后再启动正式巡线：

```bash
roslaunch flow_end follow_end.launch \
  image_topic:=/ucar_camera/image_raw \
  camera_calibration_file:="$(rospack find flow_end)/config/camera_ipm.yaml"
```

第一次实车测试应采用低速、空旷场地并准备急停。先确认直线段稳定，再测试弯道，
不要在逆透视仍有拉伸或翻转时通过修改 PID 掩盖图像问题。

## 9. 何时必须重新标定

发生以下任一情况后，至少重新执行方案 B；更换镜头或分辨率后还要重新标定内参：

- 相机支架被拆卸、碰撞或重新拧紧。
- 相机俯仰角、高度或横向位置变化。
- 更换相机或镜头。
- 修改相机输出分辨率、裁剪区域或图像翻转设置。
- 鸟瞰车道宽度明显偏离 `210 px`。

不要修改里程计回调中的车体 `pitch` 来修复 IPM。该数值是车辆姿态信息，不参与
相机逆透视矩阵计算。
