# 智慧工厂比赛总流程

对应车上工作空间：`/home/ucar/instant_ws`。正式入口：

```bash
bash /home/ucar/instant_ws/src/startup_scripts/start_full_flow.sh
```

## 一、仿真联动总流程

小车为 `ucar@192.168.1.10`；仿真电脑运行 Gazebo、仿真地图、任务3和 TCP 桥。两台电脑各自运行 ROS Master，小车不要把 `ROS_MASTER_URI` 指向仿真电脑。桥默认端口为 `26003`。

仿真电脑先启动：

```bash
bash run_task3_sim.sh
```

小车上配置并检查网络：

```bash
ssh ucar@192.168.1.10
export ENABLE_SIMULATION=true
export SIM_BRIDGE_HOST='仿真电脑IP'
export SIM_BRIDGE_PORT=26003
export SPARK_CREDENTIALS_FILE=/home/ucar/instant_ws/src/ucar_2026_smart_factory_llm/config/spark_credentials.env
ping -c 3 "$SIM_BRIDGE_HOST"
nc -vz "$SIM_BRIDGE_HOST" "$SIM_BRIDGE_PORT"
bash /home/ucar/instant_ws/src/startup_scripts/start_full_flow.sh
```

仿真完整顺序：

```text
任务1 语音完整指令 -> 第一段导航 -> 扫描三个不同二维码 URL
     -> 联网解析货品 -> Spark 判断真实/仿真类别 -> 语音播报
任务2 真实车间导航 -> OCR找真实车间 -> 厂牌对准、横移、前进停车
     -> 播报“已将[货品]放入[真实仓库]”
任务3 通过 SIM_BRIDGE_HOST:26003 联动仿真车间
任务2第二阶段 -> 前往记忆的仿真最佳观察点 -> OCR确认 -> 第二次停车
     -> 播报“仿真任务已完成，已将[货品]放入[仿真仓库]”
任务4 前往停止线 -> 切换相机 -> 红绿灯识别
任务5 复用相机进入 flow_end/line_follower 巡线并完成
```

不做仿真：

```bash
export ENABLE_SIMULATION=false
bash /home/ucar/instant_ws/src/startup_scripts/start_full_flow.sh
```

此时顺序为 `任务1 -> 真实车间任务2 -> 任务4 -> 任务5`。`SIM_BRIDGE_HOST` 是仿真电脑 IP，不是小车 IP；仿真桥未启动时不要开启仿真。

## 二、网络、凭据和环境

```bash
ssh ucar@192.168.1.10
cd /home/ucar/instant_ws
hostname -I
echo "$ROS_MASTER_URI"
```

正式脚本会加载 ROS Noetic 和车上当前工作空间。不要在一个终端叠加旧 `src3`、`src_pure_runtime_ws_v3` 或其他 catkin overlay；环境混乱时重新开 SSH 终端。

Spark 凭据优先放在：

```text
/home/ucar/instant_ws/src/ucar_2026_smart_factory_llm/config/spark_credentials.env
```

兼容旧位置：`/home/ucar/instant_ws/src/iden_controller/config/spark_credentials.env`。

```bash
chmod 600 /home/ucar/instant_ws/src/ucar_2026_smart_factory_llm/config/spark_credentials.env
```

脚本支持 `SPARK_API_PASSWORD`，也兼容同一应用的 `SPARK_API_KEY` 和 `SPARK_API_SECRET`。遇到 `AppIdNoAuthError`、401、403 或 500，先检查应用类型、凭据配对、系统时间和网络。

## 三、相机与资源占用

前段二维码/OCR使用 USB 相机；任务4/5释放后启动 `ucar_camera`。当前任务4/5接口：

```text
设备：640x480，YUYV，30 FPS
ROS：/ucar_camera/image_raw，640x480，rgb8，约15 FPS
```

检查 `/dev/video0`：

```bash
fuser -v /dev/video0
rosnode list | egrep 'usb_cam|ucar_camera|image_view'
rostopic hz /ucar_camera/image_raw
```

红绿灯和巡线复用同一个 `/ucar_camera/image_raw`，不要重复启动相机。

## 四、启动、调试和恢复

```bash
bash /home/ucar/instant_ws/src/startup_scripts/start_full_flow.sh debug:=true
FLOW_DRY_RUN=1 bash /home/ucar/instant_ws/src/startup_scripts/start_full_flow.sh
rosservice call /competition/resume
rosservice call /competition/abort
```

推荐一次说完整句：

```text
小飞小飞，前往物品领取区取得食品加工类放置在对应车间，并在仿真环境中取得电子产品类放置在对应车间。
```

必须同时包含真实类别和仿真类别。查看状态：

```bash
rostopic echo /competition/task_state
rostopic echo /factory/tts_text
rosnode list
```

## 五、比赛前内存清理

```bash
bash /home/ucar/instant_ws/src/startup_scripts/prepare_competition_memory.sh --dry-run
bash /home/ucar/instant_ws/src/startup_scripts/prepare_competition_memory.sh
```

默认只清理 Pylance、C/C++ IntelliSense、Copilot 等开发辅助进程，不碰 ROS、导航、雷达、相机、底盘、SSH和当前终端。关闭车载 VS Code 后需要更多空间时才用：

```bash
bash /home/ucar/instant_ws/src/startup_scripts/prepare_competition_memory.sh --full-vscode
```

## 六、真实参数微调提示

每次只改一个参数。推荐顺序：相机接口 -> 最终导航点 -> 扫描点容差 -> 锥桶余量 -> OCR观察时间 -> 停车速度。

- `coverage_goal_retry` 或 `coverage_anchor_skipped`：先查 TF、地图和当前位姿，再调扫描点。
- `coverage_goal_soft_timeout_sec` 是软超时，`coverage_goal_hard_timeout_sec` 是硬超时；硬超时不要无限增大。
- AMCL 与 RViz 偏差明显时，先查 `/map -> /odom -> /base_link`、激光时间戳和底盘里程计。
- 锥桶余量每次只改 `0.01~0.02m`；余量大可能在缝隙前停住，余量小会碰桶。
- 看到锥桶就后退很远，先查重复恢复、重复清理 costmap 或多个节点抢 `/cmd_vel`。

当前总入口常用停车参数：

```text
parking_staging_offset                 0.55m
parking_goal_offset                    0.26m
parking_staging_position_tolerance     0.10m
parking_docking_timeout_sec            20s
parking_lidar_stop_distance            0.15m
parking_recenter_max_travel            0.30m
parking_recenter_tolerance             0.04
```

停车建议：车身已垂直但厂牌总在一侧，先检查 `parking_recenter_lateral_sign`；停车太慢先小幅增加横移/前进速度；靠墙过近优先增加 `parking_goal_offset`，每次 `0.01~0.02m`；停车被挡时确认 move_base 已释放 `/cmd_vel`。

当前任务4最终导航点：

```text
x = 0.2595m
y = -3.09m
yaw = -1.5596rad，约 -89.36°
```

到达后还会执行约 `0.18m` 前进段。

## 七、日志速查

```text
voice_ready/listening_command       语音
qr_center_ready/scanning_qr         二维码
SRC3_SPARK_DECISION_OK              大模型成功
ROOM NAV patrolling                  锥桶房间巡检
target_locked                       OCR目标锁存
parking_*                           停车状态机
arrived                             车间停车完成
task4/camera_ready                  任务4相机完成切换
traffic_light/line_following        红绿灯/巡线
```

`NO PATH`先查 TF、地图和 costmap；`Costmap2DROS transform timeout`先查 TF和时间戳；`factory_navigator exited`继续看子日志最后状态；`competition_flow paused`修复后再 `resume`。

## 八、主要文件职责

```text
总流程：src/startup_scripts/start_full_flow.sh
内存清理：src/startup_scripts/prepare_competition_memory.sh
总入口：src/ucar_2026_competition/launch/start_full_flow.launch
总控：src/ucar_2026_competition/scripts/competition_flow_current_front_39ff_room_v1.py
阶段入口：src/ucar_2026_competition/launch/flow_node_current_front_39ff_room_v1.launch
Spark：src/ucar_2026_smart_factory_llm
二维码：src/ucar_2026_qr_speak_test 或当前 launch 引用的 qr_decoder
厂牌OCR：src/factory_sign_ppocr_rknn_test
红绿灯：src/traffic_light_vision
巡线：src/flow_end、src/line_follower
相机：src/ucar_camera
雷达：src/ydlidar
```

修改启动脚本、launch、总控或参数文件前先备份；修改后同步核对参数名、话题名、节点名和相机资源占用。
