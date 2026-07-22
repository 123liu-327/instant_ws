# 巡线调试视频保存功能使用说明

## 📹 功能概述

已将原来的ROS图像调试话题发布（`/flow_end/follow_test_debug`）改为**直接保存为AVI视频文件**，方便离线分析和长期记录。

---

## 🔧 配置参数

在 `follow_test.launch` 中新增了3个视频保存参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_video_record` | bool | `true` | 是否启用视频录制 |
| `video_fps` | int | `10` | 视频帧率（建议10-15 FPS） |
| `video_save_path` | string | `""` | 自定义保存路径（留空则自动生成） |

---

## 📁 视频保存位置

### 默认路径（推荐）
```
/tmp/follow_test_debug_YYYYMMDD_HHMMSS.avi
```
例如：`/tmp/follow_test_debug_20260528_185402.avi`

### 自定义路径
在launch文件中指定：
```xml
<arg name="video_save_path" default="/home/ucar/debug_videos/my_test_video.avi" />
```

---

## 🚀 使用方法

### 1. 启用视频录制（默认已启用）
```bash
roslaunch flow_end follow_test.launch
```

### 2. 禁用视频录制
```bash
roslaunch flow_end follow_test.launch enable_video_record:=false
```

### 3. 调整帧率
```bash
# 降低帧率减少磁盘占用
roslaunch flow_end follow_test.launch video_fps:=5

# 提高帧率获得更流畅画面
roslaunch flow_end follow_test.launch video_fps:=15
```

### 4. 指定保存路径
```bash
roslaunch flow_end follow_test.launch video_save_path:=/home/ucar/videos/test1.avi
```

---

## ⚙️ 技术参数

- **视频编码**: MJPG (Motion JPEG)
- **视频格式**: AVI
- **分辨率**: 640x480 (RESULT_COL x RESULT_ROW)
- **色彩模式**: 灰度图 (MONO8)
- **默认帧率**: 10 FPS

---

## 📊 磁盘占用估算

| 帧率 | 每小时大小 | 说明 |
|------|-----------|------|
| 5 FPS | ~50 MB | 低占用，适合长时间记录 |
| 10 FPS | ~100 MB | 推荐平衡点 |
| 15 FPS | ~150 MB | 较高流畅度 |
| 30 FPS | ~300 MB | 高占用，仅调试用 |

---

## 🔍 查看视频

### Linux系统
```bash
# 使用vlc播放
vlc /tmp/follow_test_debug_*.avi

# 使用ffplay播放
ffplay /tmp/follow_test_debug_*.avi

# 使用系统默认播放器
xdg-open /tmp/follow_test_debug_20260528_185402.avi
```

### 传输到Windows查看
```bash
# 从机器人拷贝视频
scp ucar@robot_ip:/tmp/follow_test_debug_*.avi D:/videos/
```

---

## 💡 使用建议

### ✅ 推荐场景
- **离线调试**: 录制后离线分析巡线效果
- **问题复现**: 保存异常情况的完整视频
- **算法优化**: 对比不同参数下的巡线表现
- **演示记录**: 记录机器人运行过程

### ⚠️ 注意事项
1. **磁盘空间**: 长时间运行注意监控 `/tmp` 分区空间
2. **性能影响**: 视频写入会占用少量CPU（约3-5%）
3. **帧率选择**: 巡线调试10 FPS已足够，无需过高
4. **定期清理**: 建议定期清理 `/tmp` 目录的旧视频文件

### 🗑️ 清理旧视频
```bash
# 删除3天前的视频
find /tmp -name "follow_test_debug_*.avi" -mtime +3 -delete

# 查看视频占用空间
du -sh /tmp/follow_test_debug_*.avi
```

---

## 🐛 故障排查

### 问题1: 视频文件未生成
**可能原因**:
- `enable_video_record=false`
- `/tmp` 目录无写入权限
- 磁盘空间不足

**解决方法**:
```bash
# 检查参数
rosparam get /follow_test/enable_video_record

# 检查磁盘空间
df -h /tmp

# 手动测试写入权限
touch /tmp/test_video.avi
```

### 问题2: 视频无法播放
**可能原因**:
- 节点异常退出导致视频未正确关闭
- 编码器不支持

**解决方法**:
```bash
# 使用ffmpeg重新编码
ffmpeg -i broken_video.avi -c:v libx264 fixed_video.mp4
```

### 问题3: CPU占用过高
**解决方法**:
```bash
# 降低帧率
roslaunch flow_end follow_test.launch video_fps:=5

# 或禁用视频录制
roslaunch flow_end follow_test.launch enable_video_record:=false
```

---

## 📝 日志信息

运行时会看到以下日志：

### 启动时
```
[INFO] [DEBUG_VIDEO] Started recording to: /tmp/follow_test_debug_20260528_185402.avi
```

### 停止时
```
[INFO] [DEBUG_VIDEO] Video saved to: /tmp/follow_test_debug_20260528_185402.avi
```

### 禁用时
```
[INFO] [DEBUG_VIDEO] Video recording disabled.
```

---

## 🔄 与原功能对比

| 特性 | 原ROS话题发布 | 新视频保存 |
|------|--------------|-----------|
| 实时查看 | ✅ 可用rqt_image_view | ❌ 需等录制完成 |
| 离线分析 | ❌ 需额外录制工具 | ✅ 直接生成视频文件 |
| 磁盘占用 | ❌ 占用ROS bag空间 | ✅ 独立AVI文件 |
| 长期保存 | ❌ 依赖bag录制 | ✅ 自动保存 |
| 性能开销 | 中等 | 较低 |

---

## 🎯 最佳实践

### 调试工作流
1. **启动节点** 时启用视频录制
2. **运行任务** 过程中无需干预
3. **任务结束** 后自动保存视频
4. **离线查看** 视频分析巡线效果
5. **调整参数** 后重新测试对比

### 参数调优示例
```bash
# 第一次测试：默认参数
roslaunch flow_end follow_test.launch

# 查看视频后发现偏左，调整偏置参数
roslaunch flow_end follow_test.launch right_dis_bias_right:=-12.0

# 再次测试对比效果
```

---

## 📞 技术支持

如有问题，请检查：
1. ROS节点日志输出
2. `/tmp` 目录权限和空间
3. OpenCV videoio模块是否正常编译

祝使用愉快！🎉
