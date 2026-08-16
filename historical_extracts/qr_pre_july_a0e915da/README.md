# 7月前原始二维码链路提取

- GitHub 仓库：`123liu-327/instant_ws`
- 基线提交：`a0e915daa86998e0661b10c43996fdbbef045029`
- 提交时间：2026-06-14 22:25:42 +0800
- 提交标题：`改动三`
- 选择规则：`origin/main` 在 2026-07-01 00:00:00 +0800 之前的最后提交

本目录由 Git 历史直接提取，未从当前工作树复制，也没有覆盖当前 `src`。

## 核心包

### `src/test`

原始二维码解码包，ROS 包名为 `test`。

- `src/qr_node.cpp`
  - 节点名：`qr_code_scanner`
  - 订阅：`/ucar_camera/image_raw`
  - 等待：`/qr_node_start` 收到 `start!`
  - 图像转换：`cv_bridge` 转 BGR8，再转灰度
  - 解码器：C++ ZBar，仅启用二维码
  - 发布：`/qr_code_result`，`std_msgs/String`
- `src/qr_url_parser.py`
  - 节点名：`qr_url_parser`（anonymous）
  - 订阅：`/qr_code_result`
  - 对 URL 执行 HTTP GET，超时 10 秒
  - 发布：`/qr_url_parsed`，JSON 字符串
- `src/testbe_node.cpp`
  - 持续向 `/qr_node_start` 发布 `start!`
- `CMakeLists.txt`
  - 构建 `qr_node`
  - 依赖 OpenCV、ZBar、cv_bridge、image_transport

### `src/ucar_camera`

原始相机输入包，提供 `/ucar_camera/image_raw`。它是原始 `qr_node` 的直接上游。

## 业务接入上下文

- `src/ucar_controller/scripts/process_qr.py`
- `src/ucar_controller/scripts/process_qr_test.py`
- `src/ucar_controller/scripts/process_qr_test1.py`
- `src/ucar_controller/scripts/performance_test.py`
- `src/ucar_nav/launch/ucar_navigation.launch`
- `temp/image_start.sh`

这些文件展示了原始旋转、URL 去重、解析结果汇总、导航接入和启动方式。它们不是一个可独立构建的新包，保留仅用于历史对照和后续复用。

## 原始话题链

```text
/ucar_camera/image_raw
        -> test/qr_node
        -> /qr_code_result
        -> test/qr_url_parser.py
        -> /qr_url_parsed
        -> ucar_controller 中的扫码业务脚本
```

## 注意

- 不要把本目录的 `src/test` 直接复制回当前工作区 `src`，否则可能与现有包或节点冲突。
- 原始节点依赖系统 ZBar 开发库，并且 `qr_node.cpp` 默认调用 OpenCV 窗口显示。
- 该版本没有关键帧选择、CLAHE、多尺度增强、有界队列、`/qr_decoder/status` 或 `/qr_code_data` 新接口。
- 如需复用，建议只比较算法和话题链，不直接替换当前正式节点。
