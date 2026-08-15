# `speech_command` 语音项目架构说明

> 适用工作区：`/home/ucar/instant_ws`  
> 语言包：`/home/ucar/instant_ws/src/speech_command`  
> 核对日期：2026-08-15  
> 本文描述当前源码和当前比赛启动脚本的真实状态，同时标出遗留接口和已知风险。

## 0. 总览

整个语音系统应当只向业务层暴露两个稳定公共接口：

| 方向 | 话题 | 类型 | 语义 |
|---|---|---|---|
| 识别输出 | `/factory/voice_raw_text` | `std_msgs/String` | ASR 得到的一段完整原始文本；业务层负责判断是否是合法比赛任务 |
| 播报输入 | `/factory/tts_text` | `std_msgs/String` | 任意业务节点提交的待播报中文文本 |

当前代码尚未完全收敛到这两个接口。比赛主流程仍保留旧 `/home/ucar/ucar_ws` 可执行文件、`/question`、`/wakeup`、`fixed_command_iat.py` 和 ASR 启停服务配置。本文把“目标公共接口”和“当前真实启动链”分开描述，避免误认为迁移已经完成。

当前存在三种入口：

1. `bash /home/ucar/instant_ws/src/run_competition.sh`：当前比赛包装脚本，组合旧 ASR、当前 TTS、Python IAT 和比赛业务。
2. `roslaunch ucar_2026_competition full_competition.launch`：直接比赛 launch；默认也会启用旧外部 ASR 链。
3. `roslaunch speech_command speech_command.launch`：语言包独立链，启动 `cloud_asr_test2` 和 Spark LLM；它不是当前比赛包装脚本使用的 ASR 链。

---

# 第一部分：语音识别（ASR）

## 1.1 推荐对外接口

### `/factory/voice_raw_text`

| 项目 | 内容 |
|---|---|
| 类型 | `std_msgs/String` |
| 数据 | 一段 ASR 文本，例如完整的实体环境/仿真环境双类别任务 |
| 当前发布者 | `cloud_asr_test2`；完整模式下的 `speech_command_node` 也会发布 |
| 当前订阅者 | `speech_command/scripts/spark_llm_node.py`、包内其他 Spark 实验节点、部分 legacy voice bridge |
| 业务约束 | ASR 只发布原话，不应负责推进任务；业务层应做完整性、类别合法性和去重检查 |

同一系统组合中只能有一个真实 ASR 发布者，避免同一句话被多次发布。

### `/factory/voice_command_accepted`

| 项目 | 内容 |
|---|---|
| 类型 | `std_msgs/Bool` |
| 方向 | 业务层/桥接层 → `cloud_asr_test2` |
| 语义 | `true` 表示业务层已经接受一条完整比赛任务，ASR 可以停止 |
| 状态 | 仅用于 `cloud_asr_test2` 的持续监听生命周期，不是统一业务文本接口 |

### `/factory/task_state`

`cloud_asr_test2.cpp` 创建了 `std_msgs/Int32` 发布者，但当前文件没有实际 `publish()` 调用。因此它现在只是未完成/预留接口，不能作为可靠状态源。

## 1.2 当前比赛正式启动链

### 入口 A：`run_competition.sh`

当前脚本实际执行：

```text
/home/ucar/instant_ws/src/run_competition.sh
  ├─ 启动 roscore
  ├─ 启动旧 ASR 可执行文件
  │    /home/ucar/ucar_ws/devel/lib/speech_command/speech_command_node
  ├─ 启动当前工作区 TTS-only 可执行文件
  │    /home/ucar/instant_ws/devel/lib/speech_command/speech_command_node
  │    _tts_only:=true _tts_topic:=/factory/tts_text
  ├─ 启动 fixed_command_iat.py
  └─ roslaunch ucar_2026_competition full_competition.launch
       start_external_voice:=false
```

这里把 `start_external_voice` 设为 `false`，是因为脚本已经在 roslaunch 外面自行启动旧 ASR、当前 TTS 和 `fixed_command_iat.py`，避免 launch 再启动一套。

按当前代码设计，识别数据链为：

```text
旧 speech_command_node
  → 麦克风 PCM（期望话题 /speech_command_node/audio_pcm）
  → fixed_command_iat.py
  → 讯飞 WebSocket IAT
  → /competition/iat_text（过程/诊断文本）
  → /question（仅完整双类别任务）
  → competition_flow.py
```

同时，业务层还等待：

```text
/wakeup
  → competition_flow.py::_wakeup_cb()
  → 播报“我在”
  → 调用 start_listening 服务
  → 等待 /question
  → 调用 stop_listening 服务
```

### 入口 B：直接启动 `full_competition.launch`

`full_competition.launch` 的 `start_external_voice` 默认是 `true`。它传给 `common_core.launch` 后会启动：

- `external_voice_nodes.py`
- `fixed_command_iat.py`
- 当前 `speech_command_node` 的 `tts_only` 模式
- `ucar_2026_competition_speech` 播报网关
- 比赛 LLM 与业务流程

`external_voice_nodes.py` 不做识别，它只是：

1. 找到 `/home/ucar/ucar_ws`。
2. 直接启动旧 `devel/lib/speech_command/speech_command_node`。
3. 单独把旧 `speech_command` 包加入子进程 `ROS_PACKAGE_PATH`，避免 source 整个旧工作区带来重复包。
4. 监控子进程，并在 ROS 关闭时结束它。

## 1.3 语言包独立 ASR 链：`cloud_asr_test2`

`speech_command/launch/speech_command.launch` 当前启动：

```text
cloud_asr_test2（C++ ASR）
  + spark_llm_node.py（Python 大模型）
```

`cloud_asr_test2` 的内部流程：

```text
ALSA 设备 hw:3,0
  → 16 kHz / S16_LE / 单声道 PCM
  → AIUI CMD_WRITE 音频流
  → CloudTestListener::onEvent(EVENT_RESULT)
  → 合并当前识别文本
  → /factory/voice_raw_text
  → 等待 /factory/voice_command_accepted=true
  → 停止录音并销毁 AIUI agent
```

关键行为：

- 强制把 AIUI 设置为连续交互、用户音频源并关闭 SDK 内部唤醒。
- 识别出非空文本后发布 `/factory/voice_raw_text`。
- 默认不会因无关语句或不完整语句自动退出；业务确认后才关闭。
- 同一个进程也订阅 `/factory/tts_text`，所以它同时包含一个独立云 TTS 后端。
- 代码中存在敏感凭据硬编码，文档不记录具体值；后续应迁移到受权限保护的配置或环境变量。

## 1.4 原始硬件节点：`speech_command_node`

编译产物：

```text
/home/ucar/instant_ws/devel/lib/speech_command/speech_command_node
```

主要源码：

```text
src/aiuiMain.cpp
src/AIUITester.cpp
src/AudioRecorder.cpp
src/AudioPlayer.cpp
```

非 `tts_only` 模式的识别流程：

```text
speech_command_node
  → 打开 /dev/ttyS3（115200）
  → AIUITester 创建 AIUI agent
  → AudioRecorder 连接 HID 麦克风阵列
  → AIUI EVENT_WAKEUP / EVENT_VAD / EVENT_RESULT
  → AIUITester 将文本写入全局 question / answer
  → aiuiMain.cpp::data_send()
       ├─ /question
       ├─ /answer
       ├─ /angle
       └─ /factory/voice_raw_text
```

它同时包含：

- 硬件/软件唤醒处理。
- 在线和离线 AIUI 识别结果解析。
- 本地问答表 `config/offline_QA.txt` 的查找。
- 串口和麦克风阵列控制。
- AIUI 原厂 TTS。

当前 `AIUITester.cpp` 在唤醒事件和其他位置仍直接调用 `/home/ucar/ucar_ws/src/speech_command/start.sh`，并存在旧工作区动态库路径。这意味着即使从 `instant_ws` 编译和启动该节点，完整模式也没有完全摆脱旧工作区。

## 1.5 Python IAT 和业务解析

### `ucar_2026_competition/scripts/fixed_command_iat.py`

职责：

- 订阅 `/speech_command_node/audio_pcm`（`std_msgs/UInt8MultiArray`）。
- 将 PCM 通过讯飞 WebSocket IAT 发送到云端。
- 合并增量识别片段。
- 将过程文本发布到 `/competition/iat_text`。
- 只有识别出两个不同且完整的比赛类别时，才发布 `/question`。

它是实际 ASR 解码器，不是简单话题桥。

### `ucar_2026_competition/scripts/competition_flow.py`

当前语音入口：

- 订阅 `/wakeup`。
- 订阅 `/question`。
- 通过 `_question_cb()` 解析实体类别和仿真类别。
- 拒绝非监听窗口、不完整类别或两个相同类别的指令。
- 配置中还声明 `/speech_command_node/start_listening` 和 `/speech_command_node/stop_listening`。

### `speech_command/scripts/spark_llm_node.py`

它不是 ASR 节点。它订阅：

- `/factory/voice_raw_text`
- `/factory/qr_item`

然后调用 Spark LLM，并发布：

- `/factory/tts_text`
- `/factory/target_warehouses`

包内同时存在 `scripts/spark_llm_node.py` 和 `src/spark_llm_node.py`，`rosrun` 会报告同名可执行文件不唯一。两份实现逻辑也不同，不能视为同一个文件。

## 1.6 识别侧遗留接口与缺口

| 接口/依赖 | 当前状态 | 风险 |
|---|---|---|
| `/home/ucar/ucar_ws` | `run_competition.sh`、`external_voice_nodes.py`、`AIUITester.cpp` 仍引用 | 旧二进制和当前源码可能不一致；部署必须保留两个工作区 |
| `/question` | `speech_command_node` 和 `fixed_command_iat.py` 会发布；业务层订阅 | 与 `/factory/voice_raw_text` 双轨并存，容易重复或走错入口 |
| `/wakeup` | `competition_flow.py` 订阅；当前 `speech_command` 包内没有统一发布者 | 依赖旧二进制或 legacy bridge 的隐含行为 |
| `/speech_command_node/audio_pcm` | `fixed_command_iat.py` 订阅 | 在已检查的当前和旧 `speech_command` 源码中未找到明确发布代码；运行二进制可能与源码不同 |
| `start_listening` / `stop_listening` | 业务配置和流程调用 | 当前 `instant_ws/src/speech_command` 没有对应服务实现 |
| `/factory/voice_raw_text` | 新接口已经存在 | 正式比赛主链仍未直接消费它 |

---

# 第二部分：语音播报（TTS）

## 2.1 统一对外接口

### `/factory/tts_text`

| 项目 | 内容 |
|---|---|
| 类型 | `std_msgs/String` |
| 方向 | 所有业务节点 → 唯一 TTS 播放节点 |
| 数据 | 要播报的完整中文文本 |
| 约束 | 业务层只发布文本，不应直接操作 PCM 文件、声卡或讯飞 SDK |

标准逻辑链：

```text
比赛流程 / LLM / OCR / 二维码 / 交通灯 / 巡线
  → /factory/tts_text
  → 唯一 TTS 消费者
  → 讯飞 TTS 合成
  → PCM/音频流
  → ALSA/AudioPlayer
  → 扬声器
```

同一系统组合中最多启动一个 TTS 消费者，否则同一句话会被重复播放。

## 2.2 TTS 后端 A：`speech_command_node` 的 AIUI `gTTS`

入口：

```text
/factory/tts_text
  → aiuiMain.cpp::xunfei_llm_tts_callback()
  → AIUITester.cpp::gTTS()
  → AIUI CMD_TTS
  → AIUI TTS 数据事件
  → RingBuffer
  → AudioPlayer
  → ALSA default 设备
```

参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `~tts_only` | `false` | `true` 时只订阅 TTS，不执行完整 ASR/串口流程 |
| `~tts_topic` | `/factory/tts_text` | TTS 文本输入话题；仅 `tts_only` 分支使用该变量 |

完整模式还使用异步 spinner，使 ROS 回调可以在 AIUI 主测试线程运行时处理 TTS。

当前实现注意事项：

- `tts_only` 分支在创建 `AIUITester`/AIUI agent 之前就进入 `ros::spin()`。
- `gTTS()` 只有在 `globalAgent != NULL` 时才真正发送 `CMD_TTS`。
- 因此当前 `tts_only` 代码存在“收到话题但后端未初始化、实际不出声”的风险，必须通过实车验证。
- 当前没有串行播报队列，也没有真实“播放完成”事件对外发布。

## 2.3 TTS 后端 B：`cloud_asr_test2` 的 Python 云 TTS

入口：

```text
/factory/tts_text
  → cloud_asr_test2.cpp::ttsCallback()
  → speakText()
  → python3 xf_tts_stable.py
  → tmp/tts_result.pcm
  → aplay -r 16000 -f S16_LE -c 1
  → 扬声器
```

特点：

- 合成和播放通过同步 `system()` 调用完成；回调会阻塞到播放结束。
- PCM 使用固定路径 `tmp/tts_result.pcm`。
- 该实现和 ASR 位于同一个 `cloud_asr_test2` 进程，不是独立纯 TTS 节点。
- Python 脚本路径和 PCM 路径写死在当前工作区。
- 没有公开完成话题，也没有显式多请求队列。

## 2.4 播报请求的主要来源

### 比赛播报网关

文件：

```text
ucar_2026_competition_speech/scripts/competition_announcer.py
```

它把比赛事件模板转换为文本，再发布 `/factory/tts_text`。其附加接口为：

| 接口 | 类型 | 用途 |
|---|---|---|
| `/competition_speech/request` | `std_msgs/String`（JSON） | 请求一次模板播报 |
| `/competition_speech/status` | `std_msgs/String`（JSON） | `speaking`/`completed` 估算状态 |
| `/competition_speech/completed` | `std_msgs/String` | 发布完成的事件名 |
| `/competition_speech/announce` | `ucar_2026_competition_speech/Announce` | 同步服务式播报请求 |

它具有文本去重、等待 TTS 订阅者和按字数估算阻塞时长的能力。但是 `completed` 表示估算时间到达，不是声卡或 TTS 后端返回的真实播放完成事件。

### 语言包内业务桥

`scripts/follow_tts_bridge.py`：

- 订阅 `/follow_begin`。
- 订阅 `/follow_end`。
- 可选订阅交通灯决策话题。
- 做简单防抖和固定中文模板映射。
- 发布 `/factory/tts_text`。

### 其他发布者

当前工作区中以下业务也会向 `/factory/tts_text` 发布：

- `ucar_2026_smart_factory_llm`：LLM 推理结果。
- `ucar_2026_qr_speak_test`：二维码语音测试。
- `factory_sign_ocr_test`、`factory_sign_ppocr_test`、`factory_sign_ppocr_rknn_test`：工厂标牌识别。
- `traffic_light_vision`、`flow_end`：交通灯和巡线事件。
- `yolo` 测试链。
- `speech_command` 内 Spark/LLM 实验脚本。

这些都是文本生产者，不应该各自启动第二个 TTS 消费者。

## 2.5 播报侧生命周期和并发现状

当前系统没有统一的后端完成确认协议：

- `/factory/tts_text` 只表达“请求播报”，不表示开始或完成。
- `competition_announcer.py` 的完成状态来自字数估算。
- `speech_command_node` 没有 TTS 队列、完成事件或完成话题。
- `cloud_asr_test2` 通过阻塞调用自然串行，但没有对外完成信号。
- 如果同时启动 `speech_command_node` 和 `cloud_asr_test2`，二者都会订阅 `/factory/tts_text`，会重复播报。

---

# 第三部分：文件与目录职责索引

## 3.1 `speech_command` 包核心文件

| 路径 | 大致功能 |
|---|---|
| `CMakeLists.txt` | 编译 `speech_command_node`、`cloud_asr_test`、`cloud_asr_test2` 和音频/AIUI 动态库；目前只安装 `follow_tts_bridge.py` |
| `package.xml` | ROS 包元数据和 `roscpp`、`rospy`、`std_msgs`、`roslib`、`serial` 依赖 |
| `launch/speech_command.launch` | 独立启动 `cloud_asr_test2` 与 `spark_llm_node.py`；不是当前比赛包装脚本的 ASR 入口 |
| `src/aiuiMain.cpp` | `speech_command_node` 主入口；发布识别文本、订阅 TTS、启动 AIUITester/数据发布线程并提供测试服务 |
| `src/AIUITester.cpp` | AIUI agent 生命周期、唤醒/VAD/ASR/TTS 事件处理、在线/离线识别解析和 `gTTS()` |
| `include/AIUITester.h` | `AIUITester`、`gTTS()` 和测试回调声明 |
| `include/Global.h` | 旧实现的全局状态、音频路径、串口参数、识别结果缓冲和配置开关 |
| `src/cloud_asr_test2.cpp` | 当前独立连续云 ASR；发布 `/factory/voice_raw_text`，订阅确认和 TTS；直接采集 ALSA PCM |
| `src/cloud_asr_test.cpp` | 较早的云 ASR 独立诊断工具 |
| `src/AudioRecorder.cpp` / `include/AudioRecorder.h` | 通过 HID 麦克风阵列开始/停止降噪和原始录音、设置录音角度 |
| `src/AudioPlayer.cpp` / `include/AudioPlayer.h` | ALSA PCM 播放和缓冲清理 |
| `src/TestListener.cpp` / `include/TestListener.h` | AIUI SDK 通用事件监听和 TTS 数据文件辅助处理 |
| `src/WriteAudioThread.cpp` / `include/WriteAudioThread.h` | 将测试 PCM 文件分块写入 AIUI agent，主要用于离线/文件诊断 |
| `src/FileUtil.cpp` / `include/FileUtil.h` | 配置、PCM 和日志文件读写工具 |
| `src/serial_port.cpp` | 较旧的底层 UART 打开、配置、收发辅助函数 |
| `scripts/follow_tts_bridge.py` | 巡线/交通灯状态转中文播报文本 |
| `scripts/spark_llm_node.py` | 语音原文和二维码数据合流后调用 Spark LLM；发布 TTS 和目标车间 |
| `src/spark_llm_node.py` | 另一版 Spark 状态机实现；与 `scripts/` 同名，存在运行选择歧义 |
| `src/spark_llm_node_old.py` | 更早的 LLM 触发版实现，保留作历史/实验 |
| `src/LLMBridge.py` | 旧实验桥，订阅 `/speech_to_text` 并生成 TTS；不属于当前正式接口 |
| `src/online_spark_x2.py` | Spark HTTP 调用实验代码 |
| `src/tts_vertiy.py`、`src/ttx_test.py` | TTS 验证/监听实验脚本，不属于正式启动链 |
| `xf_tts_stable.py` | WebSocket 云 TTS 合成脚本，写入固定 PCM 文件；由 `cloud_asr_test2` 调用 |
| `start.sh` / `start1.sh` | 旧的唤醒后业务/导航启动脚本；仍带旧架构痕迹 |

## 3.2 配置、资源和第三方文件

| 路径 | 大致功能 |
|---|---|
| `config/AIUI/cfg/aiui.cfg` | AIUI 主配置，包括 VAD、唤醒、ASR 和 TTS 资源路径 |
| `config/AIUI/` | AIUI 的 VAD、唤醒、ASR、TTS、xTTS 模型和资源 |
| `config/call.bnf`、`config/gm_continuous_digit.abnf` | 离线语法/数字识别语法 |
| `config/offline_QA.txt` | 本地固定问答映射 |
| `config/userwords.txt` | 用户词表 |
| `config_old/` | 旧工作区配置备份，不应作为当前运行配置 |
| `include/aiui/`、`include/json*`、其他 SDK 头文件 | 第三方 AIUI/JSON/MSC/HID/USB SDK 头文件 |
| `lib/arm64/`、`lib/arm32/`、`lib/x64/`、`lib/x86/`、`libs/` | 不同架构的讯飞、HID、MSC、USB 等预编译库 |
| `audio/` | 唤醒、离线回答和测试音频 |
| `tmp/` | 运行期 PCM 和临时数据；不是源码 |
| `bin/`、`msc/`、`AIUI/` | SDK 运行数据、缓存、日志和配置副本 |

## 3.3 语言包之外但直接影响语音链的文件

| 路径 | 大致功能 |
|---|---|
| `src/run_competition.sh` | 当前比赛总启动包装；显式组合旧 ASR、当前 TTS、Python IAT 和正式 launch |
| `ucar_2026_competition/launch/common_core.launch` | 按参数启动外部 ASR、fixed IAT、TTS、播报网关和 LLM |
| `ucar_2026_competition/launch/full_competition.launch` | 正式全流程入口；`start_external_voice` 默认 `true` |
| `ucar_2026_competition/scripts/external_voice_nodes.py` | 旧 `/home/ucar/ucar_ws` ASR 可执行文件的启动/看护器 |
| `ucar_2026_competition/scripts/fixed_command_iat.py` | PCM → 讯飞 IAT → `/question` 的 Python 识别器 |
| `ucar_2026_competition/scripts/competition_flow.py` | 订阅 `/wakeup`、`/question` 并推进任务状态机 |
| `ucar_2026_competition/config/competition.yaml` | 唤醒回复、任务回复和 ASR 启停服务名等业务参数 |
| `ucar_2026_competition_speech/scripts/competition_announcer.py` | 比赛播报模板、去重、等待和估算完成网关 |
| `ucar_2026_competition_speech/scripts/speech_templates.py` | Task1–Task5、交通灯等固定播报模板 |

---

# 第四部分：启动组合规则与检查方法

## 4.1 互斥规则

一次只能选择一个 ASR 后端：

- 旧 `/home/ucar/ucar_ws/.../speech_command_node` + `fixed_command_iat.py`；或
- 当前 `cloud_asr_test2`；或
- 当前 `speech_command_node` 完整模式。

一次只能选择一个 TTS 后端：

- 当前 `speech_command_node`；或
- `cloud_asr_test2` 内置的云 TTS。

Spark LLM 也只能启动一个同名实现，必须消除 `scripts/` 与 `src/` 的选择歧义。

## 4.2 运行时确认命令

```bash
rosnode list | grep -E 'speech|voice|iat|tts|spark'
rostopic info /factory/voice_raw_text
rostopic info /factory/tts_text
rostopic info /question
rostopic info /wakeup
rosservice list | grep speech_command_node
```

检查目标：

- `/factory/voice_raw_text` 最多一个发布者。
- `/factory/tts_text` 可以有多个业务发布者，但最多一个真正播放订阅者。
- 正式完成统一后，业务目录不应再出现 `/question`、`/wakeup` 或 `/home/ucar/ucar_ws`。

## 4.3 当前结论

当前公共接口方向已经明确：

```text
识别：/factory/voice_raw_text
播报：/factory/tts_text
```

但现有正式比赛链仍是混合架构：

```text
旧工作区负责部分底层语音
  + 当前工作区负责业务、TTS 和部分 ASR 实验
  + /question、/wakeup 继续承担正式业务握手
```

因此本文是“当前真实架构说明”，不是“已经完成统一后的架构说明”。后续若实施完全迁移，应以两个公共话题为边界，删除旧可执行文件启动器、固定 IAT 桥、legacy 话题和业务层麦克风生命周期控制。
