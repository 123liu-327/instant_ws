/**
 * @brief 云端ASR链路独立测试工具 (SDK模式)
 *
 * 用途：绕过唤醒/串口/状态机，让SDK自行管理音频，直接验证 AIUI 云端 ASR
 *
 * 与 user 模式的区别：data_source="sdk" 让 AIUI SDK 内部打开 ALSA 设备，
 * 自行采集音频、做 VAD、送云端 ASR。我们只需保持进程存活，等回调即可。
 *
 * 使用方法：
 *   cd ~/instant_ws
 *   catkin_make --pkg speech_command
 *   rosrun speech_command cloud_asr_test
 *
 * 预期行为：
 *   - 启动后自动进入持续识别（不需要唤醒词）
 *   - 对着麦克风说话，终端应实时打印识别结果
 *   - Ctrl+C 退出
 */

#include <iostream>
#include <string>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include "aiui/AIUI.h"
#include "AIUITester.h"
#include "FileUtil.h"
#include "jsoncpp/json/json.h"

using namespace std;

// ─── 全局变量 ───
static volatile bool g_running = true;
static IAIUIAgent* g_agent = nullptr;

// ─── 信号处理 ───
void sig_handler(int sig) {
    cout << "\n>>>>> 收到退出信号，正在停止..." << endl;
    g_running = false;
}

// ─── 事件监听器：只关注识别结果和错误 ───
class CloudTestListener : public IAIUIListener {
public:
    void onEvent(const IAIUIEvent& event) const {
        switch (event.getEventType()) {
            case AIUIConstant::EVENT_CONNECTED_TO_SERVER: {
    cout << "🌐 已连接到AIUI服务器" << endl;
    break;
}

case AIUIConstant::EVENT_SERVER_DISCONNECTED: {
    cout << "❌ 与AIUI服务器断开连接" << endl;
    break;
}
        case AIUIConstant::EVENT_STATE: {
            switch (event.getArg1()) {
            case AIUIConstant::STATE_IDLE:
                cout << "[STATE] IDLE" << endl;
                break;
            case AIUIConstant::STATE_READY:
                cout << "[STATE] READY - 可以开始说话了" << endl;
                break;
            case AIUIConstant::STATE_WORKING:
                cout << "[STATE] WORKING - 正在识别中..." << endl;
                break;
            }
            break;
        }

        case AIUIConstant::EVENT_CMD_RETURN:
{
    printf(
        "\n[CMD_RETURN] cmd=%d ret=%d info=%s\n",
        event.getArg1(),
        event.getArg2(),
        event.getInfo());

    break;
}
        case AIUIConstant::EVENT_VAD: {
            if (event.getArg1() == AIUIConstant::VAD_BOS) {
                cout << "[VAD] >>> 检测到语音开始" << endl;
            } else if (event.getArg1() == AIUIConstant::VAD_EOS) {
                cout << "[VAD] <<< 检测到语音结束" << endl;
            }
            break;
        }

        case AIUIConstant::EVENT_RESULT: {
            Json::Value bizParamJson;
            Json::Reader reader;
            if (!reader.parse(event.getInfo(), bizParamJson, false)) {
                cout << "[ERROR] 解析结果JSON失败" << endl;
                break;
            }

            Json::Value data = (bizParamJson["data"])[0];
            Json::Value params = data["params"];
            Json::Value content = (data["content"])[0];
            string sub = params["sub"].asString();

            if (sub == "iat") {
                // 听写结果（ASR转文字）
                Json::Value empty;
                Json::Value contentId = content.get("cnt_id", empty);
                if (contentId.empty()) break;

                string cnt_id = contentId.asString();
                int dataLen = 0;
                const char* buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
                if (buffer != nullptr && dataLen > 0) {
                    string resultStr(buffer, dataLen);
                    cout << "[ASR原始结果] " << resultStr << endl;

                    // 尝试解析出文字
                    Json::Value resultJson;
                    Json::Reader resultReader;
                    if (resultReader.parse(resultStr, resultJson)) {
                        // 拼接识别文字
                        string text = "";
                        if (resultJson.isMember("ws")) {
                            int wsCount = resultJson["ws"].size();
                            for (int i = 0; i < wsCount; i++) {
                                if (resultJson["ws"][i].isMember("cw")) {
                                    int cwCount = resultJson["ws"][i]["cw"].size();
                                    for (int j = 0; j < cwCount; j++) {
                                        text += resultJson["ws"][i]["cw"][j]["w"].asString();
                                    }
                                }
                            }
                        }
                        if (!text.empty()) {
                            cout << "✅ [识别文字] " << text << endl;
                        }
                    }
                }
            } else if (sub == "nlp") {
                // 语义理解结果
                Json::Value empty;
                Json::Value contentId = content.get("cnt_id", empty);
                if (contentId.empty()) break;

                string cnt_id = contentId.asString();
                int dataLen = 0;
                const char* buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
                if (buffer != nullptr && dataLen > 0) {
                    string resultStr(buffer, dataLen);
                    cout << "[NLP原始结果] " << resultStr << endl;

                    Json::Value resultJson;
                    Json::Reader resultReader;
                    if (resultReader.parse(resultStr, resultJson)) {
                        if (resultJson.isMember("intent") && resultJson["intent"].isMember("text")) {
                            cout << "✅ [语义问题] " << resultJson["intent"]["text"].asString() << endl;
                        }
                        if (resultJson.isMember("intent") && resultJson["intent"].isMember("answer")) {
                            cout << "✅ [语义回答] " << resultJson["intent"]["answer"]["text"].asString() << endl;
                        }
                    }
                }
            } else {
                cout << "[RESULT] sub=" << sub << endl;
            }
            break;
        }

        case AIUIConstant::EVENT_ERROR: {
    cout << "\n==========================" << endl;
    cout << "❌ AIUI ERROR" << endl;
    cout << "code = " << event.getArg1() << endl;
    cout << "info = " << event.getInfo() << endl;
    cout << "==========================\n" << endl;
    break;
}

        

        default:
            break;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_asr_test");
    ros::NodeHandle nh;

    signal(SIGINT, sig_handler);

    // 获取包路径
    string pkg_path = ros::package::getPath("speech_command");
    string cfg_path = pkg_path + "/config/AIUI/cfg/aiui.cfg";

    cout << "========================================" << endl;
    cout << "  云端 ASR 链路独立测试工具" << endl;
    cout << "========================================" << endl;
    cout << "配置文件: " << cfg_path << endl;

    // 读取配置
    string fileParam = FileUtil::readFileAsString(cfg_path);
    if (fileParam.empty()) {
        cout << "❌ 无法读取配置文件!" << endl;
        return -1;
    }

    // 强制覆盖关键参数，确保纯净的云端测试环境
    Json::Value paramJson;
    Json::Reader reader;
    if (!reader.parse(fileParam, paramJson, false)) {
        cout << "❌ 配置文件JSON解析失败!" << endl;
        return -1;
    }

    // 强制设置：关闭唤醒、持续交互、云端识别
    paramJson["speech"]["wakeup_mode"] = "off";
    paramJson["speech"]["interact_mode"] = "continuous";
    paramJson["speech"]["data_source"] = "user";
    paramJson["speech"]["intent_engine_type"] = "cloud";
    paramJson["iat"]["engine_type"] = "cloud";
    paramJson["iat"]["sample_rate"] = "16000";

    Json::FastWriter writer;
    string paramStr = writer.write(paramJson);
    cout << "\n========== 最终AIUI配置 ==========" << endl;
cout << paramStr << endl;
cout << "=================================\n" << endl;

    cout << "wakeup_mode: off (已强制)" << endl;
    cout << "interact_mode: continuous (已强制)" << endl;
    cout << "engine_type: cloud (已强制)" << endl;
    cout << "----------------------------------------" << endl;

    // 创建监听器和Agent
    static CloudTestListener listener;
    g_agent = IAIUIAgent::createAgent(paramStr.c_str(), &listener);
    if (g_agent == nullptr) {
        cout << "❌ AIUI Agent 创建失败!" << endl;
        return -1;
    }
    cout << "✅ AIUI Agent 创建成功" << endl;

    // ⚡ 关键：在 data_source=user 模式下，必须发送 CMD_START 启动识别引擎
    {
        IAIUIMessage* startMsg = IAIUIMessage::create(AIUIConstant::CMD_START, 0, 0, "", nullptr);
        g_agent->sendMessage(startMsg);
        startMsg->destroy();
        cout << ">>>>> 已发送 CMD_START，等待引擎就绪..." << endl;
        sleep(1);  // 给SDK时间初始化云端连接
    }

    IAIUIMessage* startRecord =
    IAIUIMessage::create(
        
        AIUIConstant::CMD_START_RECORD,
        0,
        0,
        "",
        nullptr);

g_agent->sendMessage(startRecord);
startRecord->destroy();

cout << ">>>>> 已发送 CMD_START_RECORD" << endl;

IAIUIMessage* wakeupMsg =
    IAIUIMessage::create(
        AIUIConstant::CMD_WAKEUP,
        0,
        0,
        "",
        nullptr);

g_agent->sendMessage(wakeupMsg);
wakeupMsg->destroy();

cout << ">>>>> 已发送 CMD_WAKEUP" << endl;
    // 打开麦克风
    snd_pcm_t* capture_handle = nullptr;
    snd_pcm_hw_params_t* hw_params = nullptr;
    unsigned int rate = 16000;
    int err;

    // 使用与主程序相同的设备名
    const char* pcm_device = "hw:XFMDPV0018";

    if ((err = snd_pcm_open(&capture_handle, pcm_device, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        // 如果专用名称失败，尝试通用设备
        pcm_device = "hw:2,0";
        if ((err = snd_pcm_open(&capture_handle, pcm_device, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
            printf("❌ 无法打开音频设备 (%s)\n", snd_strerror(err));
            g_agent->destroy();
            return -1;
        }
    }
    cout << "✅ 音频设备打开成功: " << pcm_device << endl;

    // 配置音频参数
    snd_pcm_hw_params_malloc(&hw_params);
    snd_pcm_hw_params_any(capture_handle, hw_params);
    snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, 0);
    snd_pcm_hw_params_set_channels(capture_handle, hw_params, 1);
    snd_pcm_hw_params(capture_handle, hw_params);
    snd_pcm_hw_params_free(hw_params);
    snd_pcm_prepare(capture_handle);

    int buffer_frames = 640;
    int frame_byte = 2; // 16bit = 2 bytes
    char* audio_buf = (char*)malloc(buffer_frames * frame_byte);

    cout << "✅ 音频参数配置完成 (16kHz, 16bit, mono)" << endl;
    cout << "========================================" << endl;
    cout << "🎤 请对着麦克风说话，Ctrl+C 退出" << endl;
    cout << "========================================" << endl;

    // 主循环：采集音频 → 写入SDK
    static int frame_count = 0;
    while (g_running && ros::ok()) {
        err = snd_pcm_readi(capture_handle, audio_buf, buffer_frames);
        short* pcm = (short*)audio_buf;

long sum = 0;
for(int i=0;i<buffer_frames;i++)
{
    sum += abs(pcm[i]);
}

long avg = sum / buffer_frames;

cout << "音量=" << avg << endl;
        if (err != buffer_frames) {
            if (err < 0) {
                printf("⚠️ 音频读取错误: %s\n", snd_strerror(err));
                snd_pcm_prepare(capture_handle);
            }
            continue;
        }

        // 每50帧打印一次调试信息，确认音频正在被采集和发送
        if (++frame_count % 50 == 0) {
            cout << "[DEBUG] 已发送 " << frame_count << " 帧音频数据" << endl;
        }

        Buffer* buffer = Buffer::alloc(buffer_frames * frame_byte);
        memcpy(buffer->data(), audio_buf, buffer_frames * frame_byte);
        IAIUIMessage* writeMsg = IAIUIMessage::create(
            AIUIConstant::CMD_WRITE, 0, 0,
            "data_type=audio,sample_rate=16000", buffer);
        g_agent->sendMessage(writeMsg);
        writeMsg->destroy();

        ros::spinOnce();
    }

    // 清理
    cout << "\n>>>>> 正在清理资源..." << endl;
    free(audio_buf);
    snd_pcm_close(capture_handle);
    if (g_agent) {
        g_agent->destroy();
        g_agent = nullptr;
    }
    cout << "✅ 测试结束" << endl;
    return 0;
}
