#include <iostream>
#include <string>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include "aiui/AIUI.h"
#include "AIUITester.h"
#include "FileUtil.h"
#include "jsoncpp/json/json.h"

using namespace std;
using namespace aiui; // 显式引入讯飞命名空间

static volatile bool g_running = true;
static IAIUIAgent* g_agent = nullptr;

// ─── 讯飞开放平台 精准鉴权密钥 ───
const string XF_APPID      = "4c8a2ec8";
const string XF_API_KEY    = "uFtKQJrDyzpKyiVHMAWM";
const string XF_API_SECRET = "btlLJzACqGeZKYFoYQeF";

void sig_handler(int sig) {
    cout << "\n>>>>> 收到退出信号，正在停止..." << endl;
    g_running = false;
}

/**
 * @brief 动态生成 Python 语音合成脚本并即时调用播放
 * 所有生成的文件严格存放在当前 ROS 包路径下，不涉及 /tmp
 */
void speakText(const string& text) {
    if (text.empty()) return;
    cout << "🤖 [比赛指令-核心TTS] 正在请求讯飞开放平台流式合成: \"" << text << "\"" << endl;

    // 获取当前 ROS 包的绝对路径
    string pkg_path = ros::package::getPath("speech_command");
    string py_path = pkg_path + "/xf_tts_stable.py";
    string pcm_path = pkg_path + "/tmp/tts_result.pcm";

    // 先清理上一次的缓存
    unlink(pcm_path.c_str());

    // 在当前包路径下生成测试通过的完美 Python 脚本
    ofstream pyScript(py_path.c_str());
    if (!pyScript.is_open()) {
        cerr << "❌ 无法在包路径下创建临时 Python 脚本！" << endl;
        return;
    }

    pyScript << "import websocket\n"
               "import datetime\n"
               "import hashlib\n"
               "import base64\n"
               "import hmac\n"
               "import json\n"
               "import os\n"
               "import sys\n"
               "from urllib.parse import urlencode\n\n"
               "class Ws_Param(object):\n"
               "    def __init__(self, APPID, APIKey, APISecret, Text):\n"
               "        self.APPID, self.APIKey, self.APISecret, self.Text = APPID, APIKey, APISecret, Text\n"
               "        self.CommonArgs = {\"app_id\": self.APPID}\n"
               "        self.BusinessArgs = {\"aue\": \"raw\", \"auf\": \"audio/L16;rate=16000\", \"vcn\": \"xiaoyan\", \"tte\": \"utf8\"}\n"
               "        self.Data = {\"status\": 2, \"text\": str(base64.b64encode(self.Text.encode('utf-8')), \"utf-8\")}\n\n"
               "    def create_url(self):\n"
               "        url = 'wss://tts-api.xfyun.cn/v2/tts'\n"
               "        now = datetime.datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT')\n"
               "        signature_origin = \"host: tts-api.xfyun.cn\\n\"\n"
               "        signature_origin += \"date: \" + now + \"\\n\"\n"
               "        signature_origin += \"GET /v2/tts HTTP/1.1\"\n"
               "        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()\n"
               "        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')\n"
               "        # 严格无空格校验拼接\n"
               "        authorization_origin = f'api_key=\"{self.APIKey}\",algorithm=\"hmac-sha256\",headers=\"host date request-line\",signature=\"{signature_sha}\"'\n"
               "        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8').replace('\\n', '')\n"
               "        v = {\"authorization\": authorization, \"date\": now, \"host\": \"tts-api.xfyun.cn\"}\n"
               "        return url + '?' + urlencode(v)\n\n"
               "def on_message(ws, message):\n"
               "    try:\n"
               "        res = json.loads(message)\n"
               "        if res[\"code\"] != 0: ws.close(); return\n"
               "        audio = base64.b64decode(res[\"data\"][\"audio\"])\n"
               "        # 严格保存到 C++ 指定的包内部路径\n"
               "        with open('" << pcm_path << "', 'ab') as f: f.write(audio)\n"
               "        if res[\"data\"][\"status\"] == 2: ws.close()\n"
               "    except: pass\n\n"
               "def on_error(ws, error): pass\n"
               "def on_close(ws,a,b): pass\n"
               "def on_open(ws):\n"
               "    req = {\"common\": wsParam.CommonArgs, \"business\": wsParam.BusinessArgs, \"data\": wsParam.Data}\n"
               "    ws.send(json.dumps(req))\n\n"
               "if __name__ == \"__main__\":\n"
               "    target_dir = os.path.dirname('" << pcm_path << "')\n"
               "    if not os.path.exists(target_dir): os.makedirs(target_dir)\n"
               "    wsParam = Ws_Param(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])\n"
               "    ws = websocket.WebSocketApp(wsParam.create_url(), on_message=on_message, on_error=on_error, on_close=on_close)\n"
               "    ws.on_open = on_open\n"
               "    ws.run_forever()\n";
    pyScript.close();

    // 运行脚本开始合成
    string cmd = "python3 " + py_path + " \"" + XF_APPID + "\" \"" + XF_API_KEY + "\" \"" + XF_API_SECRET + "\" \"" + text + "\"";
    int ret = system(cmd.c_str());

    // 使用测试成功开麦的 default 通道进行放音
    cout << "🔊 [喇叭] 正在外放声音..." << endl;
    string play_cmd = "aplay -D default -r 16000 -f S16_LE -c 1 " + pcm_path + " > /dev/null 2>&1";
    system(play_cmd.c_str());
}

// ─── AIUI 事件监听类 ───
class CloudTestListener : public IAIUIListener {
public:
    // ⚡ 核心修复：更正为精确的 aiui::IAIUIEvent 接口
    void onEvent(const IAIUIEvent& event) const override {
        if (event.getEventType() == AIUIConstant::EVENT_STATE) {
            if (event.getArg1() == AIUIConstant::STATE_WORKING) cout << "[STATE] WORKING - 录音听写链路就绪" << endl;
        }
        else if (event.getEventType() == AIUIConstant::EVENT_VAD) {
            if (event.getArg1() == AIUIConstant::VAD_BOS) cout << "\n🔥 [VAD] >>> 正在录音..." << endl;
            if (event.getArg1() == AIUIConstant::VAD_EOS) cout << "🛑 [VAD] <<< 录音结束，正在等待结果..." << endl;
        }
        else if (event.getEventType() == AIUIConstant::EVENT_RESULT) {
            Json::Value bizParamJson;
            Json::Reader reader;
            if (!reader.parse(event.getInfo(), bizParamJson, false)) return;

            Json::Value data = (bizParamJson["data"])[0];
            Json::Value params = data["params"];
            Json::Value content = (data["content"])[0];
            if (params["sub"].asString() == "iat") {
                string cnt_id = content["cnt_id"].asString();
                int dataLen = 0;
                const char* buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
                if (buffer != nullptr && dataLen > 0) {
                    Json::Value resultJson;
                    Json::Reader resultReader;
                    if (resultReader.parse(string(buffer, dataLen), resultJson)) {
                        string text = "";
                        Json::Value wsNode = resultJson.isMember("text") ? resultJson["text"]["ws"] : resultJson["ws"];
                        if (!wsNode.empty()) {
                            for (int i = 0; i < wsNode.size(); i++) {
                                if (wsNode[i].isMember("cw")) {
                                    text += wsNode[i]["cw"][0]["w"].asString();
                                }
                            }
                        }
                        if (!text.empty()) {
                            cout << "🎯 [当前听到文字] ➔ " << text << endl;
                            speakText("收到指令 " + text);
                        }
                    }
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_asr_test");
    ros::NodeHandle nh;
    signal(SIGINT, sig_handler);

    // 确保依赖存在
    system("pip3 install websocket-client > /dev/null 2>&1");

    string pkg_path = ros::package::getPath("speech_command");
    string cfg_path = pkg_path + "/config/AIUI/cfg/aiui.cfg";

    string fileParam = FileUtil::readFileAsString(cfg_path);
    Json::Value paramJson;
    Json::Reader reader;
    reader.parse(fileParam, paramJson, false);

    paramJson["speech"]["wakeup_mode"] = "off";
    paramJson["speech"]["interact_mode"] = "continuous";
    paramJson["speech"]["data_source"] = "user";

    Json::FastWriter writer;
    string paramStr = writer.write(paramJson);

    static CloudTestListener listener;
    g_agent = IAIUIAgent::createAgent(paramStr.c_str(), &listener);
    
    IAIUIMessage* startMsg = IAIUIMessage::create(AIUIConstant::CMD_START, 0, 0, "", nullptr);
    g_agent->sendMessage(startMsg); startMsg->destroy();
    sleep(1);

    IAIUIMessage* wakeupMsg = IAIUIMessage::create(AIUIConstant::CMD_WAKEUP, 0, 0, "", nullptr);
    g_agent->sendMessage(wakeupMsg); wakeupMsg->destroy();
    sleep(1);

    // 开机动态测试！
    speakText("语音控制与流式合成系统全链路通车。");

    // ─── 录音流初始化 ───
    snd_pcm_t* capture_handle = nullptr;
    snd_pcm_hw_params_t* hw_params = nullptr;
    unsigned int rate = 16000;
    
    if (snd_pcm_open(&capture_handle, "hw:3,0", SND_PCM_STREAM_CAPTURE, 0) < 0) {
        cout << "❌ 无法打开录音声卡" << endl; return -1;
    }

    snd_pcm_hw_params_malloc(&hw_params);
    snd_pcm_hw_params_any(capture_handle, hw_params);
    snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &rate, 0);
    snd_pcm_hw_params_set_channels(capture_handle, hw_params, 1);
    snd_pcm_hw_params(capture_handle, hw_params);
    snd_pcm_hw_params_free(hw_params);
    snd_pcm_prepare(capture_handle);

    int buffer_frames = 512;
    char* audio_buf = (char*)malloc(buffer_frames * 2);

    cout << "\n========================================" << endl;
    cout << "🎤 终极通车测试：可以说话了！" << endl;
    cout << "========================================" << endl;

    while (g_running && ros::ok()) {
        int err = snd_pcm_readi(capture_handle, audio_buf, buffer_frames);
        if (err < 0) { snd_pcm_prepare(capture_handle); continue; }

        Buffer* buffer = Buffer::alloc(buffer_frames * 2);
        memcpy(buffer->data(), audio_buf, buffer_frames * 2);
        IAIUIMessage* writeMsg = IAIUIMessage::create(AIUIConstant::CMD_WRITE, 0, 0, "data_type=audio,sample_rate=16000", buffer);
        g_agent->sendMessage(writeMsg); writeMsg->destroy();

        ros::spinOnce();
    }

    free(audio_buf);
    snd_pcm_close(capture_handle);
    if (g_agent) g_agent->destroy();
    return 0;
}
