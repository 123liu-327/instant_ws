#include <iostream>
#include <string>
#include <cstring>
#include <signal.h>
#include <unistd.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "aiui/AIUI.h"
#include "AIUITester.h"
#include "FileUtil.h"
#include "jsoncpp/json/json.h"
#include <alsa/asoundlib.h>

using namespace std;
using namespace aiui;

static volatile bool g_running = true;
static IAIUIAgent* g_agent = nullptr;

// 全局发布者
ros::Publisher pub_voice_raw_text;
ros::Publisher pub_task_state;

const string XF_APPID      = "4c8a2ec8";
const string XF_API_KEY    = "uFtKQJrDyzpKyiVHMAWM";
const string XF_API_SECRET = "btlLJzACqGeZKYFoYQeF";

void sig_handler(int sig) { g_running = false; }

void speakText(const string& text) {
    if (text.empty()) return;
    string pkg_path = ros::package::getPath("speech_command");
    string py_path = pkg_path + "/xf_tts_stable.py";
    string pcm_path = pkg_path + "/tmp/tts_result.pcm";
    string cmd = "python3 " + py_path + " \"" + XF_APPID + "\" \"" + XF_API_KEY + "\" \"" + XF_API_SECRET + "\" \"" + text + "\"";
    system(cmd.c_str());
    string play_cmd = "aplay -D default -r 16000 -f S16_LE -c 1 " + pcm_path + " > /dev/null 2>&1 &";
    system(play_cmd.c_str());
}

class CloudTestListener : public IAIUIListener {
public:
    void onEvent(const IAIUIEvent& event) const override {
        if (event.getEventType() == AIUIConstant::EVENT_RESULT) {
            Json::Value bizParamJson;
            Json::Reader reader;
            if (!reader.parse(event.getInfo(), bizParamJson, false)) return;
            
            Json::Value data = (bizParamJson["data"])[0];
            Json::Value content = (data["content"])[0];
            string cnt_id = content["cnt_id"].asString();
            int dataLen = 0;
            const char* buffer = event.getData()->getBinary(cnt_id.c_str(), &dataLen);
            
            if (buffer != nullptr && dataLen > 0) {
                Json::Value resultJson;
                Json::Reader resultReader;
                if (resultReader.parse(string(buffer, dataLen), resultJson)) {
                    string text = "";
                    Json::Value wsNode = resultJson.isMember("text") ? resultJson["text"]["ws"] : resultJson["ws"];
                    for (int i = 0; i < wsNode.size(); i++) text += wsNode[i]["cw"][0]["w"].asString();
                    
                    if (!text.empty()) {
                        cout << "🎯 [识别到指令] ➔ " << text << endl;
                        std_msgs::String msg; msg.data = text;
                        pub_voice_raw_text.publish(msg);
                        std_msgs::Int32 state_msg; state_msg.data = 1; 
                        pub_task_state.publish(state_msg);
                        speakText("指令收到，小车前往分拣区");
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

    pub_voice_raw_text = nh.advertise<std_msgs::String>("/factory/voice_raw_text", 10);
    pub_task_state = nh.advertise<std_msgs::Int32>("/factory/task_state", 10);

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

    snd_pcm_t* capture_handle = nullptr;
    snd_pcm_hw_params_t* hw_params = nullptr;
    unsigned int rate = 16000;
    const char* pcm_device = "hw:XFMDPV0018";

    if (snd_pcm_open(&capture_handle, pcm_device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        pcm_device = "hw:2,0";
        if (snd_pcm_open(&capture_handle, pcm_device, SND_PCM_STREAM_CAPTURE, 0) < 0) return -1;
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

    cout << "✅ 核心逻辑就绪，等待语音触发..." << endl;

    while (g_running && ros::ok()) {
        int err = snd_pcm_readi(capture_handle, audio_buf, buffer_frames);
        if (err < 0) { snd_pcm_prepare(capture_handle); continue; }
        
        Buffer* buffer = Buffer::alloc(buffer_frames * 2);
        memcpy(buffer->data(), audio_buf, buffer_frames * 2);
        IAIUIMessage* writeMsg = IAIUIMessage::create(AIUIConstant::CMD_WRITE, 0, 0, "data_type=audio", buffer);
        g_agent->sendMessage(writeMsg); 
        writeMsg->destroy();
        ros::spinOnce();
    }

    free(audio_buf);
    snd_pcm_close(capture_handle);
    if (g_agent) g_agent->destroy();
    return 0;
}