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
#include <std_msgs/Bool.h>
#include "aiui/AIUI.h"
#include "AIUITester.h"
#include "FileUtil.h"
#include "jsoncpp/json/json.h"
#include <alsa/asoundlib.h>

using namespace std;
using namespace aiui;

static volatile bool g_running = true;
static IAIUIAgent* g_agent = nullptr;

static bool g_asr_enable = true;
static bool g_close_after_command_accepted = true;
static bool g_command_accepted = false;

// 是否进入等待关闭状态
static bool g_need_destroy = false;

static std::string g_final_text;

// 是否已经发布过任务（防止重复publish）
static bool g_task_sent = false;

// 最近一次收到EVENT_RESULT的时间
static ros::Time g_last_result_time;

// 全局发布者
ros::Publisher pub_voice_raw_text;
ros::Publisher pub_task_state;
ros::Subscriber sub_tts;
ros::Subscriber sub_command_accepted;

const string XF_APPID      = "4c8a2ec8";
const string XF_API_KEY    = "uFtKQJrDyzpKyiVHMAWM";
const string XF_API_SECRET = "btlLJzACqGeZKYFoYQeF";

void sig_handler(int sig) { g_running = false; }

void commandAcceptedCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) return;
    g_command_accepted = true;
    g_need_destroy = true;
    ROS_INFO("Competition voice command accepted; ASR may now stop");
}

void speakText(const string& text) {
    cout << "speakText初始化" << endl;
    if (text.empty()) return;
    string pkg_path = ros::package::getPath("speech_command");
    string py_path = pkg_path + "/xf_tts_stable.py";
    string pcm_path = pkg_path + "/tmp/tts_result.pcm";
    remove(pcm_path.c_str());
    string cmd = "python3 " + py_path + " \"" + XF_APPID + "\" \"" + XF_API_KEY + "\" \"" + XF_API_SECRET + "\" \"" + text + "\"";
    system(cmd.c_str());
    string play_cmd ="aplay -D default -r 16000 -f S16_LE -c 1 "+ pcm_path;
    
    system(play_cmd.c_str());
}

void ttsCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.empty())
        return;

    cout << endl;
    cout << "==============================" << endl;
    cout << "🔊 收到TTS：" << msg->data << endl;
    cout << "==============================" << endl;

    speakText(msg->data);

    cout << "🔈 播放完成" << endl;
}

class CloudTestListener : public IAIUIListener {
public:
    void onEvent(const IAIUIEvent& event) const override {
        cout << "DEBUG: [AIUI Event] Type: " << event.getEventType() << endl;
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
                    cout << resultJson.toStyledString() << endl;
                    string text = "";
                    Json::Value wsNode = resultJson.isMember("text") ? resultJson["text"]["ws"] : resultJson["ws"];
                    for (int i = 0; i < wsNode.size(); i++) text += wsNode[i]["cw"][0]["w"].asString();
                    
                    if (!text.empty())
                    {
                        // 每收到一次识别结果，都更新时间
                        g_last_result_time = ros::Time::now();

                        // 第一次收到结果
                        if(!g_task_sent)
                        {
                            // 更新时间
                            g_last_result_time = ros::Time::now();

                            // 始终保存最新识别结果
                            g_final_text = text;

                            cout << "📝 更新识别结果：" << g_final_text << endl;

                            // 进入等待关闭
                            g_need_destroy = true;
                        }

                        // 进入等待关闭状态
                        g_need_destroy = true;
                    }

                    
                }
            }
        }
        else if (event.getEventType() == AIUIConstant::EVENT_ERROR) {
            cout << "❌ [AIUI 错误] 错误码: " << event.getArg1() 
                 << " 信息: " << event.getInfo() << endl;}
                 else if (event.getEventType() == AIUIConstant::EVENT_STATE) {
            cout << "💡 [AIUI 状态] 状态码: " << event.getArg1() 
                 << (event.getArg1() == 1 ? " (正在工作)" : " (空闲/停止)") << endl;
            }
    }
    
                };

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_asr_test");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string tts_topic;
    std::string command_accepted_topic;
    private_nh.param<std::string>("tts_topic", tts_topic, "/factory/tts_text");
    private_nh.param<std::string>(
        "command_accepted_topic", command_accepted_topic,
        "/factory/voice_command_accepted");
    private_nh.param<bool>(
        "close_after_command_accepted", g_close_after_command_accepted, true);
    g_last_result_time = ros::Time::now();
    g_task_sent = false;
    signal(SIGINT, sig_handler);

    pub_voice_raw_text = nh.advertise<std_msgs::String>("/factory/voice_raw_text", 10);
    pub_task_state = nh.advertise<std_msgs::Int32>("/factory/task_state", 10);

    sub_tts =
    nh.subscribe(
        tts_topic,
        10,
        ttsCallback);
    sub_command_accepted = nh.subscribe(
        command_accepted_topic, 1, commandAcceptedCallback);
    ROS_INFO_STREAM("TTS subscriber listening on " << tts_topic);
    ROS_INFO_STREAM(
        "ASR persistent until accepted command on "
        << command_accepted_topic);

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

    sleep(1);

IAIUIMessage* wakeupMsg =
    IAIUIMessage::create(
        AIUIConstant::CMD_WAKEUP,
        0,
        0,
        "",
        nullptr);

g_agent->sendMessage(wakeupMsg);
wakeupMsg->destroy();

cout<<"🚀 已发送CMD_WAKEUP"<<endl;
    
    IAIUIMessage* recordMsg = IAIUIMessage::create(AIUIConstant::CMD_START_RECORD, 0, 0, "data_type=audio", nullptr);
    g_agent->sendMessage(recordMsg);
    recordMsg->destroy();
    cout << "🚀 强制触发录音指令发送！" << endl;



    snd_pcm_t* capture_handle = nullptr;
    snd_pcm_hw_params_t* hw_params = nullptr;
    unsigned int rate = 16000;
    const char* pcm_device = "hw:3,0";
    // const char* pcm_device = "hw:XFMDPV0018";

    if (snd_pcm_open(&capture_handle, pcm_device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        pcm_device = "hw:3,0";
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
    //cout << "✅ 核心逻辑就绪，等待语音触发..." << endl;

    while (g_running && ros::ok()) {
        ros::spinOnce();

    if(g_need_destroy && g_agent)
    {
        double idle_time =
            (ros::Time::now() - g_last_result_time).toSec();

        // 连续1秒没有新的识别结果
        if(!g_final_text.empty())
        {
            cout << endl;
            cout << "==============================" << endl;
            cout << "🎯 最终识别结果：" << g_final_text << endl;
            cout << "==============================" << endl;

            std_msgs::String msg;
            msg.data = g_final_text;
            pub_voice_raw_text.publish(msg);


            g_final_text.clear();
        }

        if(idle_time > 2.0)
        {
            if(g_close_after_command_accepted && !g_command_accepted)
            {
                // Silence after a wake word, an unrelated phrase, or a
                // partial order must never close the competition listener.
                g_need_destroy = false;
                ROS_INFO("ASR remains active; waiting complete competition command");
                continue;
            }
            cout << "🛑 已连续 "
                << idle_time
                << " 秒没有新的识别结果，比赛指令已确认，关闭AIUI..."
                << endl;
            

            // pub_voice_raw_text.publish(msg);

            g_asr_enable = false;


            IAIUIMessage* stopRecord =
                IAIUIMessage::create(
                    AIUIConstant::CMD_STOP_RECORD,
                    0,
                    0,
                    "data_type=audio",
                    nullptr);

            g_agent->sendMessage(stopRecord);
            stopRecord->destroy();

            IAIUIMessage* stop =
                IAIUIMessage::create(
                    AIUIConstant::CMD_STOP,
                    0,
                    0,
                    "",
                    nullptr);

            g_agent->sendMessage(stop);
            stop->destroy();

            usleep(300000);

            g_agent->destroy();
            g_agent = nullptr;

            g_need_destroy = false;

            cout << "✅ AIUI 已彻底关闭（比赛模式）" << endl;

        }
    }


        if(!g_asr_enable)
        {
        usleep(10000);
        continue;
        }

        // 1. 读取真实音频数据
        int err = snd_pcm_readi(capture_handle, audio_buf, buffer_frames);
        if (err < 0) { 
            snd_pcm_prepare(capture_handle); 
            continue; 
        }

        // 2. 仅发送真实音频流 (去掉 silent_buf 干扰)
        Buffer* buffer = Buffer::alloc(buffer_frames * 2);
        memcpy(buffer->data(), audio_buf, buffer_frames * 2);
        IAIUIMessage* writeMsg =
    IAIUIMessage::create(
        AIUIConstant::CMD_WRITE,
        0,
        0,
        "data_type=audio,sample_rate=16000",
        buffer);
        if(g_agent)
        {
            g_agent->sendMessage(writeMsg);
        }
        writeMsg->destroy();
        
        
    }
    

    free(audio_buf);
    snd_pcm_close(capture_handle);
    if (g_agent) g_agent->destroy();
    return 0;
}
