#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <AIUITester.h>
#include <signal.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <thread>
#include <mutex>
#include "jsoncpp/json/json.h"

char pp11_[3000];
using namespace std;
vector<string> third_line;
bool get_request_test = false;
#define TIMEOUT 10

/* 读取配置文件数据,保存到内存中 */
int LoadUserConfig(string config_txt)
{
	std::ifstream fd(config_txt.c_str());
	if (!fd.is_open())
	{
		cout << "can not open the user config,make sure it is readable" << endl;
		return -1;
	}
	string line;
	while (std::getline(fd, line))
	{
		vector<string> vstr;
		boost::split(vstr, line, boost::is_any_of(":"));
		if (vstr.size() != 3)
		{
			break;
		}
		vector<string> questions;

		third_line.push_back(vstr[2]);
		boost::split(questions, vstr[0], boost::is_any_of("|"));
		for (vector<string>::iterator it = questions.begin(); it != questions.end(); ++it)
		{
			QA_list_.insert(std::pair<std::string, std::string>(*it, vstr[1]));
			QA_list_doc.insert(std::pair<std::string, std::string>(*it, vstr[2]));
		}
	}
	return 0;
}

/* 在用户列表中查找识别到的结果,返回数据(协议) */
std::string FindKeyWords(std::string recognise_result)
{
	std::map<std::string, std::string>::iterator it;
	auto find_result_ = recognise_result.find("。");
	if (find_result_ != string::npos)
	{
		recognise_result.erase(find_result_, 1);
	}
	auto find_result_1 = recognise_result.find(".");
	if (find_result_1 != string::npos)
	{
		recognise_result.erase(find_result_1, 1);
	}
	it = QA_list_.find(recognise_result);
	if (it != QA_list_.end())
	{
		return it->second;
	}
	else
	{
		return "";
	}
}

std::string FindDocument(std::string recognise_result)
{
	std::map<std::string, std::string>::iterator its;
	auto find_result_ = recognise_result.find("。");
	if (find_result_ != string::npos)
	{
		recognise_result.erase(find_result_, 1);
	}
	its = QA_list_doc.find(recognise_result);
	if (its != QA_list_doc.end())
	{
		return its->second;
	}
	else
	{
		return "";
	}
}

void write_serial() {}

int data_send(int argc, char **argv)
{
	ros::init(argc, argv, "publisher_Node");
	ros::NodeHandle n;
	ros::Publisher pub_question = n.advertise<std_msgs::String>("/question", 10);
	ros::Publisher pub_answer = n.advertise<std_msgs::String>("/answer", 10);
	ros::Publisher pub_angle = n.advertise<std_msgs::Int32>("/angle", 10);
	
	// ─── 🔌 核心新增：注册发布裁判原话的话题发布者（名字与 Python 端完美对齐） ───
	ros::Publisher pub_voice_raw_text = n.advertise<std_msgs::String>("/factory/voice_raw_text", 10);

	string ros_package_path1 = ros::package::getPath("speech_command");
	package_path1 = const_cast<char *>(ros_package_path1.c_str());
        
	while (ros::ok())
	{
		if ((sign_conversation_cloud == 1) || (sign_conversation_local == 1))
		{
			string question_str(question);
			string answer_str(answer);
			std_msgs::String msg1;
			msg1.data = question_str;
			pub_question.publish(msg1);

			std_msgs::String msg3;
			msg3.data = answer_str;
			pub_answer.publish(msg3);

			cout << "问题:\t" << question_str << "\n"
				 << "答案:\t" << answer_str << endl;

			// ─── ⚡ 核心新增：将听到的裁判原话通过 ROS 甩给 Python 蓄水池 ───
			if (!question_str.empty())
			{
				std_msgs::String voice_msg;
				voice_msg.data = question_str;
				pub_voice_raw_text.publish(voice_msg);
				ROS_INFO("📢 [C++听觉] 裁判原话已成功抛出 -> /factory/voice_raw_text: %s", question_str.c_str());
			}

			if (sign_conversation_local == 1)
			{
				string command_ = FindKeyWords(question_str);
				string document = FindDocument(question_str);
				cout << "下发协议:\t" << command_ << endl;
				strcpy(pp11_, package_path1);
				string wav = string((char*)pp11_);
				std::string wav_path1 = wav.append(document);
				std::string command1 = "play "+wav_path1;
				system(command1.c_str());
			}

			sign_conversation_cloud = 0;
			sign_conversation_local = 0;
		}
		if (sign_angle == true)
		{
			std_msgs::Int32 msg2;
			msg2.data = angle;
			pub_angle.publish(msg2);
			sign_angle = false;
		}
		ros::spinOnce();
	}
}

bool get_test_server(std_srvs::Trigger::Request &request,std_srvs::Trigger::Response &response)
{
	ROS_INFO("got request,start to get the speech test ...\n");
	clock_t startTime, endTime;
	startTime = clock();
	get_request_test = false;
	while (!get_request_test)
	{
		endTime = clock();                                             
		if ((double)(endTime - startTime) / CLOCKS_PER_SEC > TIMEOUT) 
		{
			response.success = false;
			response.message = "timeout_error";
			return true;
		}
	}
	response.success = true;
	response.message = "success";
	return true;
}

bool get_test_video(std_srvs::Trigger::Request &request,std_srvs::Trigger::Response &response)
{
	system("aplay /home/iflytek/ucar_ws/src/speech_command/src/tts_sample.wav &");
	response.success = true;
	response.message = "success";
	return true;
}

void test_callback()
{
	get_request_test = true;
}

// ====== 💡 核心保留：专门接收大模型文本并调用原厂 TTS 播报的回调函数 ======
void xunfei_llm_tts_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("TTS callback received: %s", msg->data.c_str()); // 改为纯英文日志
    gTTS(msg->data.c_str()); 
}

int main(int argc, char **argv)
{
	try
	{
		_serial.setPort(DEV_ID);
		_serial.setBaudrate(BAUD_RATE);
		_serial.setFlowcontrol(serial::flowcontrol_none);
		_serial.setParity(serial::parity_none); 
		_serial.setStopbits(serial::stopbits_one);
		_serial.setBytesize(serial::eightbits);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		ROS_INFO_STREAM("Port has been open successfully");
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}

	if (_serial.isOpen())
	{
		sleep(1/10);
		_serial.flush();			
		ROS_INFO_STREAM("port initial successfully------");
	}
	string ros_package_path = ros::package::getPath("speech_command");
	package_path = const_cast<char *>(ros_package_path.c_str());
	
	char pp_[3000], pp1_[3000], pp2_[3000], pp3_[3000], pp4_[3000], pp5_[3000], pp6_[3000], pp7_[3000], pp8_[3000], pp9_[3000], pp10_[3000];
	strcpy(pp_, package_path);     CFG_FILE_PATH = strcat(pp_, CFG_FILE_PATH);
	strcpy(pp1_, package_path);    SOURCE_FILE_PATH = strcat(pp1_, SOURCE_FILE_PATH);
	strcpy(pp2_, package_path);    GRAMMAR_FILE_PATH = strcat(pp2_, GRAMMAR_FILE_PATH);
	strcpy(pp3_, package_path);    TEST_AUDIO_PATH = strcat(pp3_, TEST_AUDIO_PATH);
	strcpy(pp4_, package_path);    LOG_DIR = strcat(pp4_, LOG_DIR);
	strcpy(pp5_, package_path);    CONFIG_FILE_PATH = strcat(pp5_, CONFIG_FILE_PATH);
	strcpy(pp6_, package_path);    PCM_FILE_PATH = strcat(pp6_, PCM_FILE_PATH);
	strcpy(pp7_, package_path);    ORIPCM_FILE_PATH = strcat(pp7_, ORIPCM_FILE_PATH);
	strcpy(pp8_, package_path);    WAKEUP_RESPONSE_WAV = strcat(pp8_, WAKEUP_RESPONSE_WAV);
	strcpy(pp9_, package_path);    NO_INTERNET_RESPONSE_WAV = strcat(pp9_, NO_INTERNET_RESPONSE_WAV);

	string user_config_path = ros_package_path + USER_CONFIG_PATH;
	LoadUserConfig(user_config_path);
	AIUITester t;

	ros::init(argc, argv, "speech_command_node");
	ros::NodeHandle ndHandle;

	// 1. 挂载我们新加的话题订阅
	ros::Subscriber sub_llm_tts = ndHandle.subscribe("/factory/tts_text", 10, xunfei_llm_tts_callback);
	
	/* srv 获取当前唤醒结果 */
	ros::ServiceServer service_get_test = ndHandle.advertiseService("/speech_command_node/get_test_server", get_test_server);
	ros::ServiceServer server_test_video = ndHandle.advertiseService("/speech_command_node/get_test_video", get_test_video);

	t.bind(test_callback);

	// 2. ⚡ 核心解死锁：启动异步多线程事件轮询器，专属于后台去听 /factory/tts_text 的数据
	ros::AsyncSpinner spinner(2); 
	spinner.start();

	std::thread t1(std::bind(&AIUITester::test,&t));
    std::thread t2(data_send,argc, argv);

	t1.join();
	t2.join();

	t.destory();
	return 0;
}