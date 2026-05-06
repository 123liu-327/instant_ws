#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include"task_class.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <locale.h>
using namespace std;
std::string goalObj;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimTriggerClient {
    public:
        SimTriggerClient(const char* server_ip, int port) 
            : server_ip_(server_ip), port_(port) {}
        
        bool sendTrigger() {
            cout<<"!!!"<<endl;
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                ROS_ERROR("Socket creation error");
                return false;
            }
            else ROS_INFO("Socket Success!");
    
            sockaddr_in serv_addr;
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(port_);
            
            if(inet_pton(AF_INET, server_ip_, &serv_addr.sin_addr) <= 0) {
                ROS_ERROR("Invalid address/Address not supported");
                close(sock);
                return false;
            }
            else ROS_INFO("Address Success!");
    
    
            if (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                ROS_ERROR("Connection Failed");
                close(sock);
                return false;
            }
            else{
                ROS_INFO("Connection Success!");
            }
    
            const char* message=goalObj.c_str();
            if (send(sock, message, strlen(message), 0) <= 0) {
                ROS_ERROR("Send failed");
               // close(sock);
                return false;
            }
            
            ROS_INFO("仿真触发信号已发送至: %s:%d", server_ip_, port_);
            
            //close(sock);
            return true;
        }
    std::string recRet()
    {
        char buffer[1024];
        recv(sock, buffer, 1024,0);
        ros::Rate r(200);
        while(ros::ok())
        {
            r.sleep();
        
            if(buffer[0])
            {
                ROS_INFO("recv buf:%d",buffer[0]);
                break;
            }
            
        }
        std::string ret;
        for(int i=0;i<5;i++)
            ret=ret+buffer[i];
        return ret;
    }
    private:
        const char* server_ip_;
        int port_;
        int sock;
};

std::vector<move_base_msgs::MoveBaseGoal>Goals;//导航目标数组
const int goalsSize=15;//导航目标数组大小

void initGoals()
{
    Goals.resize(goalsSize);
    //二维码扫描点
    Goals[1].target_pose.header.frame_id="map";
    Goals[1].target_pose.header.stamp=ros::Time::now();
    Goals[1].target_pose.pose.position.x=0.891;
    Goals[1].target_pose.pose.position.y=0.477;
    Goals[1].target_pose.pose.position.z=0.000;
    Goals[1].target_pose.pose.orientation.z=1;
    Goals[1].target_pose.pose.orientation.w=0;

    //过渡
    Goals[2].target_pose.header.frame_id="map";
    Goals[2].target_pose.header.stamp=ros::Time::now();
    Goals[2].target_pose.pose.position.x=0.55;//1.7
    Goals[2].target_pose.pose.position.y=1.5;//1.0
    Goals[2].target_pose.pose.position.z=0.000;
    Goals[2].target_pose.pose.orientation.z=0.7071;
    Goals[2].target_pose.pose.orientation.w=0.7071; 
    
    //仿真点
    Goals[3].target_pose.header.frame_id="map";
    Goals[3].target_pose.header.stamp=ros::Time::now();
    Goals[3].target_pose.pose.position.x=1.356;
    Goals[3].target_pose.pose.position.y=3.43;//3.363
    Goals[3].target_pose.pose.position.z=0.000;
    Goals[3].target_pose.pose.orientation.z=0.720;
    Goals[3].target_pose.pose.orientation.w=0.694;    
    
    Goals[4].target_pose.header.frame_id="map";
    Goals[4].target_pose.header.stamp=ros::Time::now();
    Goals[4].target_pose.pose.position.x=2.266;
    Goals[4].target_pose.pose.position.y=3.903;
    Goals[4].target_pose.pose.position.z=0.000;
    Goals[4].target_pose.pose.orientation.z=0;
    Goals[4].target_pose.pose.orientation.w=1;        
    //红绿灯板
    Goals[5].target_pose.header.frame_id="map";
    Goals[5].target_pose.header.stamp=ros::Time::now();
    Goals[5].target_pose.pose.position.x=3.485;
    Goals[5].target_pose.pose.position.y=3.795;
    Goals[5].target_pose.pose.position.z=0.000;
    Goals[5].target_pose.pose.orientation.z=0.707;
    Goals[5].target_pose.pose.orientation.w=0.707;  
    
    //过渡

    //巡线左起点
    Goals[6].target_pose.header.frame_id="map";
    Goals[6].target_pose.header.stamp=ros::Time::now();
    Goals[6].target_pose.pose.position.x=2.923;
    Goals[6].target_pose.pose.position.y=3.266;
    Goals[6].target_pose.pose.position.z=0.000;
    Goals[6].target_pose.pose.orientation.z=-0.662;
    Goals[6].target_pose.pose.orientation.w=0.750;

    //巡线右起点
    
    Goals[7].target_pose.header.frame_id="map";
    Goals[7].target_pose.header.stamp=ros::Time::now();
    Goals[7].target_pose.pose.position.x=4.883;
    Goals[7].target_pose.pose.position.y=3.054;
    Goals[7].target_pose.pose.position.z=0.000;
    Goals[7].target_pose.pose.orientation.z=-0.797;
    Goals[7].target_pose.pose.orientation.w=0.605;
    
    //路口2的灯
    Goals[8].target_pose.header.frame_id="map";
    Goals[8].target_pose.header.stamp=ros::Time::now();
    Goals[8].target_pose.pose.position.x=4.454;
    Goals[8].target_pose.pose.position.y=3.912;
    Goals[8].target_pose.pose.position.z=0.000;
    Goals[8].target_pose.pose.orientation.z=0.707;
    Goals[8].target_pose.pose.orientation.w=0.707;  

    Goals[9].target_pose.header.frame_id="map";
    Goals[9].target_pose.header.stamp=ros::Time::now();
    Goals[9].target_pose.pose.position.x=4.75;
    Goals[9].target_pose.pose.position.y=3.912;
    Goals[9].target_pose.pose.position.z=0.000;
    Goals[9].target_pose.pose.orientation.z=-0.7071;
    Goals[9].target_pose.pose.orientation.w=0.7071;
}
int main(int argc,char** argv)
{
    setlocale(LC_ALL, "en_US.UTF-8");  // 或 "zh_CN.UTF-8"
    ros::init(argc,argv,"nav_node");
    task_node tn;
    ros::NodeHandle nh;

    std::string server_ip;
    int server_port;
    nh.param<string>("sim_server_ip", server_ip, "192.168.0.240"); // 默认IP
    nh.param("sim_server_port", server_port, 1145); // 默认端口
    
    ROS_INFO("仿真服务器: %s:%d", server_ip.c_str(), server_port);
    SimTriggerClient trigger(server_ip.c_str(), server_port);


    MoveBaseClient ac("move_base",true);
   // ROS_ERROR("param send to lane line service");
    //tn.snedToFollow();
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("等待导航服务器启动\n");
    
    }


    //仿真测试
     tn.goalObj="des";
    // tn.getObj="des3";

  //  tn.jies();
    //仿真厕所

    initGoals();
    for(int i=3;i<4;i++)
    {
        //if(i!=0) sleep(0.5);
     //   ac.sendGoal(Goals[i]);
        
     //   bool result = ac.waitForResult();
        int result=1;
        if(result){
            actionlib::SimpleClientGoalState state = ac.getState();
            if(1){
                ROS_INFO("到达目标点 %d!", i);
                
                // 到达目标点4时触发仿真
              {
                //    tn.find_board();
                    //ros::Duration(0.8).sleep();
               //     ac.sendGoal(Goals[3]);
                    bool res = ac.waitForResult();
                    if(1){
                        // actionlib::SimpleClientGoalState state = ac.getState();
                        // if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                        //     ros::Duration(0.8).sleep();
                        // //    ac.sendGoal(Goals[3]);
                        //     ac.waitForResult();
                        // }
                    }
                    
                    
                    ROS_INFO("已到达仿真触发点，开始通信...");
                    
                    goalObj=tn.goalObj;
                    trigger.sendTrigger();
                    std::string ss=trigger.recRet();    
                    ROS_WARN("ss:%s",ss.c_str());
                    tn.simRetPlay(ss[0]);
                    for(int i=1;i<5;i++)
                        tn.simObj+=ss[i];                      
                

                }
                if(i==1)
                {
                    ROS_INFO("开始扫玛");
                    tn.start_qr_code();
                }
                if(i==5)
                {
                    tn.isGreen();
                    if(tn.Green)
                    {
                        tn.playWay();
                        ac.sendGoal(Goals[6]);
                        ac.waitForResult();
                    }
                    else
                    {
                        
                        ac.sendGoal(Goals[8]);
                        ac.waitForResult();
                        tn.playWay();
                        ac.sendGoal(Goals[9]);
                        ac.waitForResult();
                        ac.sendGoal(Goals[7]);
                        ac.waitForResult();                        
                    }   
                    

                    tn.snedToFollow();
                }    
                
            }
            else{
                ROS_WARN("到达目标点 %d 失败: %s", i, state.toString().c_str());
            }
        }
        else{
            ROS_ERROR("等待结果超时");
        }
        ros::Duration(0.8).sleep(); // 短暂延迟

    }
        /*
        if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Mission complete!");
            if(i==0)
            {
                tn.start_qr_code();
                ROS_INFO("get str:%s",tn.goalObj.c_str());
            }

        }  
        else    
            ROS_INFO("Mission failed...");
        */  
    tn.jies();
    return 0;    
}

    
