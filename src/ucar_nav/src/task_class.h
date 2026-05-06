#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include<geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include<sensor_msgs/LaserScan.h>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <dec_msgs/Detection.h>
#include <dec_msgs/DetectionArray.h>
#include <ucar_nav/EnableLaneFollow.h>
const double eps=1e-5;
const double pi=3.14159265359;
const double boardAngle=15;
using namespace cv;
std::string pref="aplay /home/ucar/ucar_ws/src/playsound/converted_wav/";
std::string afz=".wav";
class task_node
{
    public:
        ros::NodeHandle nh;
        std::string goalObj;//要找蔬菜、水果还是甜点
        int dz;
        double diffRate;
        double diff;
        int money;//有多少钱
        int goalIdx;
        int isEnd;
        int Green;
        double imgMid;
        std::string getObj;
        std::string simObj;
        std::map<std::string,int> objCnt;
        std::map<std::string,int> objCombo;
        std::string last;
        int fstYolo;
        int findTarget;
        double rcRate;
        double  midDis=80,dis15=80;//雷达正前方和偏15度角距离
        std::map<std::string,int> mp;
       
    public:
        task_node()//构造函数
        {
            dz=0;
            rcRate=0.6;
            money=20;
            goalIdx=1;
            fstYolo=0;
            imgMid=1280;
            findTarget=0;
            
            Green=0;
            midDis=99999;
            dis15=10;
            last="haha";
            isEnd=0;
            mp["fru1"]=2,mp["fru2"]=4,mp["fru3"]=5;
            mp["veg1"]=2,mp["veg2"]=5,mp["veg3"]=2;
            mp["des1"]=3,mp["des2"]=10,mp["des3"]=5;
        }

        void simRetPlay(int room)//得知在哪个房间后播报语音
        {
            std::string ppath;
            ppath+=pref;
            ppath+="fzjs";
            ppath+=afz;
            system(ppath.c_str());    
            ppath.clear();
            ppath+=pref;                    
            if(room==3)
            {
                ppath+="rooma";
                ppath+=afz;
                
            }
            else if(room==2)
            {
                ppath+="roomb";
                ppath+=afz;   
            }
            else
            {
                ppath+="roomc";
                ppath+=afz;
            }
            system(ppath.c_str());
        }

        void qr_node_callback(const std_msgs::String::ConstPtr& msg)
        {
            goalObj=msg->data;
            ROS_INFO("task_node已获取目标：%s",goalObj.c_str());
            return;
        }

        void snedToFollow()
        {
            if(Green)
                system("rosservice call /enable_lane_follow true 1");
            else
                system("rosservice call /enable_lane_follow true 0");
        }
        void start_qr_code()//开始扫码，获取任务信息，并播报语音(播报语音还没实现)
        {
           ros::Publisher qr_start_pub;
           ros::Subscriber qr_info_sub;//订阅话题获取物品信息         
           qr_info_sub=nh.subscribe<std_msgs::String>("/qr_code_result",10, boost::bind(&task_node::qr_node_callback, this, _1));
           qr_start_pub=nh.advertise<std_msgs::String>("/qr_node_start",10);
           std_msgs::String start_msg;
           start_msg.data="start!";
           ros::Rate r(5);
           for(int i=0;i<3;i++)
           {
                
                qr_start_pub.publish(start_msg);
                r.sleep();
                ros::spinOnce();
           }
            for(int i=0;i<16;i++)
            {
                r.sleep();
                ros::spinOnce();                  
            }
         
           

           if(this->goalObj=="Dessert")
           {
                this->goalObj="des";
           }
           else if(this->goalObj=="Fruit")
                this->goalObj="fru";
            else
                this->goalObj="veg";
           std::string ppath=pref;
           ppath+="bccgrw";
           ppath+=afz;
           system(ppath.c_str());
           ppath=pref;
           ppath+=this->goalObj;
           ppath+=afz;
           system(ppath.c_str());
           return;
        }   
        void yolo_callback(const dec_msgs::DetectionArrayConstPtr& msg)//检测节点的回调，初步调整方向
        {
 
            for(int i=0;i<msg->detections.size();i++)
            {
                if(goalObj[2]==msg->detections[i].class_name[2]&&goalObj[1]==msg->detections[i].class_name[1]&&goalObj[0]==msg->detections[i].class_name[0])
                {
                    if(msg->detections[i].class_name.data()==last)      
                        objCombo[msg->detections[i].class_name.data()]++;
                    else 
                        objCombo[msg->detections[i].class_name.data()]=1,objCombo[last]=0;
                    last=msg->detections[i].class_name.data();
                    if(objCombo[msg->detections[i].class_name.data()]<2)
                        return;
                    last=msg->detections[i].class_name.data();
                    objCnt[msg->detections[i].class_name.data()]++;
                    findTarget=1;
                    diff=msg->detections[i].x_max+msg->detections[i].x_min-imgMid;
                    diffRate=std::fabs(diff)/(msg->detections[i].x_max-msg->detections[i].x_min+80);
                    if(diffRate<0.99)
                        dz=1;
                    else
                    {
                        geometry_msgs::Twist vel;
                        if(diff>0)
                        {
                        //  ROS_INFO("too left");
                            if(fstYolo==0)
                            {

                                imgMid+=-0.235*1280.0/(msg->detections[i].x_max-msg->detections[i].x_min+820);
                                ROS_INFO("ImgMid:%f",imgMid);
                            }
                                
                            
                        }
                            
                        else
                        {
                        //  ROS_INFO("too right");
                            if(fstYolo==0)
                            {
                                imgMid-=-0.235*1280.0/(msg->detections[i].x_max-msg->detections[i].x_min+820);
                                ROS_INFO("ImgMid:%f",imgMid);                                
                            }
                    

                        }
                            
                        
                    }
                
                }
            }
            fstYolo=1;
        }

        void LidarCallBack(const sensor_msgs::LaserScanConstPtr& msg)
        {
            int front_index = (0 - msg->angle_min) / msg->angle_increment;
            int angle_index =  front_index+(boardAngle*pi/180.0)/msg->angle_increment;
            midDis=msg->ranges[front_index];
            dis15=msg->ranges[angle_index];
            ROS_INFO("雷达测距：midDis=%f,dis15=%f",midDis,dis15);
        }
        void find_board()//到达仿真点后开始找板子
        {
             ros::Publisher vel_pub;
             vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",20);
            ros::Subscriber lidar_sub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,boost::bind(&task_node::LidarCallBack,this,_1));
             geometry_msgs::Twist myVel;
             ros::Rate r1(20);
             myVel.linear.x=-0.5;
             midDis=0.0;
             double hcDis=0.32;
             for(int i=0;i<20;i++)
                ros::spinOnce(),r1.sleep();
            if(midDis<0.3)
             for(int ii=0;ii<20;ii++){
                ROS_WARN("test:%f",midDis);
                ros::spinOnce();
                vel_pub.publish(myVel);
                r1.sleep();
             }
            for(int i=1;i<=25;i++){
                myVel.linear.x=0;
                vel_pub.publish(myVel);     
                r1.sleep();                   
            }             
             midDis=99999.0;
            ros::Publisher str_pub=nh.advertise<std_msgs::String>("/detection_start",4);
             for(int i=0;i<5;i++)
            {

                std_msgs::String strMsg;
                strMsg.data="start";
                str_pub.publish(strMsg);
                ros::spinOnce();
                r1.sleep();
            }
             ROS_INFO("speed has been published");
             ros::Subscriber yolo_sub;
             yolo_sub=nh.subscribe<dec_msgs::DetectionArray>("/detection_result",10,boost::bind(&task_node::yolo_callback,this,_1));
             ros::Rate r(100);
             while(ros::ok())
             {          
                myVel.angular.z=0.3;
                
           //     ROS_INFO("be in rotate");
                ros::spinOnce();
                if(findTarget==1)
                {
                    if(dz)
                    {
                        ROS_INFO("yi dui zhun");
                        geometry_msgs::Twist stp;
                        vel_pub.publish(stp);
                        ros::spinOnce();
                        break;
                    }
                    else
                    {
                        if(diff>0)
                        {
                            myVel.angular.z=-0.15;
                    //        ROS_INFO("too right");
                            
                        }
                            
                        else
                        {
                    //        ROS_INFO("too left");
                            myVel.angular.z=0.15;
                        }
                        
                    }
                //    ROS_INFO("diffRate:%f",diffRate);
                //    findTarget=0;             
                }
                vel_pub.publish(myVel);   
                r.sleep();
             }
             
             myVel.angular.z=0;
             while(ros::ok())
             {
                ros::spinOnce();
                if(midDis>0.4||midDis==0)
                    myVel.linear.x=0.3;
                else
                {
                    myVel.linear.x=0;
                    ROS_INFO("已经靠近");
                    break;
                }    
                vel_pub.publish(myVel);
                r.sleep();
             }
            double hxjl=0;
            int cnt=0;
            ros::Time lidarStart = ros::Time::now();
            while(ros::ok())
            {
                ros::Time lidarCur=ros::Time::now();
                ros::spinOnce();
                double edge3=sqrt(midDis*midDis+dis15*dis15-2*midDis*dis15*cos(boardAngle*pi/180.0));
                geometry_msgs::Twist vel_msg;
                double diff1=dis15*dis15-(midDis*midDis+edge3*edge3);
                if(midDis==99999)
                    diff1=99999;
                if(std::isinf(midDis)||midDis==0||std::isinf(dis15)||dis15==0)
                    continue;
                if(diff1>eps)
                {
                    ROS_INFO("太右了，调整一些");
                    vel_msg.angular.z=std::max(-150*fabs(diff1),-0.5);
                }
                else if(diff1<-eps)
                {
                    ROS_INFO("太左了，调整一些");
                    vel_msg.angular.z=std::min(150*fabs(diff1),0.5);
                    
                }
                else
                {
                    ROS_INFO("矫正完成");    
                    vel_msg.angular.z=0;
                    vel_pub.publish(vel_msg);
                    break;
                }
                if(fabs(vel_msg.angular.z)<0.05||(lidarCur.toSec()-lidarStart.toSec()>=5.20))//速度很慢时已经调整得很好了，直接停下
                {
                    ROS_INFO("矫正完成");    
                    vel_msg.angular.z=0;
                    vel_pub.publish(vel_msg);
                    break;
                }
                if(cnt==1)
                {
                    double dtheta=acos((edge3*edge3+midDis*midDis-dis15*dis15)/(2.0*edge3*midDis))-pi/2.0;
                    hxjl=midDis*sin(dtheta);
                    ROS_INFO("横向距离:%f",hxjl);
                    cnt=1;
                }
                cnt++;
                vel_pub.publish(vel_msg);
                r.sleep();
            }
            
            myVel.linear.x=0;
            ros::Time start = ros::Time::now();
            int hxfst=0;
            hxjl*=-0.97;
            while(ros::ok())
            {
                ROS_INFO("横向调整中");
                ros::spinOnce();
                
                if(hxjl<0)
                {
                    myVel.linear.y=0.1;
                }
                else
                {
                    myVel.linear.y=-0.1;
                }
                ros::Time cur=ros::Time::now();
                if(hxfst==0)
                    start=ros::Time::now();
                if((cur.toSec()-start.toSec())>10*fabs(hxjl))
                {
                    ROS_INFO("调整完毕");
                    break;
                }
                vel_pub.publish(myVel);
                if(hxfst==0)
                {
                    start=ros::Time::now();
                }
                r.sleep();
                hxfst=1;
            }
            myVel.linear.y=0;
            while(ros::ok())
            {
                ros::spinOnce();
                if(midDis>0.28||midDis==0)
                {
                    myVel.linear.x=0.1;
                }
                else
                {
                    myVel.linear.x=0;
                    break;
                }
                vel_pub.publish(myVel);
                r.sleep();
            }
            int mxCnt=0;
            for(std::map<std::string,int>::iterator it=objCnt.begin();it!=objCnt.end();it++)
            {
                if(it->second>mxCnt)
                    mxCnt=it->second,getObj=it->first;
            }
            std::string ppath=pref;
            ppath+="wyqd";
            ppath+=afz;
            system(ppath.c_str());
            ppath=pref;
            ppath+=getObj;
            ppath+=afz;
            system(ppath.c_str());
        }   


        void greenCallback(const sensor_msgs::ImageConstPtr& msg)
        {
            if(Green>=0)
                return;
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
                // 转换到HSV颜色空间
            Mat hsv;
            cvtColor(cv_ptr->image, hsv, COLOR_BGR2HSV);

            // 定义颜色范围（HSV格式）
            // 红色范围（需要考虑色环的0°和180°两端）
            Scalar lower_red1(0, 70, 50);
            Scalar upper_red1(10, 255, 255);
            Scalar lower_red2(160, 70, 50);
            Scalar upper_red2(180, 255, 255);

            // 绿色范围
            Scalar lower_green(35, 70, 50);
            Scalar upper_green(85, 255, 255);

            // 创建颜色掩膜
            Mat mask_red1, mask_red2, mask_red, mask_green;
            inRange(hsv, lower_red1, upper_red1, mask_red1);
            inRange(hsv, lower_red2, upper_red2, mask_red2);
            bitwise_or(mask_red1, mask_red2, mask_red);  // 合并红色范围
            inRange(hsv, lower_green, upper_green, mask_green);

            // 形态学操作（去噪）
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));
            morphologyEx(mask_red, mask_red, MORPH_OPEN, kernel);
            morphologyEx(mask_green, mask_green, MORPH_OPEN, kernel);

            // 查找轮廓
            std::vector<std::vector<Point>> contours_red, contours_green;
            findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            findContours(mask_green, contours_green, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            ROS_INFO("isGreen():begin judge");
            // 过滤并标记红色区域
            for (size_t i = 0; i < contours_red.size(); i++) {
                double area = contourArea(contours_red[i]);
                if (area > 500) { // 过滤小面积区域
                    ROS_INFO("red light!!!");
                    this->Green=0;
                    Rect rect = boundingRect(contours_red[i]);
                    rectangle(cv_ptr->image, rect, Scalar(0,0,255), 2); // 红色框
                    putText(cv_ptr->image, "Red", Point(rect.x, rect.y-5), 
                            FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,0,255), 2);
                    Green=0;
                }
            }

            // 过滤并标记绿色区域
            for (size_t i = 0; i < contours_green.size(); i++) {
                double area = contourArea(contours_green[i]);
                if (area > 500) {
                    this->Green=1;
                    ROS_INFO("green light!!!");
                    Rect rect = boundingRect(contours_green[i]);
                    rectangle(cv_ptr->image, rect, Scalar(0,255,0), 2); // 绿色框
                    putText(cv_ptr->image, "Green", Point(rect.x, rect.y-5), 
                            FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);
                    Green=1;
                }
            }



        }
        void endCallback(const std_msgs::StringConstPtr&msg)
        {
            if(msg->data[0]=='e')
                isEnd=1;
        }
        void jies()
        {
            ros::Subscriber end_sub=nh.subscribe("/laneline/result",10,&task_node::endCallback,this);
            ros::Rate r(10);
            while(ros::ok())
            {
                ros::spinOnce();
                ROS_INFO("waitting laneline");
                if(isEnd)
                    break;
                r.sleep();
            }
            std::string ppath=pref;
            ppath+="wc1";
            ppath+=afz;
            system(ppath.c_str());

            ppath=pref,ppath+=getObj,ppath+=afz;
            system(ppath.c_str());

            ppath=pref,ppath+="and",ppath+=afz;
            system(ppath.c_str());
            ppath=pref;
            ppath+=simObj,money-=mp[simObj];
            money-=mp[getObj];
            int sum=20-money;
            ppath+=afz;
            system(ppath.c_str());

            ppath=pref,ppath+="zjhf",ppath+=afz,system(ppath.c_str());

            ppath=pref;
            std::string num;
            while(sum)
            {
                num=char(sum%10+'0')+num;
                sum/=10;
            }
            ppath+=num,ppath+=afz,system(ppath.c_str());
            ppath=pref,ppath+="y",ppath+=afz,system(ppath.c_str());

            ppath=pref,ppath+="xzl",ppath+=afz,system(ppath.c_str());

            ppath=pref;
            num.clear();
            while(money)
            {
                num=char(money%10+'0')+num;
                money/=10;
            }
            ppath+=num;
            ppath+=afz;
            system(ppath.c_str());
            ppath=pref,ppath+="y",ppath+=afz,system(ppath.c_str());
        }
        bool isGreen()
        {
            ROS_WARN("isGreen isGreen isGreen");
            Green=-1;
            ros::Subscriber image_sub=nh.subscribe("/ucar_camera/image_raw",1,&task_node::greenCallback,this);
            ros::Rate r(2);
            
            for(int i=0;i<3;i++)
            {
                ROS_INFO("enter isGreen():%d",i);
                ros::spinOnce();

                r.sleep();
            }
                

            return 0;
        }
        void playWay()
        {
            std::string ppath=pref;
            if(Green==1)
            {
                ppath+="lk1";
            }
            else
                ppath+="lk2";
            ppath+=afz;
            system(ppath.c_str());
        }






};