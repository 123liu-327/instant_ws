#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
double  midDis,dis15;
const double eps=1e-5;
const double pi=3.14159265359;
const double boardAngle=15;
void LidarCallBack(const sensor_msgs::LaserScan msg)
{
    int front_index = (0 - msg.angle_min) / msg.angle_increment;
    int angle_index =  front_index+(boardAngle*pi/180.0)/msg.angle_increment;
    midDis=msg.ranges[front_index];
    dis15=msg.ranges[angle_index];
    ROS_INFO("雷达测距：midDis=%f,dis15=%f",midDis,dis15);
}
int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"lidar_node");
    ros::NodeHandle n;
    ros::Subscriber lidar_sub=n.subscribe("/scan",10,&LidarCallBack);
    printf("雷达节点启动\n");
    
    
    ros::Publisher vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    ros::Rate r(10);
    r.sleep();
    while(ros::ok())
    {
        ros::spinOnce();
        double edge3=sqrt(midDis*midDis+dis15*dis15-2*midDis*dis15*cos(boardAngle*pi/180.0));
        geometry_msgs::Twist vel_msg;
        double diff=dis15*dis15-(midDis*midDis+edge3*edge3);
        if(diff>eps)
        {
            ROS_INFO("太右了，调整一些");
            vel_msg.angular.z=std::max(-100*fabs(diff),-0.5);
        }
        else if(diff<-eps)
        {
            ROS_INFO("太左了，调整一些");
            vel_msg.angular.z=std::min(100*fabs(diff),0.5);
        }
        else
        {
            ROS_INFO("矫正完成");    
        }
        ROS_INFO("速度：%f",vel_msg.angular.z);
        vel_pub.publish(vel_msg);
        r.sleep();
    }

    ros::spin();
    return 0;
}