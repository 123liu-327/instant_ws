#include<ros/ros.h>
#include<std_msgs/String.h>
int main(int argc,char *argv[])
{
    ros::init(argc,argv, "testbe_node");
    ros::NodeHandle nh;
    ros::Publisher start_pub;
    ros::Rate r(10);
    start_pub=nh.advertise<std_msgs::String>("/qr_node_start",10);
    while(ros::ok())
    {
        std_msgs::String msgs;
        msgs.data="start!";
        start_pub.publish<std_msgs::String>(msgs);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}