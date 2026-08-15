#ifndef FLOW_END_CALLBACK_H
#define FLOW_END_CALLBACK_H
#include <flow_end/follow.h>
//雷达回调
void _LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
//摄像头回调
void _CamCallback(const sensor_msgs::ImageConstPtr & msg);
//IMU回调
void _imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
//自定义回调：用于接收巡线方式的回调
void _beginCallback(const std_msgs::String::ConstPtr &msg);
//里程计
void _odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
#endif