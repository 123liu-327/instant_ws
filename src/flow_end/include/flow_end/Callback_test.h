#ifndef FLOW_END_CALLBACK_TEST_H
#define FLOW_END_CALLBACK_TEST_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <string>

namespace flow_end {
namespace callback_test {

void refreshRuntimeParams();

void advertiseTopics(ros::NodeHandle &nh, const std::string &cmd_vel_topic,
                     const std::string &end_topic);
void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void beginCallback(const std_msgs::String::ConstPtr &msg);

void subscribeTopics(ros::NodeHandle &nh, const std::string &image_topic,
                     const std::string &imu_topic, const std::string &odom_topic,
                     const std::string &begin_topic);

}  // namespace callback_test
}  // namespace flow_end

#endif  // FLOW_END_CALLBACK_TEST_H
