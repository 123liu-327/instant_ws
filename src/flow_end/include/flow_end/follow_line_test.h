#ifndef FLOW_END_FOLLOW_LINE_TEST_H
#define FLOW_END_FOLLOW_LINE_TEST_H

#include <ros/ros.h>

#include <string>

namespace flow_end {
namespace follow_test {

// 将 launch 参数或 /follow_begin 指令中的 left/middle/right 转成内部路径模式。
bool setPathSelect(const std::string &raw_value);
// 返回当前路径模式名称，用于启动日志和调试状态输出。
std::string currentPathName();
// 从 follow_test.cpp 注入运行参数。这样入口文件只管读参数，具体逻辑仍放在
// follow_line_test.cpp 中，后续改地图逻辑时不需要改 main。
void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double initial_turn_angle_deg,
               double initial_turn_angular_speed, int initial_turn_rpts_threshold,
               double initial_turn_pause_sec);
// 初始化视觉处理需要的逆透视查找表，对应原工程中的 ImagePerspective_Init()。
void initializeImagePipeline();
// 注册控制、结束、状态和调试图像发布器。
void advertiseTopics(ros::NodeHandle &nh, const std::string &cmd_vel_topic,
                     const std::string &end_topic);
// 注册图像、IMU、里程计和启动指令订阅器。
void subscribeTopics(ros::NodeHandle &nh, const std::string &image_topic,
                     const std::string &imu_topic, const std::string &odom_topic,
                     const std::string &begin_topic);
// 发布当前测试节点状态，例如 IDLE、RUNNING_right、PARKING、FINISHED。
void publishStatus(const std::string &state);
// 连续发布零速度，保证底盘可靠停车。
void publishStop();
// 单次巡线处理：取最新图像、提线、选路径、角点停车判断、发布 /cmd_vel。
int followLineTestOnce();
// 预留退出标志接口，目前跟随原工程的 sig_INT。
bool shouldExit();
// 节点退出前停车并关闭 OpenCV 窗口。
void shutdown();

}  // namespace follow_test
}  // namespace flow_end

#endif  // FLOW_END_FOLLOW_LINE_TEST_H
