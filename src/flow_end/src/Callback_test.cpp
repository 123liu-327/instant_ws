#include <flow_end/Callback_test.h>

#include <flow_end/follow_line_test.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>

namespace flow_end {
namespace callback_test {
namespace {

std::string normalizeCommand(std::string value) {
    for (char &c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

bool commandMatchesParamList(const std::string &value, const std::string &param_name,
                             const std::vector<std::string> &defaults) {
    ros::NodeHandle private_nh("~");
    std::vector<std::string> commands;
    if (!private_nh.getParam(param_name, commands) || commands.empty()) {
        commands = defaults;
    }

    const std::string normalized_value = normalizeCommand(value);
    for (std::string command : commands) {
        if (normalized_value == normalizeCommand(command)) {
            return true;
        }
    }
    return false;
}

bool isStopCommand(const std::string &value) {
    return commandMatchesParamList(value, "stop_commands", {"stop", "pause"});
}

std::string pathParamPrefix(follow_test::PathSelect path) {
    switch (path) {
        case follow_test::PathSelect::LEFT:
            return "left";
        case follow_test::PathSelect::MIDDLE:
            return "middle";
        case follow_test::PathSelect::RIGHT:
            return "right";
    }
    return "right";
}

void applyPathBiasParams() {
    ros::NodeHandle private_nh("~");
    const std::string prefix = pathParamPrefix(follow_test::path_select);

    double default_left_bias = 0.0;
    double default_right_bias = 0.0;
    double time_local = Time_local;

    private_nh.param<double>("default_dis_bias_left", default_left_bias, default_left_bias);
    private_nh.param<double>("default_dis_bias_right", default_right_bias, default_right_bias);
    private_nh.param<double>("time_local", time_local, time_local);

    double left_bias = default_left_bias;
    double right_bias = default_right_bias;
    private_nh.param<double>(prefix + "_dis_bias_left", left_bias, left_bias);
    private_nh.param<double>(prefix + "_dis_bias_right", right_bias, right_bias);
    private_nh.param<double>(prefix + "_time_local", time_local, time_local);

    Dis_Bias_Left = static_cast<float>(left_bias);
    Dis_Bias_Right = static_cast<float>(right_bias);
    Time_local = time_local;
}

}  // namespace

void refreshRuntimeParams() {
    ros::NodeHandle private_nh("~");

    bool publish_debug = follow_test::publish_debug_image;
    bool show_debug_window = follow_test::show_window;
    bool enable_parking = follow_test::parking_enabled;
    double speed = follow_test::base_speed;
    double distance = follow_test::aim_distance;
    double y_bias_m = follow_test::aim_y_bias_m;
    bool enable_initial_turn = follow_test::initial_turn_enabled;
    double turn_angle_deg = follow_test::initial_turn_angle_deg;
    double turn_angular_speed = follow_test::initial_turn_angular_speed;
    int turn_rpts_threshold = follow_test::initial_turn_rpts_threshold;
    double turn_pause_sec = follow_test::initial_turn_pause_sec;
    double min_turn_pid_speed = follow_test::min_pid_speed;
    double branch_approach_dist = follow_test::y_approach_dist;
    double branch_turn_angle_deg = follow_test::y_turn_angle_deg;
    double branch_turn_angular_speed = follow_test::y_turn_angular_speed;
    double branch_turn_pause_sec = follow_test::y_turn_pause_sec;
    int branch_detect_max_id = follow_test::y_detect_max_id;
    int branch_detect_confirm_frames = follow_test::y_detect_confirm_frames;

    private_nh.param<bool>("publish_debug_image", publish_debug, publish_debug);
    private_nh.param<bool>("show_window", show_debug_window, show_debug_window);
    private_nh.param<bool>("parking_enabled", enable_parking, enable_parking);
    private_nh.param<double>("base_speed", speed, speed);
    private_nh.param<double>("aim_distance", distance, distance);
    private_nh.param<double>("aim_y_bias_m", y_bias_m, y_bias_m);
    private_nh.param<bool>("initial_turn_enabled", enable_initial_turn, enable_initial_turn);
    private_nh.param<double>("initial_turn_angle_deg", turn_angle_deg, turn_angle_deg);
    private_nh.param<double>("initial_turn_angular_speed", turn_angular_speed, turn_angular_speed);
    private_nh.param<int>("initial_turn_rpts_threshold", turn_rpts_threshold, turn_rpts_threshold);
    private_nh.param<double>("initial_turn_pause_sec", turn_pause_sec, turn_pause_sec);
    private_nh.param<double>("min_pid_speed", min_turn_pid_speed, min_turn_pid_speed);
    private_nh.param<double>("y_approach_dist", branch_approach_dist, branch_approach_dist);
    private_nh.param<double>("y_turn_angle_deg", branch_turn_angle_deg, branch_turn_angle_deg);
    private_nh.param<double>("y_turn_angular_speed", branch_turn_angular_speed, branch_turn_angular_speed);
    private_nh.param<double>("y_turn_pause_sec", branch_turn_pause_sec, branch_turn_pause_sec);
    private_nh.param<int>("y_detect_max_id", branch_detect_max_id, branch_detect_max_id);
    private_nh.param<int>("y_detect_confirm_frames", branch_detect_confirm_frames, branch_detect_confirm_frames);

    follow_test::configure(publish_debug, show_debug_window, enable_parking,
                           speed, distance, y_bias_m, enable_initial_turn,
                           turn_angle_deg, turn_angular_speed,
                           turn_rpts_threshold, turn_pause_sec,
                           min_turn_pid_speed,
                           branch_approach_dist, branch_turn_angle_deg,
                           branch_turn_angular_speed, branch_turn_pause_sec,
                           branch_detect_max_id, branch_detect_confirm_frames);
}

void advertiseTopics(ros::NodeHandle &nh, const std::string &cmd_vel_topic,
                     const std::string &end_topic) {
    ros::NodeHandle private_nh("~");
    std::string status_topic = "/flow_end/follow_test_status";
    std::string debug_topic = "/flow_end/follow_test_debug";

    private_nh.param<std::string>("status_topic", status_topic, status_topic);
    private_nh.param<std::string>("debug_topic", debug_topic, debug_topic);

    pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
    end_pub = nh.advertise<std_msgs::String>(end_topic, 10);
    follow_test::status_pub = nh.advertise<std_msgs::String>(status_topic, 10);
    follow_test::debug_pub = nh.advertise<sensor_msgs::Image>(debug_topic, 1);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat resized;
        cv::resize(cv_ptr->image, resized, cv::Size(RESULT_COL, RESULT_ROW));
        std::lock_guard<std::mutex> lock(frame_mutex);
        frame = resized.clone();
    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = yaw * 180.0 / M_PI;
    curent_wz = msg->angular_velocity.z;
    current_angular_velocity_z = msg->angular_velocity.z;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static bool has_origin = false;
    static float x0 = 0.0f;
    static float y0 = 0.0f;

    const float x_now = msg->pose.pose.position.x;
    const float y_now = msg->pose.pose.position.y;
    current_linear_velocity_x = msg->twist.twist.linear.x;

    if (!has_origin) {
        x0 = x_now;
        y0 = y_now;
        has_origin = true;
        return;
    }

    const float dx = x_now - x0;
    const float dy = y_now - y0;
    odom_dist = std::sqrt(dx * dx + dy * dy);
}

void beginCallback(const std_msgs::String::ConstPtr &msg) {
    refreshRuntimeParams();

    if (isStopCommand(msg->data)) {
        run_car = false;
        follow_test::motion_state = follow_test::MotionState::IDLE;
        follow_test::initial_turn_integrated_angle_deg = 0.0;
        follow_test::initial_turn_has_last_time = false;
        follow_test::resetParkingCornerState();
        follow_test::publishStop();
        follow_test::publishStatus("IDLE");
        ROS_WARN("[CMD] 停车指令 | command=%s | 状态=IDLE", msg->data.c_str());
        return;
    }

    if (!follow_test::setPathSelect(msg->data)) {
        ROS_WARN("[CMD] 未知指令 | command=%s", msg->data.c_str());
        return;
    }

    applyPathBiasParams();
    run_car = true;
    zeroCount = 0;
    zero_flag = false;
    follow_test::resetParkingCornerState();
    follow_test::startInitialTurnIfNeeded();
    
    // 启动调试信息
    ROS_WARN("[CMD] StartFollow | path=%s | bias_left=%.1f | bias_right=%.1f | Time_local=%.2f | init_turn=%d | parking=%d | base_speed=%.2f m/s | init_angle=%.1f deg | rpts_thresh=%d | min_pid_speed=%.2f",
         pathToString(follow_test::path_select).c_str(),
         Dis_Bias_Left, Dis_Bias_Right,
         Time_local,
         follow_test::initial_turn_enabled,
         follow_test::parking_enabled,
         follow_test::base_speed,
         follow_test::initial_turn_angle_deg,
         follow_test::initial_turn_rpts_threshold,
         follow_test::min_pid_speed);
}

void subscribeTopics(ros::NodeHandle &nh, const std::string &image_topic,
                     const std::string &imu_topic, const std::string &odom_topic,
                     const std::string &begin_topic) {
    refreshRuntimeParams();

    ros::NodeHandle private_nh("~");
    int image_queue_size = 1;
    int imu_queue_size = 10;
    int odom_queue_size = 10;
    int begin_queue_size = 10;

    private_nh.param<int>("image_queue_size", image_queue_size, image_queue_size);
    private_nh.param<int>("imu_queue_size", imu_queue_size, imu_queue_size);
    private_nh.param<int>("odom_queue_size", odom_queue_size, odom_queue_size);
    private_nh.param<int>("begin_queue_size", begin_queue_size, begin_queue_size);

    image_queue_size = std::max(1, image_queue_size);
    imu_queue_size = std::max(1, imu_queue_size);
    odom_queue_size = std::max(1, odom_queue_size);
    begin_queue_size = std::max(1, begin_queue_size);

    static std::vector<ros::Subscriber> subscribers;
    subscribers.clear();
    subscribers.push_back(nh.subscribe(image_topic, image_queue_size, imageCallback));
    subscribers.push_back(nh.subscribe(imu_topic, imu_queue_size, imuCallback));
    subscribers.push_back(nh.subscribe(odom_topic, odom_queue_size, odomCallback));
    subscribers.push_back(nh.subscribe(begin_topic, begin_queue_size, beginCallback));
}

}  // namespace callback_test
}  // namespace flow_end
