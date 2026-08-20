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

}  // namespace

void refreshRuntimeParams() {
    ros::NodeHandle private_nh("~");

    bool publish_debug = follow_test::publish_debug_image;
    bool show_debug_window = follow_test::show_window;
    bool enable_parking = follow_test::parking_enabled;
    bool allow_either_l = follow_test::parking_allow_either_l;
    double extra_parking_dist = follow_test::parking_extra_dist;
    double forward_parking_speed = follow_test::parking_forward_speed;
    double lateral_parking_speed = follow_test::parking_lateral_speed;
    double lateral_parking_deadband = follow_test::parking_lateral_deadband;
    double lateral_cmd_sign = follow_test::parking_lateral_cmd_sign;
    std::string parking_mode = follow_test::parking_motion_mode;
    double max_parking_angular_speed = follow_test::parking_max_angular_speed;
    double second_arc_max_parking_angular_speed =
        follow_test::parking_second_arc_max_angular_speed;
    double parking_heading_kp = follow_test::parking_yaw_kp;
    double parking_heading_tolerance_deg = follow_test::parking_yaw_tolerance_deg;
    double parking_timeout_sec = follow_test::parking_timeout;
    double parking_odom_timeout_sec = follow_test::parking_odom_timeout;
    double speed = follow_test::base_speed;
    double distance = follow_test::aim_distance;
    double y_bias_m = follow_test::aim_y_bias_m;
    bool enable_right_turn_assist = follow_test::right_turn_assist_enabled;
    double right_min_aim_distance = follow_test::right_turn_min_aim_distance;
    double right_error_start = follow_test::right_turn_error_start;
    double right_error_full = follow_test::right_turn_error_full;
    double right_min_speed = follow_test::right_turn_min_speed;
    double right_wz_compensation = follow_test::right_turn_wz_compensation;
    double right_max_wz = follow_test::right_turn_max_wz;
    double right_odom_timeout = follow_test::right_turn_odom_timeout;
    double right_odom_response_ratio = follow_test::right_turn_odom_response_ratio;
    int right_odom_confirm_frames = follow_test::right_turn_odom_confirm_frames;
    std::string vision_source = follow_test::vision_source;
    double line_track_timeout = follow_test::line_track_timeout;
    double line_track_min_confidence = follow_test::line_track_min_confidence;
    int line_track_min_points = follow_test::line_track_min_points;
    bool enable_initial_turn = follow_test::initial_turn_enabled;
    double turn_angle_deg = follow_test::initial_turn_angle_deg;
    double turn_angular_speed = follow_test::initial_turn_angular_speed;
    int turn_rpts_threshold = follow_test::initial_turn_rpts_threshold;
    double turn_pause_sec = follow_test::initial_turn_pause_sec;
    double min_turn_pid_speed = follow_test::min_pid_speed;
    bool enable_lost_corner_search = follow_test::lost_corner_search_enabled;
    double lost_corner_timeout = follow_test::lost_corner_search_timeout;
    double lost_corner_angular_speed = follow_test::lost_corner_search_angular_speed;
    double lost_corner_linear_speed = follow_test::lost_corner_search_linear_speed;
    double branch_turn_angle_deg = follow_test::y_turn_angle_deg;
    double branch_turn_angular_speed = follow_test::y_turn_angular_speed;
    double branch_turn_pause_sec = follow_test::y_turn_pause_sec;
    int branch_detect_min_id = follow_test::y_detect_min_id;
    int branch_detect_max_id = follow_test::y_detect_max_id;
    int branch_detect_confirm_frames = follow_test::y_detect_confirm_frames;
    double branch_extra_forward_dist = follow_test::y_extra_forward_dist;
    double branch_hard_drive_speed = follow_test::y_hard_drive_speed;
    double branch_hard_drive_odom_timeout = follow_test::y_hard_drive_odom_timeout;
    double branch_hard_drive_max_duration = follow_test::y_hard_drive_max_duration;
    int branch_guided_min_points = follow_test::y_guided_min_points;
    int branch_guided_lost_confirm_frames = follow_test::y_guided_lost_confirm_frames;
    double branch_guided_error_threshold = follow_test::y_guided_error_threshold;
    int branch_guided_error_confirm_frames = follow_test::y_guided_error_confirm_frames;
    double branch_guided_odom_timeout = follow_test::y_guided_odom_timeout;
    double branch_guided_max_duration = follow_test::y_guided_max_duration;
    double branch_hard_heading_kp = follow_test::y_hard_heading_kp;
    double branch_hard_heading_max_wz = follow_test::y_hard_heading_max_wz;
    double branch_hard_heading_deadband_deg = follow_test::y_hard_heading_deadband_deg;
    double branch_hard_heading_imu_timeout = follow_test::y_hard_heading_imu_timeout;
    double branch_reacquire_speed = follow_test::y_reacquire_speed;
    double branch_reacquire_max_dist = follow_test::y_reacquire_max_dist;
    double branch_reacquire_odom_timeout = follow_test::y_reacquire_odom_timeout;
    double branch_reacquire_max_duration = follow_test::y_reacquire_max_duration;
    int branch_reacquire_min_points = follow_test::y_reacquire_min_points;
    int branch_reacquire_confirm_frames = follow_test::y_reacquire_confirm_frames;
    double branch_crossbar_seek_speed = follow_test::y_crossbar_seek_speed;
    int branch_crossbar_lost_confirm_frames = follow_test::y_crossbar_lost_confirm_frames;
    double branch_crossbar_target_long_m = follow_test::y_crossbar_target_long_m;
    double branch_crossbar_long_tolerance_m = follow_test::y_crossbar_long_tolerance_m;
    double branch_crossbar_max_abs_lat_m = follow_test::y_crossbar_max_abs_lat_m;
    int branch_crossbar_confirm_frames = follow_test::y_crossbar_confirm_frames;
    double branch_crossbar_seek_max_odom = follow_test::y_crossbar_seek_max_odom;
    follow_test::MotionControlConfig motion_config;

    private_nh.param<bool>("publish_debug_image", publish_debug, publish_debug);
    private_nh.param<bool>("show_window", show_debug_window, show_debug_window);
    private_nh.param<bool>("parking_enabled", enable_parking, enable_parking);
    private_nh.param<bool>("parking_allow_either_l", allow_either_l, allow_either_l);
    private_nh.param<double>("parking_extra_dist", extra_parking_dist, extra_parking_dist);
    private_nh.param<double>("parking_forward_speed", forward_parking_speed, forward_parking_speed);
    private_nh.param<double>("parking_lateral_speed", lateral_parking_speed, lateral_parking_speed);
    private_nh.param<double>("parking_lateral_deadband", lateral_parking_deadband, lateral_parking_deadband);
    private_nh.param<double>("parking_lateral_cmd_sign", lateral_cmd_sign, lateral_cmd_sign);
    private_nh.param<std::string>("parking_motion_mode", parking_mode, parking_mode);
    private_nh.param<double>("parking_max_angular_speed", max_parking_angular_speed, max_parking_angular_speed);
    private_nh.param<double>("parking_second_arc_max_angular_speed",
                             second_arc_max_parking_angular_speed,
                             second_arc_max_parking_angular_speed);
    private_nh.param<double>("parking_yaw_kp", parking_heading_kp, parking_heading_kp);
    private_nh.param<double>("parking_yaw_tolerance_deg", parking_heading_tolerance_deg, parking_heading_tolerance_deg);
    private_nh.param<double>("parking_timeout", parking_timeout_sec, parking_timeout_sec);
    private_nh.param<double>("parking_odom_timeout", parking_odom_timeout_sec, parking_odom_timeout_sec);
    private_nh.param<double>("base_speed", speed, speed);
    private_nh.param<double>("aim_distance", distance, distance);
    private_nh.param<double>("aim_y_bias_m", y_bias_m, y_bias_m);
    private_nh.param<bool>("right_turn_assist_enabled", enable_right_turn_assist,
                           enable_right_turn_assist);
    private_nh.param<double>("right_turn_min_aim_distance", right_min_aim_distance,
                             right_min_aim_distance);
    private_nh.param<double>("right_turn_error_start", right_error_start,
                             right_error_start);
    private_nh.param<double>("right_turn_error_full", right_error_full,
                             right_error_full);
    private_nh.param<double>("right_turn_min_speed", right_min_speed,
                             right_min_speed);
    private_nh.param<double>("right_turn_wz_compensation", right_wz_compensation,
                             right_wz_compensation);
    private_nh.param<double>("right_turn_max_wz", right_max_wz,
                             right_max_wz);
    private_nh.param<double>("right_turn_odom_timeout", right_odom_timeout,
                             right_odom_timeout);
    private_nh.param<double>("right_turn_odom_response_ratio",
                             right_odom_response_ratio,
                             right_odom_response_ratio);
    private_nh.param<int>("right_turn_odom_confirm_frames",
                          right_odom_confirm_frames,
                          right_odom_confirm_frames);
    private_nh.param<std::string>("vision_source", vision_source, vision_source);
    private_nh.param<double>("line_track_timeout", line_track_timeout,
                             line_track_timeout);
    private_nh.param<double>("line_track_min_confidence", line_track_min_confidence,
                             line_track_min_confidence);
    private_nh.param<int>("line_track_min_points", line_track_min_points,
                          line_track_min_points);
    private_nh.param<bool>("initial_turn_enabled", enable_initial_turn, enable_initial_turn);
    private_nh.param<double>("initial_turn_angle_deg", turn_angle_deg, turn_angle_deg);
    private_nh.param<double>("initial_turn_angular_speed", turn_angular_speed, turn_angular_speed);
    private_nh.param<int>("initial_turn_rpts_threshold", turn_rpts_threshold, turn_rpts_threshold);
    private_nh.param<double>("initial_turn_pause_sec", turn_pause_sec, turn_pause_sec);
    private_nh.param<double>("min_pid_speed", min_turn_pid_speed, min_turn_pid_speed);
    private_nh.param<bool>("lost_corner_search_enabled", enable_lost_corner_search, enable_lost_corner_search);
    private_nh.param<double>("lost_corner_search_timeout", lost_corner_timeout, lost_corner_timeout);
    private_nh.param<double>("lost_corner_search_angular_speed", lost_corner_angular_speed, lost_corner_angular_speed);
    private_nh.param<double>("lost_corner_search_linear_speed", lost_corner_linear_speed, lost_corner_linear_speed);
    private_nh.param<double>("y_turn_angle_deg", branch_turn_angle_deg, branch_turn_angle_deg);
    private_nh.param<double>("y_turn_angular_speed", branch_turn_angular_speed, branch_turn_angular_speed);
    private_nh.param<double>("y_turn_pause_sec", branch_turn_pause_sec, branch_turn_pause_sec);
    private_nh.param<int>("y_detect_min_id", branch_detect_min_id, branch_detect_min_id);
    private_nh.param<int>("y_detect_max_id", branch_detect_max_id, branch_detect_max_id);
    private_nh.param<int>("y_detect_confirm_frames", branch_detect_confirm_frames, branch_detect_confirm_frames);
    private_nh.param<double>("y_extra_forward_dist", branch_extra_forward_dist,
                             branch_extra_forward_dist);
    private_nh.param<double>("y_hard_drive_speed", branch_hard_drive_speed,
                             branch_hard_drive_speed);
    private_nh.param<double>("y_hard_drive_odom_timeout", branch_hard_drive_odom_timeout,
                             branch_hard_drive_odom_timeout);
    private_nh.param<double>("y_hard_drive_max_duration", branch_hard_drive_max_duration,
                             branch_hard_drive_max_duration);
    private_nh.param<int>("y_guided_min_points", branch_guided_min_points,
                          branch_guided_min_points);
    private_nh.param<int>("y_guided_lost_confirm_frames", branch_guided_lost_confirm_frames,
                          branch_guided_lost_confirm_frames);
    private_nh.param<double>("y_guided_error_threshold", branch_guided_error_threshold,
                             branch_guided_error_threshold);
    private_nh.param<int>("y_guided_error_confirm_frames", branch_guided_error_confirm_frames,
                          branch_guided_error_confirm_frames);
    private_nh.param<double>("y_guided_odom_timeout", branch_guided_odom_timeout,
                             branch_guided_odom_timeout);
    private_nh.param<double>("y_guided_max_duration", branch_guided_max_duration,
                             branch_guided_max_duration);
    private_nh.param<double>("y_hard_heading_kp", branch_hard_heading_kp,
                             branch_hard_heading_kp);
    private_nh.param<double>("y_hard_heading_max_wz", branch_hard_heading_max_wz,
                             branch_hard_heading_max_wz);
    private_nh.param<double>("y_hard_heading_deadband_deg", branch_hard_heading_deadband_deg,
                             branch_hard_heading_deadband_deg);
    private_nh.param<double>("y_hard_heading_imu_timeout", branch_hard_heading_imu_timeout,
                             branch_hard_heading_imu_timeout);
    private_nh.param<double>("y_reacquire_speed", branch_reacquire_speed,
                             branch_reacquire_speed);
    private_nh.param<double>("y_reacquire_max_dist", branch_reacquire_max_dist,
                             branch_reacquire_max_dist);
    private_nh.param<double>("y_reacquire_odom_timeout", branch_reacquire_odom_timeout,
                             branch_reacquire_odom_timeout);
    private_nh.param<double>("y_reacquire_max_duration", branch_reacquire_max_duration,
                             branch_reacquire_max_duration);
    private_nh.param<int>("y_reacquire_min_points", branch_reacquire_min_points,
                          branch_reacquire_min_points);
    private_nh.param<int>("y_reacquire_confirm_frames", branch_reacquire_confirm_frames,
                          branch_reacquire_confirm_frames);
    private_nh.param<double>("y_crossbar_seek_speed", branch_crossbar_seek_speed, branch_crossbar_seek_speed);
    private_nh.param<int>("y_crossbar_lost_confirm_frames", branch_crossbar_lost_confirm_frames, branch_crossbar_lost_confirm_frames);
    private_nh.param<double>("y_crossbar_target_long_m", branch_crossbar_target_long_m, branch_crossbar_target_long_m);
    private_nh.param<double>("y_crossbar_long_tolerance_m", branch_crossbar_long_tolerance_m, branch_crossbar_long_tolerance_m);
    private_nh.param<double>("y_crossbar_max_abs_lat_m", branch_crossbar_max_abs_lat_m, branch_crossbar_max_abs_lat_m);
    private_nh.param<int>("y_crossbar_confirm_frames", branch_crossbar_confirm_frames, branch_crossbar_confirm_frames);
    private_nh.param<double>("y_crossbar_seek_max_odom", branch_crossbar_seek_max_odom, branch_crossbar_seek_max_odom);

    private_nh.param<int>("control_path_smooth_window", motion_config.path_smooth_window, motion_config.path_smooth_window);
    private_nh.param<double>("control_path_ema_alpha", motion_config.path_ema_alpha, motion_config.path_ema_alpha);
    private_nh.param<double>("control_error_filter_alpha", motion_config.error_filter_alpha, motion_config.error_filter_alpha);
    private_nh.param<double>("control_yaw_deadband", motion_config.yaw_deadband, motion_config.yaw_deadband);
    private_nh.param<double>("control_kp_yaw", motion_config.kp_yaw, motion_config.kp_yaw);
    private_nh.param<double>("control_ki_yaw", motion_config.ki_yaw, motion_config.ki_yaw);
    private_nh.param<double>("control_kd_yaw", motion_config.kd_yaw, motion_config.kd_yaw);
    private_nh.param<double>("control_integral_limit", motion_config.integral_limit, motion_config.integral_limit);
    private_nh.param<double>("control_integral_error_threshold", motion_config.integral_error_threshold, motion_config.integral_error_threshold);
    private_nh.param<double>("control_adaptive_error_threshold", motion_config.adaptive_error_threshold, motion_config.adaptive_error_threshold);
    private_nh.param<double>("control_adaptive_kp_scale", motion_config.adaptive_kp_scale, motion_config.adaptive_kp_scale);
    private_nh.param<double>("control_adaptive_kd_scale", motion_config.adaptive_kd_scale, motion_config.adaptive_kd_scale);
    private_nh.param<double>("control_max_wz", motion_config.max_wz, motion_config.max_wz);
    private_nh.param<double>("control_soft_wz_limit", motion_config.soft_wz_limit, motion_config.soft_wz_limit);
    private_nh.param<double>("control_max_wz_rate", motion_config.max_wz_rate, motion_config.max_wz_rate);
    private_nh.param<double>("control_turn_slowdown", motion_config.turn_slowdown, motion_config.turn_slowdown);
    private_nh.param<double>("control_slow_error", motion_config.slow_error, motion_config.slow_error);
    private_nh.param<double>("control_min_speed", motion_config.min_speed, motion_config.min_speed);
    private_nh.param<double>("control_degraded_speed_scale", motion_config.degraded_speed_scale, motion_config.degraded_speed_scale);
    private_nh.param<double>("control_max_accel", motion_config.max_accel, motion_config.max_accel);
    private_nh.param<double>("control_max_decel", motion_config.max_decel, motion_config.max_decel);
    private_nh.param<double>("control_cmd_filter_alpha", motion_config.cmd_filter_alpha, motion_config.cmd_filter_alpha);
    private_nh.param<double>("lost_line_coast_sec", motion_config.lost_line_coast_sec, motion_config.lost_line_coast_sec);
    private_nh.param<double>("lost_line_coast_speed_scale", motion_config.lost_line_coast_speed_scale, motion_config.lost_line_coast_speed_scale);

    follow_test::configure(publish_debug, show_debug_window, enable_parking,
                           speed, distance, y_bias_m, enable_initial_turn,
                           turn_angle_deg, turn_angular_speed,
                           turn_rpts_threshold, turn_pause_sec,
                           min_turn_pid_speed, allow_either_l, extra_parking_dist,
                           forward_parking_speed, lateral_parking_speed,
                           lateral_parking_deadband, lateral_cmd_sign,
                           parking_mode, max_parking_angular_speed,
                           second_arc_max_parking_angular_speed,
                           parking_heading_kp, parking_heading_tolerance_deg,
                           parking_timeout_sec, parking_odom_timeout_sec,
                           enable_lost_corner_search, lost_corner_timeout,
                           lost_corner_angular_speed, lost_corner_linear_speed);
    follow_test::configureRightTurnAssist(
        enable_right_turn_assist, right_min_aim_distance,
        right_error_start, right_error_full, right_min_speed,
        right_wz_compensation, right_max_wz, right_odom_timeout,
        right_odom_response_ratio, right_odom_confirm_frames);
    follow_test::configureVisionSource(vision_source, line_track_timeout,
                                       line_track_min_confidence,
                                       line_track_min_points);
    follow_test::configureYBranch(
        branch_turn_angle_deg,
        branch_turn_angular_speed, branch_turn_pause_sec,
        branch_detect_min_id, branch_detect_max_id,
        branch_detect_confirm_frames,
        branch_extra_forward_dist, branch_hard_drive_speed,
        branch_hard_drive_odom_timeout, branch_hard_drive_max_duration,
        branch_guided_min_points, branch_guided_lost_confirm_frames,
        branch_guided_error_threshold, branch_guided_error_confirm_frames,
        branch_guided_odom_timeout, branch_guided_max_duration,
        branch_hard_heading_kp, branch_hard_heading_max_wz,
        branch_hard_heading_deadband_deg, branch_hard_heading_imu_timeout,
        branch_reacquire_speed, branch_reacquire_max_dist,
        branch_reacquire_odom_timeout, branch_reacquire_max_duration,
        branch_reacquire_min_points, branch_reacquire_confirm_frames,
        branch_crossbar_seek_speed, branch_crossbar_lost_confirm_frames,
        branch_crossbar_target_long_m, branch_crossbar_long_tolerance_m,
        branch_crossbar_max_abs_lat_m, branch_crossbar_confirm_frames,
        branch_crossbar_seek_max_odom);
    follow_test::configureMotionController(motion_config);
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
    follow_test::last_imu_time = ros::Time::now();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static bool has_origin = false;
    static float x0 = 0.0f;
    static float y0 = 0.0f;

    const float x_now = msg->pose.pose.position.x;
    const float y_now = msg->pose.pose.position.y;
    current_linear_velocity_x = msg->twist.twist.linear.x;
    follow_test::current_odom_angular_velocity_z = msg->twist.twist.angular.z;
    follow_test::current_odom_position_x = x_now;
    follow_test::current_odom_position_y = y_now;
    follow_test::last_odom_time = ros::Time::now();

    static bool has_total_origin = false;
    static double total_last_x = 0.0;
    static double total_last_y = 0.0;
    if (has_total_origin) {
        const double step = std::hypot(
            static_cast<double>(x_now) - total_last_x,
            static_cast<double>(y_now) - total_last_y);
        if (std::isfinite(step) && step <= 0.50) {
            follow_test::odom_total_distance += step;
        } else {
            ROS_WARN_THROTTLE(
                1.0, "[ODOM] Ignore invalid cumulative-distance step %.3fm", step);
        }
    }
    total_last_x = x_now;
    total_last_y = y_now;
    has_total_origin = true;

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
        follow_test::resetYBranchState();
        follow_test::resetParkingCornerState();
        follow_test::resetExternalVisionState();
        follow_test::publishStop();
        follow_test::publishStatus("IDLE");
        ROS_WARN("[CMD] 停车指令 | command=%s | 状态=IDLE", msg->data.c_str());
        return;
    }

    if (!follow_test::setPathSelect(msg->data)) {
        ROS_WARN("[CMD] 未知指令 | command=%s", msg->data.c_str());
        return;
    }

    follow_test::applyPathBiasParams(follow_test::path_select);
    run_car = true;
    zeroCount = 0;
    zero_flag = false;
    follow_test::resetParkingCornerState();
    follow_test::resetExternalVisionState();
    follow_test::startInitialTurnIfNeeded();
    
    // 启动调试信息
    ROS_WARN("[CMD] StartFollow | path=%s | bias_left=%.1f | bias_right=%.1f | Time_local=%.2f | init_turn=%d | parking=%d | allow_either_l=%d | parking_extra=%.3f | park_mode=%s | park_vx=%.2f | park_vy=%.2f | park_max_wz=%.2f | park_second_max_wz=%.2f | lat_deadband=%.3f | y_sign=%.0f | lost_search=%d | base_speed=%.2f m/s | init_angle=%.1f deg | rpts_thresh=%d | min_pid_speed=%.2f",
         pathToString(follow_test::path_select).c_str(),
         Dis_Bias_Left, Dis_Bias_Right,
         Time_local,
         follow_test::initial_turn_enabled,
         follow_test::parking_enabled,
         follow_test::parking_allow_either_l,
         follow_test::parking_extra_dist,
         follow_test::parking_motion_mode.c_str(),
         follow_test::parking_forward_speed,
         follow_test::parking_lateral_speed,
         follow_test::parking_max_angular_speed,
         follow_test::parking_second_arc_max_angular_speed,
         follow_test::parking_lateral_deadband,
         follow_test::parking_lateral_cmd_sign,
         follow_test::lost_corner_search_enabled,
         follow_test::base_speed,
         follow_test::initial_turn_angle_deg,
         follow_test::initial_turn_rpts_threshold,
         follow_test::min_pid_speed);
}

void lineTrackCallback(const line_follower::LineTrack::ConstPtr &msg) {
    follow_test::updateLineTrack(msg);
}

void subscribeTopics(ros::NodeHandle &nh, const std::string &image_topic,
                     const std::string &imu_topic, const std::string &odom_topic,
                     const std::string &begin_topic,
                     const std::string &line_track_topic) {
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
    subscribers.push_back(nh.subscribe(line_track_topic, image_queue_size, lineTrackCallback));
}

}  // namespace callback_test
}  // namespace flow_end
