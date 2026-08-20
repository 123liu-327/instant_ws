#include <flow_end/Callback_test.h>
#include <flow_end/follow_line_test.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_test");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string image_topic;
    std::string imu_topic;
    std::string odom_topic;
    std::string cmd_vel_topic;
    std::string begin_topic;
    std::string end_topic;
    std::string path_param;
    std::string line_track_topic;
    bool publish_debug_image = true;
    bool show_window = false;
    bool parking_enabled = true;
    bool parking_allow_either_l = true;
    double parking_extra_dist = 0.215;
    double parking_forward_speed = 0.20;
    double parking_lateral_speed = 0.10;
    double parking_lateral_deadband = 0.03;
    double parking_lateral_cmd_sign = 1.0;
    std::string parking_motion_mode = "s_curve";
    double parking_max_angular_speed = 0.35;
    double parking_second_arc_max_angular_speed = 0.11;
    double parking_yaw_kp = 1.5;
    double parking_yaw_tolerance_deg = 3.0;
    double parking_timeout = 6.0;
    double parking_odom_timeout = 0.5;
    double base_speed = 0.30;
    double aim_distance = 0.10;
    double aim_y_bias_m = 0.20;
    bool initial_turn_enabled = true;
    double initial_turn_angle_deg = 30.0;
    double initial_turn_angular_speed = 0.35;
    int initial_turn_rpts_threshold = 40;
    double initial_turn_pause_sec = 0.5;
    double min_pid_speed = 0.08;
    bool lost_corner_search_enabled = true;
    double lost_corner_search_timeout = 0.6;
    double lost_corner_search_angular_speed = 0.25;
    double lost_corner_search_linear_speed = 0.0;
    
    // 视频保存相关参数
    bool enable_video_record = true;
    int video_fps = 10;
    std::string video_save_path = "";

    private_nh.param<std::string>("image_topic", image_topic, "/ucar_image/image_raw");
    private_nh.param<std::string>("imu_topic", imu_topic, "/imu");
    private_nh.param<std::string>("odom_topic", odom_topic, "/odom");
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    private_nh.param<std::string>("begin_topic", begin_topic, "/follow_begin");
    private_nh.param<std::string>("end_topic", end_topic, "/follow_end");
    private_nh.param<std::string>("path_select", path_param, "right");
    private_nh.param<std::string>("line_track_topic", line_track_topic, "/line_follower/track");
    private_nh.param<bool>("publish_debug_image", publish_debug_image, true);
    private_nh.param<bool>("show_window", show_window, false);
    private_nh.param<bool>("parking_enabled", parking_enabled, true);
    private_nh.param<bool>("parking_allow_either_l", parking_allow_either_l, true);
    private_nh.param<double>("parking_extra_dist", parking_extra_dist, 0.215);
    private_nh.param<double>("parking_forward_speed", parking_forward_speed, 0.20);
    private_nh.param<double>("parking_lateral_speed", parking_lateral_speed, 0.10);
    private_nh.param<double>("parking_lateral_deadband", parking_lateral_deadband, 0.03);
    private_nh.param<double>("parking_lateral_cmd_sign", parking_lateral_cmd_sign, 1.0);
    private_nh.param<std::string>("parking_motion_mode", parking_motion_mode, "s_curve");
    private_nh.param<double>("parking_max_angular_speed", parking_max_angular_speed, 0.35);
    private_nh.param<double>("parking_second_arc_max_angular_speed",
                             parking_second_arc_max_angular_speed, 0.11);
    private_nh.param<double>("parking_yaw_kp", parking_yaw_kp, 1.5);
    private_nh.param<double>("parking_yaw_tolerance_deg", parking_yaw_tolerance_deg, 3.0);
    private_nh.param<double>("parking_timeout", parking_timeout, 6.0);
    private_nh.param<double>("parking_odom_timeout", parking_odom_timeout, 0.5);
    private_nh.param<double>("base_speed", base_speed, 0.30);
    private_nh.param<double>("aim_distance", aim_distance, 0.10);
    private_nh.param<double>("aim_y_bias_m", aim_y_bias_m, 0.20);
    private_nh.param<bool>("initial_turn_enabled", initial_turn_enabled, true);
    private_nh.param<double>("initial_turn_angle_deg", initial_turn_angle_deg, 30.0);
    private_nh.param<double>("initial_turn_angular_speed", initial_turn_angular_speed, 0.35);
    private_nh.param<int>("initial_turn_rpts_threshold", initial_turn_rpts_threshold, 40);
    private_nh.param<double>("initial_turn_pause_sec", initial_turn_pause_sec, 0.5);
    private_nh.param<double>("min_pid_speed", min_pid_speed, 0.08);
    private_nh.param<bool>("lost_corner_search_enabled", lost_corner_search_enabled, true);
    private_nh.param<double>("lost_corner_search_timeout", lost_corner_search_timeout, 0.6);
    private_nh.param<double>("lost_corner_search_angular_speed", lost_corner_search_angular_speed, 0.25);
    private_nh.param<double>("lost_corner_search_linear_speed", lost_corner_search_linear_speed, 0.0);
    
    // 读取视频保存参数
    private_nh.param<bool>("enable_video_record", enable_video_record, true);
    private_nh.param<int>("video_fps", video_fps, 10);
    private_nh.param<std::string>("video_save_path", video_save_path, "");

    flow_end::follow_test::configure(publish_debug_image, show_window, parking_enabled,
                                     base_speed, aim_distance, aim_y_bias_m,
                                     initial_turn_enabled, initial_turn_angle_deg,
                                     initial_turn_angular_speed, initial_turn_rpts_threshold,
                                     initial_turn_pause_sec, min_pid_speed,
                                     parking_allow_either_l, parking_extra_dist,
                                     parking_forward_speed, parking_lateral_speed,
                                     parking_lateral_deadband, parking_lateral_cmd_sign,
                                     parking_motion_mode, parking_max_angular_speed,
                                     parking_second_arc_max_angular_speed,
                                     parking_yaw_kp, parking_yaw_tolerance_deg,
                                     parking_timeout, parking_odom_timeout,
                                     lost_corner_search_enabled,
                                     lost_corner_search_timeout,
                                     lost_corner_search_angular_speed,
                                     lost_corner_search_linear_speed);
    
    // 配置视频保存
    flow_end::follow_test::configureVideo(enable_video_record, video_fps, video_save_path);

    if (!flow_end::follow_test::setPathSelect(path_param)) {
        ROS_WARN("Invalid path_select param '%s', fallback to right.", path_param.c_str());
        flow_end::follow_test::setPathSelect("right");
    }

    flow_end::follow_test::initializeImagePipeline();
    flow_end::callback_test::advertiseTopics(nh, cmd_vel_topic, end_topic);
    flow_end::callback_test::subscribeTopics(nh, image_topic, imu_topic, odom_topic,
                                             begin_topic, line_track_topic);

    flow_end::follow_test::publishStatus("IDLE");
    ROS_WARN("follow_test started. path_select=%s, waiting for %s Left/Middle/Right.",
             flow_end::follow_test::currentPathName().c_str(), begin_topic.c_str());

    ros::Rate loop_rate(30);
    while (ros::ok() && !flow_end::follow_test::shouldExit()) {
        ros::spinOnce();
        flow_end::follow_test::followLineTestOnce();
        loop_rate.sleep();
    }

    flow_end::follow_test::shutdown();
    return 0;
}
