#include <flow_end/follow_line_test.h>

// follow_test.cpp 只保留 ROS 节点入口和参数读取。
// 具体巡线、角点检测、停车等业务逻辑全部放到 follow_line_test.cpp，
// 这样后续修改地图策略时，不需要反复改 main 和话题初始化部分。
int main(int argc, char **argv) {
    ros::init(argc, argv, "follow_test");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 这些参数都可以在 follow_test.launch 中覆盖，默认值保持和原 flow_end 包一致。
    std::string image_topic;
    std::string imu_topic;
    std::string odom_topic;
    std::string cmd_vel_topic;
    std::string begin_topic;
    std::string end_topic;
    std::string path_param;
    bool publish_debug_image = true;
    bool show_window = false;
    bool parking_enabled = true;
    double base_speed = 0.30;
    double aim_distance = 0.10;
    double aim_y_bias_m = 0.20;
    bool initial_turn_enabled = true;
    double initial_turn_angle_deg = 30.0;
    double initial_turn_angular_speed = 0.35;
    int initial_turn_rpts_threshold = 40;
    double initial_turn_pause_sec = 0.5;

    // 话题参数：把节点和外部系统解耦，换相机话题或控制话题时不用改代码。
    private_nh.param<std::string>("image_topic", image_topic, "/ucar_image/image_raw");
    private_nh.param<std::string>("imu_topic", imu_topic, "/imu");
    private_nh.param<std::string>("odom_topic", odom_topic, "/odom");
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    private_nh.param<std::string>("begin_topic", begin_topic, "/follow_begin");
    private_nh.param<std::string>("end_topic", end_topic, "/follow_end");
    private_nh.param<std::string>("path_select", path_param, "right");
    private_nh.param<bool>("publish_debug_image", publish_debug_image, true);
    private_nh.param<bool>("show_window", show_window, false);
    private_nh.param<bool>("parking_enabled", parking_enabled, true);
    private_nh.param<double>("base_speed", base_speed, 0.30);
    private_nh.param<double>("aim_distance", aim_distance, 0.10);
    private_nh.param<double>("aim_y_bias_m", aim_y_bias_m, 0.20);
    private_nh.param<bool>("initial_turn_enabled", initial_turn_enabled, true);
    private_nh.param<double>("initial_turn_angle_deg", initial_turn_angle_deg, 30.0);
    private_nh.param<double>("initial_turn_angular_speed", initial_turn_angular_speed, 0.35);
    private_nh.param<int>("initial_turn_rpts_threshold", initial_turn_rpts_threshold, 40);
    private_nh.param<double>("initial_turn_pause_sec", initial_turn_pause_sec, 0.5);

    // 将参数传给具体逻辑文件。入口文件不直接访问巡线内部变量。
    flow_end::follow_test::configure(publish_debug_image, show_window, parking_enabled,
                                     base_speed, aim_distance, aim_y_bias_m,
                                     initial_turn_enabled, initial_turn_angle_deg,
                                     initial_turn_angular_speed, initial_turn_rpts_threshold,
                                     initial_turn_pause_sec);
    // path_select 支持 left/middle/right。非法值默认回退到 right，避免节点直接退出。
    if (!flow_end::follow_test::setPathSelect(path_param)) {
        ROS_WARN("Invalid path_select param '%s', fallback to right.", path_param.c_str());
        flow_end::follow_test::setPathSelect("right");
    }

    // 初始化视觉处理查找表，并注册发布器/订阅器。
    flow_end::follow_test::initializeImagePipeline();
    flow_end::follow_test::advertiseTopics(nh, cmd_vel_topic, end_topic);
    flow_end::follow_test::subscribeTopics(nh, image_topic, imu_topic, odom_topic, begin_topic);

    flow_end::follow_test::publishStatus("IDLE");
    ROS_WARN("follow_test started. path_select=%s, waiting for %s Left/Middle/Right.",
             flow_end::follow_test::currentPathName().c_str(), begin_topic.c_str());

    ros::Rate loop_rate(30);
    while (ros::ok() && !flow_end::follow_test::shouldExit()) {
        ros::spinOnce();
        // followLineTestOnce() 内部会检查 run_car。
        // 未收到 /follow_begin 时不会发布运动控制，只保持节点待命。
        flow_end::follow_test::followLineTestOnce();
        loop_rate.sleep();
    }

    flow_end::follow_test::shutdown();
    return 0;
}
