#include <flow_end/follow.h>
#include <flow_end/follow_line_test.h>
#include <flow_end/ImagePerspectiveInit.h>
#include <flow_end/MatTransform.h>
#include <flow_end/process_image.h>
#include <flow_end/corner_move.h>
#include <flow_end/PID.h>

#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>

// follow_line_test.cpp 是 follow_test 节点的具体工作逻辑文件。
// 这里保留原 flow_end 视觉巡线算法依赖的全局变量，同时去掉原地图中的环岛和激光雷达流程。
// follow_test.cpp 负责参数和 ROS 入口，本文件负责：
// 1. 接收图像/IMU/里程计/启动指令；
// 2. 调用 process_image() 提取左右边线；
// 3. 根据 left/middle/right 选择控制路径；
// 4. 保留角点检测和停车逻辑；
// 5. 发布 /cmd_vel、状态和调试图像。

// 原工程的视觉算法大量使用全局变量。为了让 follow_test 独立于 follow.cpp/follow_line.cpp
// 编译运行，这里重新定义这些变量，避免链接原来的完整地图逻辑。
double change_un_Mat[3][3] = {{-2.897018, 2.446196, -388.368977},
                              {-0.061836, 1.194630, -756.140464},
                              {-0.000272, 0.008324, -4.335235}};
double invMat[3][3];
int point_map[RESULT_ROW][RESULT_COL][2];
uint8_t *PerImg_ip[RESULT_ROW][RESULT_COL];
uint8_t SimBinImage[RESULT_ROW][RESULT_COL];
float mapx[RESULT_ROW][RESULT_COL];
float mapy[RESULT_ROW][RESULT_COL];
ros::Publisher pub;
ros::Publisher end_pub;

float slope = 0.0f;
float angle_deg = 0.0f;
double current_yaw_lidar = 0.0;
bool is_lidar_update = false;
std::mutex frame_mutex;
cv::Mat frame;
double current_yaw = 0.0;
double curent_wz = 0.0;
enum track_type_e track_type = TRACK_RIGHT;
bool run_car = false;
std::atomic<bool> sig_INT(false);
ros::Time move_start_time;
ros::Time move_start_time_after_laser;
int check = 0;
float Dist_1 = 100.0f;
double pre_yaw = 0.0;
double pre_angle_deg = 0.0;
bool check_after_laser = false;
bool check_imu = false;
int after_bizhang_x = 0;
int after_bizhang_y = 0;
uint8_t test_img[RESULT_ROW][RESULT_COL];

const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

image_t img_raw = DEF_IMAGE(NULL, RESULT_COL, RESULT_ROW);
float begin_x = 25;
float begin_y = 400;
int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num = 0, ipts1_num = 0;
float thres = 30;
float block_size = 7;
float clip_value = 1;
float line_blur_kernel = 7;
float pixel_per_meter = 500;
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num = 0, rpts1_num = 0;
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num = 0, rpts1b_num = 0;
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num = 0, rpts1s_num = 0;
float sample_dist = 0.01;
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num = 0, rpts1a_num = 0;
float angle_dist = 0.1;
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num = 0, rpts1an_num = 0;
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num = 0, rptsc1_num = 0;
float rptsc0e[POINTS_MAX_LEN][2];
float rptsc1e[POINTS_MAX_LEN][2];
int rptsc0e_num = 0, rptsc1e_num = 0;

int Ypt0_rpts0s_id = 0, Ypt1_rpts1s_id = 0;
bool Ypt0_found = false, Ypt1_found = false;
int Lpt0_rpts0s_id = 0, Lpt1_rpts1s_id = 0;
bool Lpt0_found = false, Lpt1_found = false;
const float PI = 3.14159265358979323846f;
bool is_straight0 = false, is_straight1 = false;
float (*rpts)[2] = nullptr;
int rpts_num = 0;
int zeroCount = 0;
bool zero_flag = false;
uint8_t img_line_data[RESULT_ROW][RESULT_COL];
image_t img_line = DEF_IMAGE((uint8_t *)img_line_data, RESULT_COL, RESULT_ROW);
ros::Time Global_move_timer;
float Dis_Bias_Left = 0.0f;
float Dis_Bias_Right = 0.0f;
double current_linear_velocity_x = 0.0;
double current_angular_velocity_z = 0.0;
ros::Time last_imu_time;
bool imu_first_msg = true;
bool check_L_0 = false;
bool check_L_1 = false;
bool local_corner_point = false;
float odom_dist = 0.0f;
bool is_start = false;
double Time_local = 0.5;
int Round_step = 0;
float Round_step1_k = 0.0f;
float angle_rad_step1 = 0.0f;
float angle_deg_step1 = 0.0f;
ros::Time Round_timer(0);
float Laser_linear_dis = 0.0f;
bool Laser_dis_check = false;

namespace flow_end {
namespace follow_test {

// 测试节点的路径选择模式：
// LEFT   使用左侧偏移线 rptsc0e；
// MIDDLE 同时有左右线时取两条偏移线的中点；
// RIGHT  使用右侧偏移线 rptsc1e。
enum class PathSelect { LEFT, MIDDLE, RIGHT };

PathSelect path_select = PathSelect::RIGHT;
enum class MotionState { IDLE, ALIGNING_LEFT, ALIGNING_RIGHT, ALIGN_PAUSE, FOLLOWING };

MotionState motion_state = MotionState::IDLE;
ros::Publisher debug_pub;
ros::Publisher status_pub;
float middle_path[POINTS_MAX_LEN][2];
int middle_path_num = 0;

// 这些运行参数由 follow_test.cpp 从 launch/private param 读入后通过 configure() 写入。
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
double initial_turn_integrated_angle_deg = 0.0;
ros::Time initial_turn_last_time;
bool initial_turn_has_last_time = false;
double  min_pid_speed= 0.08    //                                     
ros::Time initial_turn_pause_start;

std::string normalize(std::string value) {
    // 指令统一转成小写，兼容 Left/left/L 等写法。
    for (char &c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

std::string pathToString(PathSelect path) {
    switch (path) {
        case PathSelect::LEFT: return "left";
        case PathSelect::MIDDLE: return "middle";
        case PathSelect::RIGHT: return "right";
    }
    return "unknown";
}

void publishStatus(const std::string &state);
void publishDebugImage(const sensor_msgs::ImageConstPtr &source_msg = sensor_msgs::ImageConstPtr());

double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

int selectedPathPointCount() {
    if (path_select == PathSelect::LEFT) {
        return rptsc0e_num;
    }
    if (path_select == PathSelect::RIGHT) {
        return rptsc1e_num;
    }
    return std::min(rptsc0e_num, rptsc1e_num);
}

void startInitialTurnIfNeeded() {
    if (!initial_turn_enabled || path_select == PathSelect::MIDDLE) {
        motion_state = MotionState::FOLLOWING;
        publishStatus("RUNNING_" + pathToString(path_select));
        return;
    }

    motion_state = path_select == PathSelect::LEFT ? MotionState::ALIGNING_LEFT : MotionState::ALIGNING_RIGHT;
    // 起步预转角沿用原 follow_line.cpp 的思路：不用 yaw 两帧相减，
    // 而是把 IMU 角速度 wz 对时间积分，得到已经转过的角度。
    initial_turn_integrated_angle_deg = 0.0;
    initial_turn_last_time = ros::Time::now();
    initial_turn_has_last_time = true;
    pid.reset();
    publishStatus(path_select == PathSelect::LEFT ? "ALIGNING_left" : "ALIGNING_right");
}

bool setPathSelect(const std::string &raw_value) {
    const std::string value = normalize(raw_value);
    if (value == "left" || value == "l") {
        path_select = PathSelect::LEFT;
        track_type = TRACK_LEFT;
        return true;
    }
    if (value == "middle" || value == "mid" || value == "center" || value == "centre" || value == "m") {
        path_select = PathSelect::MIDDLE;
        track_type = TRACK_MIDDLE;
        return true;
    }
    if (value == "right" || value == "r") {
        path_select = PathSelect::RIGHT;
        track_type = TRACK_RIGHT;
        return true;
    }
    return false;
}

void publishStop() {
    // 连续发布零速度，比只发一次更可靠，避免底盘控制器错过停车指令。
    geometry_msgs::Twist stop_msg;
    for (int i = 0; i < 10; ++i) {
        pub.publish(stop_msg);
        ros::Duration(0.03).sleep();
    }
}

void publishStatus(const std::string &state) {
    if (!status_pub) {
        return;
    }
    std_msgs::String msg;
    msg.data = state;
    status_pub.publish(msg);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        // 相机输入统一转成 BGR8，并缩放到算法固定尺寸 RESULT_COL x RESULT_ROW。
        // 原视觉算法使用二维数组处理图像，所以这里只缓存 resize 后的 cv::Mat。
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
    // 保留 IMU 姿态信息，方便后续需要按地图状态增加姿态约束。
    // 当前 follow_test 基础巡线控制主要依赖视觉误差。
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = yaw * 180.0 / M_PI;
    curent_wz = msg->angular_velocity.z;
    current_angular_velocity_z = msg->angular_velocity.z;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // 记录从第一次收到 odom 到当前的平面位移距离。
    // 当前测试逻辑没有把 odom_dist 作为地图状态机条件，但停车/地图扩展时可以复用。
    static bool has_origin = false;
    static float x0 = 0.0f;
    static float y0 = 0.0f;
    const float x_now = msg->pose.pose.position.x;
    const float y_now = msg->pose.pose.position.y;
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
    // /follow_begin 是测试节点的启动/切换入口：
    // Left/Right/Middle 启动并切换路径；Stop/Pause 停车并进入 IDLE。
    const std::string value = normalize(msg->data);
    if (value == "stop" || value == "pause") {
        run_car = false;
        motion_state = MotionState::IDLE;
        initial_turn_integrated_angle_deg = 0.0;
        initial_turn_has_last_time = false;
        publishStop();
        publishStatus("IDLE");
        ROS_WARN("follow_test stopped by command: %s", msg->data.c_str());
        return;
    }

    if (!setPathSelect(msg->data)) {
        ROS_WARN("Unknown follow_test command/path: %s", msg->data.c_str());
        return;
    }

    run_car = true;
    zeroCount = 0;
    zero_flag = false;
    startInitialTurnIfNeeded();
    ROS_WARN("follow_test started, path_select=%s state=%s",
             pathToString(path_select).c_str(),
             motion_state == MotionState::FOLLOWING ? "FOLLOWING" : "ALIGNING");
}

void detectCorners() {
    // 角点检测沿用原 follow_line.cpp 的思路：
    // 对左右边线的局部角度数组 rpts0a/rpts1a 做非极大值式判断，
    // 30~65 度认为可能是 Y 型岔口角点，70~110 度认为可能是 L 型直角点。
    // follow_test 主要保留 L 型停车逻辑，Y 点结果先作为调试信息输出。
    Ypt1_found = false;
    Lpt1_found = false;
    Ypt0_found = false;
    Lpt0_found = false;
    is_straight0 = rpts0s_num > 1.0 / sample_dist;
    is_straight1 = rpts1s_num > 1.0 / sample_dist;

    for (int i = 0; i < rpts0s_num; i++) {
        if (rpts0an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
        if (!Ypt0_found && 30.0 / 180.0 * PI < conf && conf < 65.0 / 180.0 * PI && i < 0.8 / sample_dist) {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        if (!Lpt0_found && 70.0 / 180.0 * PI < conf && conf < 110.0 / 180.0 * PI && i < 0.8 / sample_dist) {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        if (conf > 20.0 / 180.0 * PI && i < 1.0 / sample_dist) {
            is_straight0 = false;
        }
        if (Ypt0_found && Lpt0_found && !is_straight0) break;
    }

    for (int i = 0; i < rpts1s_num; i++) {
        if (rpts1an[i] == 0) continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        if (!Ypt1_found && 30.0 / 180.0 * PI < conf && conf < 65.0 / 180.0 * PI && i < 0.8 / sample_dist) {
            Ypt1_rpts1s_id = i;
            Ypt1_found = true;
        }
        if (!Lpt1_found && 70.0 / 180.0 * PI < conf && conf < 110.0 / 180.0 * PI && i < 0.8 / sample_dist) {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }
        if (conf > 20.0 / 180.0 * PI && i < 1.0 / sample_dist) {
            is_straight1 = false;
        }
        if (Ypt1_found && Lpt1_found && !is_straight1) break;
    }
}

bool handleParkingCorner() {
    // 停车逻辑保留原工程的“近距离 L 角点停车”能力。
    // 判断到靠近图像底部的 L 型角点后，先计算角点相对车体中心的距离，
    // 再用一个低速前进/横移的小循环把车挪到停车位置，最后发布 STOP。
    if (!parking_enabled) {
        return false;
    }

    float corner_dot[2] = {0.0f, 0.0f};
    bool is_stop_corner = false;

    if (Lpt1_found && Lpt1_rpts1s_id >= 3 && rptsc1_num > 0) {
        // 右侧线 L 角点：角点前后点在图像中形成“向左折”的趋势，
        // 且角点位于图像下方，说明停车点已经接近车体。
        int im1 = clip(Lpt1_rpts1s_id - (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        int ip1 = clip(Lpt1_rpts1s_id + (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        is_stop_corner = (rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1] > 20) &&
                         (rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0] < -20) &&
                         (rptsc1[Lpt1_rpts1s_id][1] > RESULT_ROW - 40);
        if (is_stop_corner) {
            corner_move(rpts1s, corner_dot, Lpt1_rpts1s_id, -pixel_per_meter * ROAD_WIDTH / 2);
        }
    } else if (Lpt0_found && Lpt0_rpts0s_id >= 3 && rptsc0_num > 0) {
        // 左侧线 L 角点：判断条件和右侧线对称，横向方向相反。
        int im0 = clip(Lpt0_rpts0s_id - (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        int ip0 = clip(Lpt0_rpts0s_id + (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        is_stop_corner = (rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1] > 20) &&
                         (rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0] > 20) &&
                         (rptsc0[Lpt0_rpts0s_id][1] > RESULT_ROW - 40);
        if (is_stop_corner) {
            corner_move(rpts0s, corner_dot, Lpt0_rpts0s_id, pixel_per_meter * ROAD_WIDTH / 2);
        }
    }

    if (!is_stop_corner) {
        return false;
    }

    const float cx = RESULT_COL / 2.0f;
    const float cy = RESULT_ROW + 10.0f;
    // 将图像坐标误差换算为近似米制距离。pixel_per_meter 来自原工程标定经验值。
    float target_dis = -(corner_dot[1] - cy) / pixel_per_meter;
    float target_dis_x = -(corner_dot[0] - cx) / pixel_per_meter;
    geometry_msgs::Twist local_msg;
    ros::Time last_time = ros::Time::now();

    ROS_WARN("Parking corner detected. target_dis=%.3f target_dis_x=%.3f", target_dis, target_dis_x);
    publishStatus("PARKING");

    while (ros::ok()) {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();
        const float dt = std::max(0.001, (now - last_time).toSec());
        last_time = now;

        local_msg.linear.x = 0.15;
        local_msg.linear.y = 0.0;
        // 横向误差较大时增加 y 方向微调，让停车点尽量落到车体中心附近。
        if (std::abs(target_dis_x) >= 0.08) {
            local_msg.linear.y = target_dis_x > 0 ? 0.1 : -0.1;
        }
        local_msg.angular.z = 0.0;

        target_dis -= local_msg.linear.x * dt;
        target_dis_x -= local_msg.linear.y * dt;

        if (target_dis < -0.215f) {
            // 到达停车距离后，先发零速度，再向 end_topic 发布 STOP，
            // 这样外部上层逻辑可以知道本段巡线已经结束。
            publishStop();
            std_msgs::String end_msg;
            end_msg.data = "STOP";
            for (int i = 0; i < 10; ++i) {
                end_pub.publish(end_msg);
                ros::Duration(0.05).sleep();
            }
            run_car = false;
            publishStatus("FINISHED");
            ROS_WARN("follow_test parking finished, STOP published.");
            return true;
        }

        pub.publish(local_msg);
    }

    return true;
}

void selectControlPath() {
    // 根据 path_select 选择最终控制用路径 rpts：
    // left  -> 优先使用左线偏移路径 rptsc0e；
    // right -> 优先使用右线偏移路径 rptsc1e；
    // middle-> 两边都有时取平均线，缺一边时退化为可用的一边。
    middle_path_num = 0;
    rpts = nullptr;
    rpts_num = 0;

    if (path_select == PathSelect::LEFT) {
        if (rptsc0e_num > 0) {
            rpts = rptsc0e;
            rpts_num = rptsc0e_num;
        } else if (rptsc1e_num > 0) {
            rpts = rptsc1e;
            rpts_num = rptsc1e_num;
        }
        return;
    }

    if (path_select == PathSelect::RIGHT) {
        if (rptsc1e_num > 0) {
            rpts = rptsc1e;
            rpts_num = rptsc1e_num;
        } else if (rptsc0e_num > 0) {
            rpts = rptsc0e;
            rpts_num = rptsc0e_num;
        }
        return;
    }

    if (rptsc0e_num > 0 && rptsc1e_num > 0) {
        // 中线模式不是重新扫线，而是在已经提取出的左右目标线之间做逐点平均。
        middle_path_num = std::min(rptsc0e_num, rptsc1e_num);
        for (int i = 0; i < middle_path_num; ++i) {
            middle_path[i][0] = (rptsc0e[i][0] + rptsc1e[i][0]) * 0.5f;
            middle_path[i][1] = (rptsc0e[i][1] + rptsc1e[i][1]) * 0.5f;
        }
        rpts = middle_path;
        rpts_num = middle_path_num;
    } else if (rptsc0e_num > 0) {
        rpts = rptsc0e;
        rpts_num = rptsc0e_num;
    } else if (rptsc1e_num > 0) {
        rpts = rptsc1e;
        rpts_num = rptsc1e_num;
    }
}

bool handleInitialTurn() {
    if (motion_state == MotionState::ALIGN_PAUSE) {
        geometry_msgs::Twist stop_msg;
        pub.publish(stop_msg);
        publishDebugImage();

        const double elapsed = (ros::Time::now() - initial_turn_pause_start).toSec();
        if (elapsed >= initial_turn_pause_sec) {
            motion_state = MotionState::FOLLOWING;
            publishStatus("RUNNING_" + pathToString(path_select));
            ROS_WARN("initial turn pause finished path=%s pause=%.2f",
                     pathToString(path_select).c_str(), elapsed);
            return true;
        }

        ROS_WARN_THROTTLE(0.5, "initial turn pause path=%s elapsed=%.2f target=%.2f",
                          pathToString(path_select).c_str(), elapsed, initial_turn_pause_sec);
        return true;
    }

    if (motion_state != MotionState::ALIGNING_LEFT && motion_state != MotionState::ALIGNING_RIGHT) {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if (!initial_turn_has_last_time) {
        initial_turn_last_time = now;
        initial_turn_has_last_time = true;
    }

    // 原 follow_line.cpp 的 Round_step 转角算法就是这一句：
    // rotated_angle += dt * curent_wz * 57.3。这里保留同样思想，
    // curent_wz 单位是 rad/s，乘 57.3 约等于转换成 deg/s。
    double dt = (now - initial_turn_last_time).toSec();//计算时间
    initial_turn_last_time = now;
    if (dt < 0.0 || dt > 0.2) {
        dt = 0.0;
    }
    initial_turn_integrated_angle_deg += curent_wz * dt * 57.3;//积分计算已经走过的角度

    const double turned_abs = std::abs(initial_turn_integrated_angle_deg);
    const int selected_count = selectedPathPointCount();
    const bool angle_ok = turned_abs >= initial_turn_angle_deg;
    const bool line_ok = selected_count >= initial_turn_rpts_threshold;

    if (angle_ok || line_ok) {
        publishStop();
        motion_state = MotionState::ALIGN_PAUSE;
        initial_turn_has_last_time = false;
        initial_turn_pause_start = ros::Time::now();
        publishStatus("ALIGN_PAUSE_" + pathToString(path_select));
        ROS_WARN("initial turn finished path=%s integrated_angle=%.2f wz=%.3f selected_rpts=%d angle_ok=%d line_ok=%d",
                 pathToString(path_select).c_str(), initial_turn_integrated_angle_deg, curent_wz,
                 selected_count, angle_ok, line_ok);
        return true;
    }//添加的角度和线判断指令

    geometry_msgs::Twist msg;
    const double turned_abs = std::abs(initial_turn_integrated_angle_deg);
    msg.linear.x=0.0
// 剩余角度，越接近目标越小
    const double remaining_angle = std::max(0.0, initial_turn_angle_deg - turned_abs);

    // PID 输出角速度大小
    double pid_speed = pid.compute(initial_turn_angle_deg, turned_abs);

    // 防止方向被 PID 符号影响，这里只取大小
    pid_speed = std::abs(pid_speed);

    // 限制最大角速度，避免太猛
    pid_speed = std::min(pid_speed, std::abs(initial_turn_angular_speed));

    // 给一个最小角速度，避免快到目标时转不动
    pid_speed = std::max(pid_speed, min_pid_speed);

    // 根据状态决定左转还是右转
    msg.angular.z = motion_state == MotionState::ALIGNING_LEFT ? pid_speed : -pid_speed;

    pub.publish(msg);


    publishDebugImage();

    ROS_WARN_THROTTLE(0.5, "initial turn path=%s integrated_angle=%.2f wz=%.3f target=%.2f selected_rpts=%d threshold=%d",
                      pathToString(path_select).c_str(), initial_turn_integrated_angle_deg, curent_wz,
                      initial_turn_angle_deg, selected_count, initial_turn_rpts_threshold);
    return true;
}

void publishDebugImage(const sensor_msgs::ImageConstPtr &source_msg) {
    // 调试图像是 MONO8：底图来自 ImageUsed，随后叠加左右偏移线和当前控制路径。
    // 颜色值约定：0 标左线，80 标右线，160 标当前选择的控制路径。
    for (int i = 0; i < RESULT_ROW; ++i) {
        for (int j = 0; j < RESULT_COL; ++j) {
            img_line_data[i][j] = ImageUsed[i][j];
        }
    }
    for (int i = 0; i < rptsc0e_num; ++i) {
        AT_IMAGE(&img_line, clip(rptsc0e[i][0], 0, img_line.width - 1),
                 clip(rptsc0e[i][1], 0, img_line.height - 1)) = 0;
    }
    for (int i = 0; i < rptsc1e_num; ++i) {
        AT_IMAGE(&img_line, clip(rptsc1e[i][0], 0, img_line.width - 1),
                 clip(rptsc1e[i][1], 0, img_line.height - 1)) = 80;
    }
    for (int i = 0; i < rpts_num; ++i) {
        AT_IMAGE(&img_line, clip(rpts[i][0], 0, img_line.width - 1),
                 clip(rpts[i][1], 0, img_line.height - 1)) = 160;
    }

    cv::Mat debug_gray = convert2DArrayToMat(img_line_data);
    if (show_window) {
        cv::imshow("follow_test", debug_gray);
        cv::waitKey(1);
    }
    if (publish_debug_image && debug_pub) {
        std_msgs::Header header;
        if (source_msg) {
            header = source_msg->header;
        }
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            header, sensor_msgs::image_encodings::MONO8, debug_gray).toImageMsg();
        debug_pub.publish(msg);
    }
}

int followLineTestOnce() {
    // 主循环每次调用一次。没有收到 /follow_begin 时 run_car=false，直接返回。
    if (!run_car) {
        return 0;
    }
    cv::Mat local_frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        if (frame.empty()) {
            return -1;
        }
        local_frame = frame.clone();
    }

    cv::Mat image_gray;
    // 图像预处理流程保持原算法顺序：
    // BGR -> 灰度 -> 二维数组 -> 黑白反转 -> process_image() 完成扫线和路径生成。
    cv::cvtColor(local_frame, image_gray, cv::COLOR_BGR2GRAY);
    convertMatTo2DArray(image_gray, PER_IMG);
    invertImage(PER_IMG);
    img_raw.data = PER_IMG[0];
    process_image();

    // 综合巡线顺序：
    // 1. detectCorners() 给停车/岔路调试提供 L 点、Y 点结果；
    // 2. selectControlPath() 根据 left/middle/right 选择最终控制路径；
    // 3. handleInitialTurn() 只在刚收到 Left/Right 启动指令后接管底盘；
    // 4. 预转完成后才进入停车判断和常规视觉巡线速度输出。
    detectCorners();
    selectControlPath();
    if (handleInitialTurn()) {
        return 0;
    }

    if (handleParkingCorner()) {
        // 检测到停车角点并完成停车后，本轮不再继续发布巡线速度。
        return 0;
    }

    float error = 0.0f;
    float v = 0.0f;
    if (rpts_num == 0) {
        // 连续丢线时逐步停车；偶发一帧丢线时仍低速前进，减少图像抖动影响。
        zeroCount++;
        if (zeroCount >= 2) {
            zero_flag = true;
        }
        error = 0.0f;
        v = zero_flag ? 0.0f : 0.15f;
    } else {
        zeroCount = 0;
        zero_flag = false;
        // 取前方 aim_distance 处的路径点作为瞄准点。
        // dx/dy 转成 atan2 角度误差，再用误差大小降低线速度。
        const int aim_idx = clip(round(aim_distance / sample_dist), 0, rpts_num - 1);
        const float cx = RESULT_COL / 2.0f;
        const float cy = RESULT_ROW + 10.0f;
        const float dx = rpts[aim_idx][0] - cx;
        const float dy = cy - rpts[aim_idx][1] + aim_y_bias_m * pixel_per_meter;
        error = -atan2f(dx, dy);
        v = static_cast<float>(base_speed - std::abs(error) * base_speed);
        v = std::max(0.05f, v);
    }

    geometry_msgs::Twist msg;
    msg.linear.x = v;
    msg.angular.z = -error;
    pub.publish(msg);
    publishDebugImage();

    ROS_WARN_THROTTLE(1.0, "follow_test path=%s rpts=%d error=%.3f v=%.3f L0=%d L1=%d Y0=%d Y1=%d",
                      pathToString(path_select).c_str(), rpts_num, error, v,
                      Lpt0_found, Lpt1_found, Ypt0_found, Ypt1_found);
    return 0;
}

std::string currentPathName() {
    return pathToString(path_select);
}

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double turn_angle_deg,
               double turn_angular_speed, int turn_rpts_threshold,
               double turn_pause_sec) {
    // 保存 launch 参数，供后续图像调试、停车开关和速度控制使用。
    publish_debug_image = publish_debug;
    show_window = show_debug_window;
    parking_enabled = enable_parking;
    base_speed = speed;
    aim_distance = distance;
    aim_y_bias_m = y_bias_m;
    initial_turn_enabled = enable_initial_turn;
    initial_turn_angle_deg = std::max(0.0, turn_angle_deg);
    initial_turn_angular_speed = std::max(0.0, turn_angular_speed);
    initial_turn_rpts_threshold = std::max(1, turn_rpts_threshold);
    initial_turn_pause_sec = std::max(0.0, turn_pause_sec);
}

void initializeImagePipeline() {
    // 建立逆透视映射表 point_map/PerImg_ip，process_image() 依赖这些查找表。
    ImagePerspective_Init();
    Global_move_timer = ros::Time::now();
}

void advertiseTopics(ros::NodeHandle &nh, const std::string &cmd_vel_topic,
                     const std::string &end_topic) {
    // pub/end_pub 使用原工程全局变量名，保证复用原有头文件和辅助函数时不需要大改。
    pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
    end_pub = nh.advertise<std_msgs::String>(end_topic, 10);
    status_pub = nh.advertise<std_msgs::String>("/flow_end/follow_test_status", 10);
    debug_pub = nh.advertise<sensor_msgs::Image>("/flow_end/follow_test_debug", 1);
}

void subscribeTopics(ros::NodeHandle &nh, const std::string &image_topic,
                     const std::string &imu_topic, const std::string &odom_topic,
                     const std::string &begin_topic) {
    // subscribers 必须保持生命周期，否则函数返回后订阅器析构，回调不会再触发。
    static std::vector<ros::Subscriber> subscribers;
    subscribers.clear();
    subscribers.push_back(nh.subscribe(image_topic, 1, imageCallback));
    subscribers.push_back(nh.subscribe(imu_topic, 10, imuCallback));
    subscribers.push_back(nh.subscribe(odom_topic, 10, odomCallback));
    subscribers.push_back(nh.subscribe(begin_topic, 10, beginCallback));
}

bool shouldExit() {
    return sig_INT.load();
}

void shutdown() {
    // 节点退出时主动停车，避免调试时 Ctrl-C 后底盘保留上一条速度。
    publishStop();
  
}

}  // namespace follow_test
}  // namespace flow_end
