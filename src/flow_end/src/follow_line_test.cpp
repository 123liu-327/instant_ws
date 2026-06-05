#include <flow_end/follow.h>
#include <flow_end/follow_line_test.h>
#include <flow_end/follow_motion_controller.h>
#include <flow_end/ImagePerspectiveInit.h>
#include <flow_end/MatTransform.h>
#include <flow_end/process_image.h>
#include <flow_end/corner_move.h>
#include <flow_end/PID.h>

#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <string>

// follow_line_test.cpp 是 follow_test 节点的具体工作逻辑文件。
// 这里保留原 flow_end 视觉巡线算法依赖的全局变量，同时去掉原地图中的环岛和激光雷达流程。
// follow_test.cpp 负责 ROS 入口，Callback_test.cpp 负责订阅回调。
// 本文件只保留巡线业务逻辑：
// 1. 调用 process_image() 提取左右边线；
// 2. 根据 left/middle/right 选择控制路径；
// 3. 保留角点检测、起步预转和停车逻辑；
// 4. 发布 /cmd_vel、状态和调试图像。

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
bool is_degraded_mode = false;  // 退化模式标注：当无法使用首选路径时的退化状态

namespace flow_end {
namespace follow_test {

cv::VideoWriter debug_video_writer;
bool video_recording = false;
bool enable_video_record = true;
std::string video_save_path = "/tmp/follow_test_debug.avi";
bool auto_video_save_path = true;
int video_fps = 10;

PathSelect path_select = PathSelect::RIGHT;
MotionState motion_state = MotionState::IDLE;
ros::Publisher debug_pub;
ros::Publisher status_pub;
float middle_path[POINTS_MAX_LEN][2];
int middle_path_num = 0;

// These runtime params are written through configure(); Callback_test.cpp refreshes
// them from the private parameter server before each start command.
bool publish_debug_image = true;
bool show_window = false;
bool parking_enabled = true;
bool parking_allow_either_l = true;
double parking_extra_dist = 0.215;
double parking_forward_speed = 0.15;
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
double min_pid_speed = 0.08;
ros::Time initial_turn_pause_start;
bool y_branch_mode_requested = false;
PathSelect pending_branch_path = PathSelect::RIGHT;
double y_approach_dist = 0.20;
double y_turn_angle_deg = 45.0;
double y_turn_angular_speed = 0.35;
double y_turn_pause_sec = 0.5;
int y_detect_min_id = 35;
double y_center_aim_dist = 0.45;
double y_approach_speed = 0.18;
double y_center_max_wz = 0.25;
int y_lost_confirm_frames = 3;
double y_entry_min_odom = 0.08;
double y_entry_max_odom = 0.45;
int y_entry_lost_count = 0;
int y_detect_max_id = 80;
int y_detect_confirm_frames = 2;
int y_detect_confirm_count = 0;
double y_crossbar_seek_speed = 0.08;
int y_crossbar_lost_confirm_frames = 3;
double y_crossbar_target_long_m = 0.25;
double y_crossbar_long_tolerance_m = 0.10;
double y_crossbar_max_abs_lat_m = 0.18;
int y_crossbar_confirm_frames = 2;
double y_crossbar_seek_max_odom = 0.60;
int y_crossbar_lost_count = 0;
int y_crossbar_confirm_count = 0;
float y_crossbar_seek_start_odom = 0.0f;
ros::Time y_crossbar_seek_start_time;
double y_turn_integrated_angle_deg = 0.0;
ros::Time y_turn_last_time;
bool y_turn_has_last_time = false;
ros::Time y_approach_start_time;
float y_approach_start_odom = 0.0f;
ros::Time y_turn_pause_start;
FollowMotionController motion_controller;

void configureMotionController(const MotionControlConfig &config) {
    motion_controller.configure(config);
}

void resetMotionController() {
    motion_controller.reset();
}

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

void resetParkingCornerState() {
    // Kept for state-transition call sites. The restored parking behavior no
    // longer tracks a first/second square corner pair.
}

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
    resetMotionController();
    if (y_branch_mode_requested) {
        motion_state = MotionState::FOLLOWING_STRAIGHT;
        y_turn_integrated_angle_deg = 0.0;
        y_turn_has_last_time = false;
        y_detect_confirm_count = 0;
        y_entry_lost_count = 0;
        y_crossbar_lost_count = 0;
        y_crossbar_confirm_count = 0;
        forward_crossbar_result.found = false;
        resetParkingCornerState();
        pid.reset();
        publishStatus("Y_SEARCH_" + pathToString(pending_branch_path));
        return;
    }

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
    if (value == "yleft" || value == "y_left" || value == "yl") {
        resetMotionController();
        path_select = PathSelect::MIDDLE;
        pending_branch_path = PathSelect::LEFT;
        y_branch_mode_requested = true;
        track_type = TRACK_MIDDLE;
        return true;
    }
    if (value == "yright" || value == "y_right" || value == "yr") {
        resetMotionController();
        path_select = PathSelect::MIDDLE;
        pending_branch_path = PathSelect::RIGHT;
        y_branch_mode_requested = true;
        track_type = TRACK_MIDDLE;
        return true;
    }
    if (value == "left" || value == "l") {
        resetMotionController();
        path_select = PathSelect::LEFT;
        pending_branch_path = PathSelect::LEFT;
        y_branch_mode_requested = false;
        track_type = TRACK_LEFT;
        return true;
    }
    if (value == "middle" || value == "mid" || value == "center" || value == "centre" || value == "m") {
        resetMotionController();
        path_select = PathSelect::MIDDLE;
        pending_branch_path = PathSelect::MIDDLE;
        y_branch_mode_requested = false;
        track_type = TRACK_MIDDLE;
        return true;
    }
    if (value == "right" || value == "r") {
        resetMotionController();
        path_select = PathSelect::RIGHT;
        pending_branch_path = PathSelect::RIGHT;
        y_branch_mode_requested = false;
        track_type = TRACK_RIGHT;
        return true;
    }
    return false;
}

void publishStop() {
    // 连续发布零速度，比只发一次更可靠，避免底盘控制器错过停车指令。
    resetMotionController();
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
        if (!Ypt0_found && 25.0 / 180.0 * PI < conf && conf < 65.0 / 180.0 * PI && i < 0.8 / sample_dist) {
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
        if (!Ypt1_found && 25.0 / 180.0 * PI < conf && conf < 65.0 / 180.0 * PI && i < 0.8 / sample_dist) {
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

bool handleYBranchFlow() {
    if (motion_state == MotionState::FOLLOWING_STRAIGHT) {
        const bool y_seen = Ypt0_found || Ypt1_found;
        const bool lost_all_lines = (rpts_num == 0 && rptsc0e_num == 0 && rptsc1e_num == 0);

        if (y_seen) {
            y_detect_confirm_count++;
            y_crossbar_lost_count = 0;
        } else {
            y_detect_confirm_count = 0;
            if (lost_all_lines) {
                y_crossbar_lost_count++;
            } else {
                y_crossbar_lost_count = 0;
            }
        }

        ROS_WARN_THROTTLE(0.5,
                          "[Y_BRANCH] Searching | next_path=%s | Y0=%d(id=%d) | Y1=%d(id=%d) | y_seen=%d | y_confirm=%d/%d | lost=%d/%d | id_filter=off",
                          pathToString(pending_branch_path).c_str(),
                          Ypt0_found, Ypt0_rpts0s_id,
                          Ypt1_found, Ypt1_rpts1s_id,
                          y_seen,
                          y_detect_confirm_count,
                          y_detect_confirm_frames,
                          y_crossbar_lost_count,
                          y_crossbar_lost_confirm_frames);

        if (y_detect_confirm_count >= y_detect_confirm_frames) {
            motion_state = MotionState::Y_CENTER_APPROACH;
            y_approach_start_odom = odom_dist;
            y_approach_start_time = ros::Time::now();
            y_entry_lost_count = 0;
            y_crossbar_lost_count = 0;
            y_crossbar_confirm_count = 0;
            publishStatus("Y_CENTER_APPROACH_" + pathToString(pending_branch_path));
            ROS_WARN("[Y_BRANCH] Center approach started | next_path=%s | Y0=%d(id=%d) | Y1=%d(id=%d) | confirm=%d/%d | odom=%.3fm | id_filter=off",
                     pathToString(pending_branch_path).c_str(),
                     Ypt0_found, Ypt0_rpts0s_id,
                     Ypt1_found, Ypt1_rpts1s_id,
                     y_detect_confirm_count,
                     y_detect_confirm_frames,
                     odom_dist);
            return true;
        }

        if (y_crossbar_lost_count >= y_crossbar_lost_confirm_frames) {
            resetMotionController();
            motion_state = MotionState::Y_CROSSBAR_SEEK;
            y_crossbar_seek_start_odom = odom_dist;
            y_crossbar_seek_start_time = ros::Time::now();
            y_crossbar_confirm_count = 0;
            pid.reset();
            publishStatus("Y_CROSSBAR_SEEK_" + pathToString(pending_branch_path));
            ROS_WARN("[Y_BRANCH] Lost Y and lines, seeking crossbar | next_path=%s | lost=%d/%d | odom=%.3fm",
                     pathToString(pending_branch_path).c_str(),
                     y_crossbar_lost_count,
                     y_crossbar_lost_confirm_frames,
                     odom_dist);
            return true;
        }
        return false;
    }

    if (motion_state == MotionState::Y_CROSSBAR_SEEK) {
        const float moved = std::abs(odom_dist - y_crossbar_seek_start_odom);
        const bool found = detect_forward_crossbar();
        const bool long_ok = found &&
                             std::abs(forward_crossbar_result.long_m -
                                      static_cast<float>(y_crossbar_target_long_m)) <=
                                 static_cast<float>(y_crossbar_long_tolerance_m);
        const bool lat_ok = found &&
                            std::abs(forward_crossbar_result.lat_m) <=
                                static_cast<float>(y_crossbar_max_abs_lat_m);

        if (found && long_ok && lat_ok) {
            y_crossbar_confirm_count++;
        } else {
            y_crossbar_confirm_count = 0;
        }

        const bool reached_by_crossbar = y_crossbar_confirm_count >= y_crossbar_confirm_frames;
        const bool reached_by_max_odom = moved >= static_cast<float>(y_crossbar_seek_max_odom);
        if (reached_by_crossbar || reached_by_max_odom) {
            resetMotionController();
            motion_state = pending_branch_path == PathSelect::LEFT ?
                           MotionState::Y_ALIGNING_LEFT : MotionState::Y_ALIGNING_RIGHT;
            y_turn_integrated_angle_deg = 0.0;
            y_turn_last_time = ros::Time::now();
            y_turn_has_last_time = true;
            pid.reset();
            publishStatus("Y_TURN_" + pathToString(pending_branch_path));
            ROS_WARN("[Y_CROSSBAR] Trigger turn | reason=%s | next_path=%s | found=%d | center=(%d,%d) | long=%.3fm | lat=%.3fm | confirm=%d/%d | moved=%.3fm",
                     reached_by_crossbar ? "crossbar" : "max_odom",
                     pathToString(pending_branch_path).c_str(),
                     found,
                     forward_crossbar_result.center_x,
                     forward_crossbar_result.center_y,
                     forward_crossbar_result.long_m,
                     forward_crossbar_result.lat_m,
                     y_crossbar_confirm_count,
                     y_crossbar_confirm_frames,
                     moved);
            forward_crossbar_result.found = false;
            return true;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = y_crossbar_seek_speed;
        msg.angular.z = 0.0;
        pub.publish(msg);
        publishDebugImage();

        ROS_WARN_THROTTLE(0.5,
                          "[Y_CROSSBAR] Seeking | found=%d | center=(%d,%d) | long=%.3fm | lat=%.3fm | confirm=%d/%d | moved=%.3fm | v=%.2f",
                          found,
                          forward_crossbar_result.center_x,
                          forward_crossbar_result.center_y,
                          forward_crossbar_result.long_m,
                          forward_crossbar_result.lat_m,
                          y_crossbar_confirm_count,
                          y_crossbar_confirm_frames,
                          moved,
                          msg.linear.x);
        return true;
    }

    if (motion_state == MotionState::Y_CENTER_APPROACH) {
        const float moved = std::abs(odom_dist - y_approach_start_odom);
        const bool lost_entry = (rpts_num == 0 && rptsc0e_num == 0 && rptsc1e_num == 0);
        if (lost_entry) {
            y_entry_lost_count++;
        } else {
            y_entry_lost_count = 0;
        }

        const bool reached_by_lost_lines =
            moved >= static_cast<float>(y_entry_min_odom) &&
            y_entry_lost_count >= y_lost_confirm_frames;
        const bool reached_by_max_odom = moved >= static_cast<float>(y_entry_max_odom);

        if (reached_by_lost_lines || reached_by_max_odom) {
            resetMotionController();
            motion_state = pending_branch_path == PathSelect::LEFT ?
                           MotionState::Y_ALIGNING_LEFT : MotionState::Y_ALIGNING_RIGHT;
            y_turn_integrated_angle_deg = 0.0;
            y_turn_last_time = ros::Time::now();
            y_turn_has_last_time = true;
            pid.reset();
            publishStatus("Y_TURN_" + pathToString(pending_branch_path));
            ROS_WARN("[Y_BRANCH] Entry reached by %s | next_path=%s | moved=%.3fm | lost=%d/%d",
                     reached_by_lost_lines ? "lost lines" : "max odom",
                     pathToString(pending_branch_path).c_str(),
                     moved,
                     y_entry_lost_count,
                     y_lost_confirm_frames);
            return true;
        }

        MotionControlInput control_input;
        control_input.path = rpts;
        control_input.path_num = rpts_num;
        control_input.path_key = static_cast<int>(path_select);
        control_input.path_name = pathToString(path_select);
        control_input.degraded = is_degraded_mode;
        control_input.base_speed = y_approach_speed;
        control_input.aim_distance = y_center_aim_dist;
        control_input.aim_y_bias_m = 0.0;
        control_input.sample_dist = sample_dist;
        control_input.pixel_per_meter = pixel_per_meter;
        control_input.image_width = RESULT_COL;
        control_input.image_height = RESULT_ROW;
        control_input.allow_lost_coast = false;
        control_input.max_wz_override = y_center_max_wz;
        const MotionControlOutput control_output = motion_controller.compute(control_input);
        pub.publish(control_output.cmd);

        const double elapsed = (ros::Time::now() - y_approach_start_time).toSec();
        ROS_WARN_THROTTLE(0.5,
                          "[Y_BRANCH] Center approaching | next_path=%s | rpts=%d | error=%.3f | moved=%.3fm | lost=%d/%d | v=%.2f | wz=%.2f | elapsed=%.2fs",
                          pathToString(pending_branch_path).c_str(),
                          rpts_num,
                          control_output.filtered_error,
                          moved,
                          y_entry_lost_count,
                          y_lost_confirm_frames,
                          control_output.cmd.linear.x,
                          control_output.cmd.angular.z,
                          elapsed);
        return true;
    }

    if (motion_state == MotionState::Y_ALIGNING_LEFT ||
        motion_state == MotionState::Y_ALIGNING_RIGHT) {
        const ros::Time now = ros::Time::now();
        double dt = 0.0;
        if (y_turn_has_last_time) {
            dt = (now - y_turn_last_time).toSec();
        }
        y_turn_last_time = now;
        y_turn_has_last_time = true;

        if (dt > 0.0 && dt < 0.2) {
            y_turn_integrated_angle_deg += std::abs(curent_wz) * dt * 180.0 / M_PI;
        }

        if (y_turn_integrated_angle_deg >= y_turn_angle_deg) {
            publishStop();
            motion_state = MotionState::Y_ALIGN_PAUSE;
            y_turn_pause_start = ros::Time::now();
            publishStatus("Y_TURN_PAUSE_" + pathToString(pending_branch_path));
            ROS_WARN("[Y_TURN] Finished | next_path=%s | integrated_angle=%.2fdeg/%.2fdeg | wz=%.3f",
                     pathToString(pending_branch_path).c_str(),
                     y_turn_integrated_angle_deg, y_turn_angle_deg, curent_wz);
            return true;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = motion_state == MotionState::Y_ALIGNING_LEFT ?
                        std::abs(y_turn_angular_speed) : -std::abs(y_turn_angular_speed);
        pub.publish(msg);

        ROS_WARN_THROTTLE(0.5,
                          "[Y_TURN] Turning | next_path=%s | integrated_angle=%.2fdeg/%.2fdeg | wz=%.3f rad/s | dt=%.3fs | turn_direction=%s",
                          pathToString(pending_branch_path).c_str(),
                          y_turn_integrated_angle_deg, y_turn_angle_deg,
                          curent_wz, dt,
                          motion_state == MotionState::Y_ALIGNING_LEFT ? "LEFT" : "RIGHT");
        return true;
    }

    if (motion_state == MotionState::Y_ALIGN_PAUSE) {
        geometry_msgs::Twist stop_msg;
        pub.publish(stop_msg);

        const double elapsed = (ros::Time::now() - y_turn_pause_start).toSec();
        if (elapsed >= y_turn_pause_sec) {
            path_select = pending_branch_path;
            track_type = path_select == PathSelect::LEFT ? TRACK_LEFT : TRACK_RIGHT;
            y_branch_mode_requested = false;
            y_turn_has_last_time = false;
            motion_state = MotionState::FOLLOWING;
            resetMotionController();
            resetParkingCornerState();
            publishStatus("RUNNING_" + pathToString(path_select));
            ROS_WARN("[Y_BRANCH] Switched to branch follow | path=%s | pause=%.2fs",
                     pathToString(path_select).c_str(), elapsed);
        }
        return true;
    }

    return false;
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
    const char *parking_line_type = "None";
    int parking_corner_id = -1;
    const int parking_min_corner_id = 1;
    const float parking_shape_thresh_px = 15.0f;
    const float parking_bottom_margin_px = 80.0f;

    auto tryRightParkingCorner = [&]() -> bool {
        if (!Lpt1_found || Lpt1_rpts1s_id < parking_min_corner_id || Lpt1_rpts1s_id >= rptsc1_num) {
            return false;
        }
        // 右侧线 L 角点：角点前后点在图像中形成“向左折”的趋势，
        // 且角点位于图像下方，说明停车点已经接近车体。
        int im1 = clip(Lpt1_rpts1s_id - (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        int ip1 = clip(Lpt1_rpts1s_id + (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        const bool valid_corner = (rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1] > parking_shape_thresh_px) &&
                                  (rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0] < -parking_shape_thresh_px) &&
                                  (rptsc1[Lpt1_rpts1s_id][1] > RESULT_ROW - parking_bottom_margin_px);
        if (!valid_corner) {
            return false;
        }
        corner_move(rpts1s, corner_dot, Lpt1_rpts1s_id, -pixel_per_meter * ROAD_WIDTH / 2);
        parking_line_type = "Right_L";
        parking_corner_id = Lpt1_rpts1s_id;
        return true;
    };

    auto tryLeftParkingCorner = [&]() -> bool {
        if (!Lpt0_found || Lpt0_rpts0s_id < parking_min_corner_id || Lpt0_rpts0s_id >= rptsc0_num) {
            return false;
        }
        // 左侧线 L 角点：判断条件和右侧线对称，横向方向相反。
        int im0 = clip(Lpt0_rpts0s_id - (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        int ip0 = clip(Lpt0_rpts0s_id + (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        const bool valid_corner = (rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1] > parking_shape_thresh_px) &&
                                  (rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0] > parking_shape_thresh_px) &&
                                  (rptsc0[Lpt0_rpts0s_id][1] > RESULT_ROW - parking_bottom_margin_px);
        if (!valid_corner) {
            return false;
        }
        corner_move(rpts0s, corner_dot, Lpt0_rpts0s_id, pixel_per_meter * ROAD_WIDTH / 2);
        parking_line_type = "Left_L";
        parking_corner_id = Lpt0_rpts0s_id;
        return true;
    };

    if (parking_allow_either_l) {
        is_stop_corner = tryRightParkingCorner() || tryLeftParkingCorner();
    } else if (path_select == PathSelect::RIGHT) {
        is_stop_corner = tryRightParkingCorner();
    } else if (path_select == PathSelect::LEFT) {
        is_stop_corner = tryLeftParkingCorner();
    }

    if (!is_stop_corner) {
        // 周期性打印角点检测状态（即使未检测到停车点）
        ROS_WARN_THROTTLE(2.0, "[PARKING] CornerDetect | path=%s | L0=%d(id=%d) | L1=%d(id=%d) | Y0=%d(id=%d) | Y1=%d(id=%d) | left_pts=%d | right_pts=%d | parking_enable=%d",
                  pathToString(path_select).c_str(),
                  Lpt0_found, Lpt0_rpts0s_id, Lpt1_found, Lpt1_rpts1s_id,
                  Ypt0_found, Ypt0_rpts0s_id, Ypt1_found, Ypt1_rpts1s_id,
                  rptsc0_num, rptsc1_num, parking_enabled);
        return false;
    }

    const float cx = RESULT_COL / 2.0f;
    const float cy = RESULT_ROW + 10.0f;
    // 将图像坐标误差换算为近似米制距离。pixel_per_meter 来自原工程标定经验值。
    float target_dis = -(corner_dot[1] - cy) / pixel_per_meter;
    float target_dis_x = -(corner_dot[0] - cx) / pixel_per_meter;
    geometry_msgs::Twist local_msg;
    ros::Time last_time = ros::Time::now();

    ROS_WARN("[PARKING] Corner detected | corner_dot=(%.1f,%.1f) | image_center=(%.1f,%.1f) | "
             "long_dist=%.3fm | lat_bias=%.3fm | line_type=%s | corner_id=%d",
             corner_dot[0], corner_dot[1], cx, cy,
             target_dis, target_dis_x,
             parking_line_type,
             parking_corner_id);
    publishStatus("PARKING");
    resetMotionController();

    int parking_loop_count = 0;  // 停车循环计数器
    ros::Time parking_start_time = ros::Time::now();  // 停车开始时间
    float last_print_dis = target_dis;  // 上次打印时的距离
    const float initial_target_dis = target_dis;
    const float parking_start_odom = odom_dist;
    const float parking_total_dist = std::max(0.001f, std::abs(target_dis) + static_cast<float>(parking_extra_dist));
    float parking_moved_from_velocity = 0.0f;
    float previous_target_dis = target_dis;  // 上一次的目标距离
    ros::Rate parking_rate(30.0);

    while (ros::ok()) {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();
        float dt = (now - last_time).toSec();

        // Only integrate with real elapsed time; the rate sleep below prevents
        // the parking loop from virtually consuming distance in a few ms.
        if (dt < 0.0f || dt > 0.2f) {
            ROS_WARN_THROTTLE(1.0, "[PARKING_TIME] Invalid dt=%.4fs, skip distance integration this cycle", dt);
            dt = 0.0f;
        }
        last_time = now;
        parking_loop_count++;

        local_msg.linear.x = parking_forward_speed;
        local_msg.linear.y = 0.0;
        // 横向误差较大时增加 y 方向微调，让停车点尽量落到车体中心附近。
        if (std::abs(target_dis_x) >= 0.08) {
            local_msg.linear.y = target_dis_x > 0 ? 0.1 : -0.1;
        }
        local_msg.angular.z = 0.0;

        parking_moved_from_velocity += static_cast<float>(std::abs(current_linear_velocity_x)) * dt;
        const float parking_moved = std::max(std::abs(odom_dist - parking_start_odom),
                                             parking_moved_from_velocity);

        // 保存旧值用于检测异常
        previous_target_dis = target_dis;
        target_dis = initial_target_dis - parking_moved;
        target_dis_x -= local_msg.linear.y * dt;

        // 检测距离异常（不应该增加）
        if (target_dis > previous_target_dis && parking_loop_count > 10) {
            ROS_ERROR("[PARKING_ERROR] Distance increased! prev=%.3fm -> curr=%.3fm | dt=%.4fs",
                     previous_target_dis, target_dis, dt);
        }

        float elapsed_sec = (now - parking_start_time).toSec();
        if (elapsed_sec > 1.0f && parking_moved < 0.02f) {
            ROS_WARN_THROTTLE(1.0, "[PARKING_STUCK] cmd_vel is being published but odom_moved=%.3fm/%.3fm | odom_vx=%.3fm/s",
                              parking_moved, parking_total_dist, current_linear_velocity_x);
        }

        // 靠近停车点时的调试信息（距离 < 0.5m 时开始打印）
        if (std::abs(target_dis) < 0.5f) {
            // Use the full planned parking distance: visible corner distance
            // plus the configured extra distance after crossing the L point.
            float remaining_dis = std::max(0.0f, parking_total_dist - parking_moved);
            float progress_percent = ((parking_total_dist - remaining_dis) / parking_total_dist) * 100.0f;

            // Print when distance changes to avoid spamming
            if (std::abs(target_dis - last_print_dis) > 0.02f) {
                last_print_dis = target_dis;
                ROS_WARN("[PARKING_PROGRESS] Approaching... | progress=%.0f%% | long_dist=%.3fm/%.3fm | "
                         "lat_bias=%.3fm | vel=(%.2f,%.2f) | dt=%.4fs | loops=%d | elapsed=%.2fs",
                         std::max(0.0f, std::min(progress_percent, 100.0f)),
                         remaining_dis, parking_total_dist,
                         target_dis_x,
                         local_msg.linear.x, local_msg.linear.y,
                         dt, parking_loop_count, elapsed_sec);
            }
        }

        if (parking_moved >= parking_total_dist) {
            // 到达停车距离后，先发零速度，再向 end_topic 发布 STOP，
            // 这样外部上层逻辑可以知道本段巡线已经结束。

            float total_time = (now - parking_start_time).toSec();
            ROS_WARN("[PARKING] Parking finished! | final_long_dist=%.3fm | final_lat_bias=%.3fm | "
                     "odom_moved=%.3fm/%.3fm | total_loops=%d | total_time=%.2fs | line_type=%s",
                     std::max(0.0f, parking_total_dist - parking_moved),
                     target_dis_x,
                     parking_moved,
                     parking_total_dist,
                     parking_loop_count,
                     total_time,
                     parking_line_type);

            publishStop();
            std_msgs::String end_msg;
            end_msg.data = "STOP";
            for (int i = 0; i < 10; ++i) {
                end_pub.publish(end_msg);
                ros::Duration(0.05).sleep();
            }
            run_car = false;
            resetParkingCornerState();
            publishStatus("FINISHED");
            return true;
        }

        pub.publish(local_msg);
        parking_rate.sleep();
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
    is_degraded_mode = false;  // 每次选择前重置退化状态

    if (path_select == PathSelect::LEFT) {
        if (rptsc0e_num > 0) {
            rpts = rptsc0e;
            rpts_num = rptsc0e_num;
        } else if (rptsc1e_num > 0) {
            rpts = rptsc1e;
            rpts_num = rptsc1e_num;
            is_degraded_mode = true;  // LEFT模式但没有左线，退化使用右线
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
            is_degraded_mode = true;  // RIGHT模式但没有右线，退化使用左线
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
        is_degraded_mode = true;  // MIDDLE模式只有左线，退化为单侧
    } else if (rptsc1e_num > 0) {
        rpts = rptsc1e;
        rpts_num = rptsc1e_num;
        is_degraded_mode = true;  // MIDDLE模式只有右线，退化为单侧
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
            return true;
        }

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

        // 预转角完成调试信息
        ROS_WARN("[INIT_TURN] 预转角完成 | path=%s | 积分角度=%.2f° | 目标角度=%.2f° | wz=%.3f rad/s | "
                 "选中线点=%d | 阈值=%d | 角度达标=%d | 线点达标=%d",
                 pathToString(path_select).c_str(),
                 initial_turn_integrated_angle_deg,
                 initial_turn_angle_deg,
                 curent_wz,
                 selected_count,
                 initial_turn_rpts_threshold,
                 angle_ok, line_ok);
        return true;
    }//添加的角度和线判断指令

    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;

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

    // 预转角执行中调试信息
   // Initial turn execution debug info
ROS_WARN_THROTTLE(0.5,
                  "[INIT_TURN] Turning | path=%s | integrated_angle=%.2f°/%.2f° | wz=%.3f rad/s | "
                  "selected_points=%d/%d | dt=%.3fs | PID_output=%.3f | turn_direction=%s",
                  pathToString(path_select).c_str(),
                  initial_turn_integrated_angle_deg, initial_turn_angle_deg,
                  curent_wz,
                  selected_count, initial_turn_rpts_threshold,
                  dt, pid_speed,
                  motion_state == MotionState::ALIGNING_LEFT ? "LEFT" : "RIGHT");

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

    auto drawPointLabel = [&](float pts[][2], int pts_num, int idx,
                              const std::string &label, uint8_t gray,
                              bool cross_marker) {
        if (idx < 0 || idx >= pts_num) {
            return;
        }
        const int x = clip(static_cast<int>(std::round(pts[idx][0])), 0, RESULT_COL - 1);
        const int y = clip(static_cast<int>(std::round(pts[idx][1])), 0, RESULT_ROW - 1);
        const cv::Point p(x, y);
        const cv::Scalar color(gray);

        if (cross_marker) {
            cv::line(debug_gray, cv::Point(std::max(0, x - 6), y),
                     cv::Point(std::min(RESULT_COL - 1, x + 6), y), color, 2);
            cv::line(debug_gray, cv::Point(x, std::max(0, y - 6)),
                     cv::Point(x, std::min(RESULT_ROW - 1, y + 6)), color, 2);
        } else {
            cv::circle(debug_gray, p, 6, color, 2);
        }

        cv::putText(debug_gray, label,
                    cv::Point(std::min(RESULT_COL - 1, x + 8), std::max(12, y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1);
    };

    if (Lpt0_found) {
        drawPointLabel(rpts0s, rpts0s_num, Lpt0_rpts0s_id, "L0", 255, false);
    }
    if (Lpt1_found) {
        drawPointLabel(rpts1s, rpts1s_num, Lpt1_rpts1s_id, "L1", 220, false);
    }
    if (Ypt0_found) {
        drawPointLabel(rpts0s, rpts0s_num, Ypt0_rpts0s_id, "Y0", 200, true);
    }
    if (Ypt1_found) {
        drawPointLabel(rpts1s, rpts1s_num, Ypt1_rpts1s_id, "Y1", 180, true);
    }
    if (motion_state == MotionState::Y_CROSSBAR_SEEK && forward_crossbar_result.found) {
        const int x = clip(forward_crossbar_result.center_x, 0, RESULT_COL - 1);
        const int y = clip(forward_crossbar_result.center_y, 0, RESULT_ROW - 1);
        const int half_width = std::max(4, std::min(forward_crossbar_result.width_px / 2, RESULT_COL / 2));
        const cv::Scalar color(240);
        cv::line(debug_gray,
                 cv::Point(std::max(0, x - half_width), y),
                 cv::Point(std::min(RESULT_COL - 1, x + half_width), y),
                 color, 2);
        cv::line(debug_gray,
                 cv::Point(x, std::max(0, y - 6)),
                 cv::Point(x, std::min(RESULT_ROW - 1, y + 6)),
                 color, 2);
        cv::putText(debug_gray, "Y_BAR",
                    cv::Point(std::min(RESULT_COL - 1, x + 8), std::max(12, y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1);
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

    // 显示窗口（如果启用）
    if (show_window) {
        cv::imshow("follow_test", debug_gray);
        cv::waitKey(1);
    }

    // 视频保存（替代原来的ROS话题发布）
    if (publish_debug_image && enable_video_record) {
        // 第一次调用时初始化VideoWriter
        if (!video_recording) {
            // 生成带时间戳的文件名
            std::time_t now = std::time(nullptr);
            char timestamp[64];
            std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

            if (auto_video_save_path) {
                video_save_path = "/tmp/follow_test_debug_" + std::string(timestamp) + ".avi";
            }

            debug_video_writer.open(
                video_save_path,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                video_fps,
                cv::Size(RESULT_COL, RESULT_ROW),
                false  // 灰度图
            );

            if (debug_video_writer.isOpened()) {
                video_recording = true;
                ROS_INFO("[DEBUG_VIDEO] Started recording to: %s", video_save_path.c_str());
            } else {
                ROS_ERROR("[DEBUG_VIDEO] Failed to open video writer!");
            }
        }

        // 写入当前帧
        if (video_recording && debug_video_writer.isOpened()) {
            debug_video_writer.write(debug_gray);
        }
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

    if (handleYBranchFlow()) {
        return 0;
    }

    if (handleParkingCorner()) {
        // 检测到停车角点并完成停车后，本轮不再继续发布巡线速度。
        return 0;
    }

    if (rpts_num == 0) {
        zeroCount++;
        if (zeroCount >= 2) {
            zero_flag = true;
        }
    } else {
        zeroCount = 0;
        zero_flag = false;
    }

    MotionControlInput control_input;
    control_input.path = rpts;
    control_input.path_num = rpts_num;
    control_input.path_key = static_cast<int>(path_select);
    control_input.path_name = pathToString(path_select);
    control_input.degraded = is_degraded_mode;
    control_input.base_speed = base_speed;
    control_input.aim_distance = aim_distance;
    control_input.aim_y_bias_m = aim_y_bias_m;
    control_input.sample_dist = sample_dist;
    control_input.pixel_per_meter = pixel_per_meter;
    control_input.image_width = RESULT_COL;
    control_input.image_height = RESULT_ROW;
    control_input.allow_lost_coast = true;

    const MotionControlOutput control_output = motion_controller.compute(control_input);
    pub.publish(control_output.cmd);
    publishDebugImage();
// 主循环调试信息：输出当前选用的路径、路径点数量、是否退化、误差和速度，以及角点检测状态和丢线计数。
ROS_WARN_THROTTLE(1.0, "[FOLLOW] Running | path=%s | rpts=%d | degraded=%d | error=%.3f rad | v=%.3f m/s | L0=%d | L1=%d | Y0=%d | Y1=%d | lost_line_count=%d | zero_flag=%d",
                  pathToString(path_select).c_str(), rpts_num, is_degraded_mode,
                  control_output.filtered_error, control_output.cmd.linear.x,
                  Lpt0_found, Lpt1_found, Ypt0_found, Ypt1_found,
                  zeroCount, zero_flag);
ROS_WARN_THROTTLE(1.0, "[CONTROL] path=%s | rpts=%d | degraded=%d | raw_error=%.3f | filt_error=%.3f | target_v=%.3f | cmd_v=%.3f | cmd_wz=%.3f | lost=%d | coast=%d",
                  pathToString(path_select).c_str(), rpts_num, is_degraded_mode,
                  control_output.raw_error,
                  control_output.filtered_error,
                  control_output.target_v,
                  control_output.cmd.linear.x,
                  control_output.cmd.angular.z,
                  control_output.lost,
                  control_output.coasting);

    return 0;
}

std::string currentPathName() {
    return pathToString(path_select);
}

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double turn_angle_deg,
               double turn_angular_speed, int turn_rpts_threshold,
               double turn_pause_sec, double min_turn_pid_speed) {
    configure(publish_debug, show_debug_window, enable_parking,
              speed, distance, y_bias_m, enable_initial_turn,
              turn_angle_deg, turn_angular_speed, turn_rpts_threshold,
              turn_pause_sec, min_turn_pid_speed,
              y_approach_dist, y_turn_angle_deg,
              y_turn_angular_speed, y_turn_pause_sec,
              y_detect_min_id, y_detect_max_id, y_detect_confirm_frames,
              y_center_aim_dist, y_approach_speed, y_center_max_wz,
              y_lost_confirm_frames, y_entry_min_odom, y_entry_max_odom,
              parking_allow_either_l, parking_extra_dist, parking_forward_speed,
              y_crossbar_seek_speed, y_crossbar_lost_confirm_frames,
              y_crossbar_target_long_m, y_crossbar_long_tolerance_m,
              y_crossbar_max_abs_lat_m, y_crossbar_confirm_frames,
              y_crossbar_seek_max_odom);
}

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double turn_angle_deg,
               double turn_angular_speed, int turn_rpts_threshold,
               double turn_pause_sec, double min_turn_pid_speed,
               double branch_approach_dist, double branch_turn_angle_deg,
               double branch_turn_angular_speed, double branch_turn_pause_sec,
               int branch_detect_min_id, int branch_detect_max_id,
               int branch_detect_confirm_frames, double branch_center_aim_dist,
               double branch_approach_speed, double branch_center_max_wz,
               int branch_lost_confirm_frames, double branch_entry_min_odom,
               double branch_entry_max_odom, bool allow_either_l,
               double extra_dist, double forward_speed,
               double branch_crossbar_seek_speed,
               int branch_crossbar_lost_confirm_frames,
               double branch_crossbar_target_long_m,
               double branch_crossbar_long_tolerance_m,
               double branch_crossbar_max_abs_lat_m,
               int branch_crossbar_confirm_frames,
               double branch_crossbar_seek_max_odom) {
    // 保存 launch 参数，供后续图像调试、停车开关和速度控制使用。
    publish_debug_image = publish_debug;
    show_window = show_debug_window;
    parking_enabled = enable_parking;
    parking_allow_either_l = allow_either_l;
    parking_extra_dist = std::max(0.0, extra_dist);
    parking_forward_speed = std::max(0.01, forward_speed);
    base_speed = speed;
    aim_distance = distance;
    aim_y_bias_m = y_bias_m;
    initial_turn_enabled = enable_initial_turn;
    initial_turn_angle_deg = std::max(0.0, turn_angle_deg);
    initial_turn_angular_speed = std::max(0.0, turn_angular_speed);
    initial_turn_rpts_threshold = std::max(1, turn_rpts_threshold);
    initial_turn_pause_sec = std::max(0.0, turn_pause_sec);
    min_pid_speed = std::max(0.0, min_turn_pid_speed);
    y_approach_dist = std::max(0.0, branch_approach_dist);
    y_turn_angle_deg = std::max(0.0, branch_turn_angle_deg);
    y_turn_angular_speed = std::max(0.0, branch_turn_angular_speed);
    y_turn_pause_sec = std::max(0.0, branch_turn_pause_sec);
    y_detect_min_id = std::max(1, branch_detect_min_id);
    y_detect_max_id = std::max(1, branch_detect_max_id);
    y_detect_confirm_frames = std::max(1, branch_detect_confirm_frames);
    y_center_aim_dist = std::max(0.01, branch_center_aim_dist);
    y_approach_speed = std::max(0.0, branch_approach_speed);
    y_center_max_wz = std::max(0.0, branch_center_max_wz);
    y_lost_confirm_frames = std::max(1, branch_lost_confirm_frames);
    y_entry_min_odom = std::max(0.0, branch_entry_min_odom);
    y_entry_max_odom = std::max(y_entry_min_odom, branch_entry_max_odom);
    y_crossbar_seek_speed = std::max(0.0, branch_crossbar_seek_speed);
    y_crossbar_lost_confirm_frames = std::max(1, branch_crossbar_lost_confirm_frames);
    y_crossbar_target_long_m = std::max(0.0, branch_crossbar_target_long_m);
    y_crossbar_long_tolerance_m = std::max(0.0, branch_crossbar_long_tolerance_m);
    y_crossbar_max_abs_lat_m = std::max(0.0, branch_crossbar_max_abs_lat_m);
    y_crossbar_confirm_frames = std::max(1, branch_crossbar_confirm_frames);
    y_crossbar_seek_max_odom = std::max(0.0, branch_crossbar_seek_max_odom);
}

void configureVideo(bool enable_record, int fps, const std::string &save_path) {
    enable_video_record = enable_record;
    video_fps = std::max(1, std::min(30, fps));  // 限制在1-30 FPS范围内
    auto_video_save_path = save_path.empty();
    if (!save_path.empty()) {
        video_save_path = save_path;
    }
    // 如果禁用了视频录制，确保关闭已打开的writer
    if (!enable_record && debug_video_writer.isOpened()) {
        debug_video_writer.release();
        video_recording = false;
        ROS_INFO("[DEBUG_VIDEO] Video recording disabled.");
    }
}

void initializeImagePipeline() {
    // 建立逆透视映射表 point_map/PerImg_ip，process_image() 依赖这些查找表。
    ImagePerspective_Init();
    Global_move_timer = ros::Time::now();
}

bool shouldExit() {
    return sig_INT.load();
}

void shutdown() {
    // 节点退出时主动停车，避免调试时 Ctrl-C 后底盘保留上一条速度。
    publishStop();

    // 关闭视频录制
    if (video_recording && debug_video_writer.isOpened()) {
        debug_video_writer.release();
        video_recording = false;
        ROS_INFO("[DEBUG_VIDEO] Video saved to: %s", video_save_path.c_str());
    }
}

}  // namespace follow_test
}  // namespace flow_end
