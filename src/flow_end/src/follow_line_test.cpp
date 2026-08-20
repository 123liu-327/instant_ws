#include <flow_end/follow.h>
#include <flow_end/follow_line_test.h>
#include <flow_end/follow_motion_controller.h>
#include <flow_end/parking_s_curve.h>
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
#include <iomanip>
#include <sstream>
#include <string>
#include <limits>

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

int Ypt0_rpts0s_id = -1, Ypt1_rpts1s_id = -1;
bool Ypt0_found = false, Ypt1_found = false;
int Lpt0_rpts0s_id = -1, Lpt1_rpts1s_id = -1;
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
FollowMotionController motion_controller;

// These runtime params are written through configure(); Callback_test.cpp refreshes
// them from the private parameter server before each start command.
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
ros::Time last_odom_time;
ros::Time last_imu_time;
double current_odom_position_x = 0.0;
double current_odom_position_y = 0.0;
double odom_total_distance = 0.0;
double base_speed = 0.30;
double aim_distance = 0.10;
double aim_y_bias_m = 0.20;
bool right_turn_assist_enabled = true;
double right_turn_min_aim_distance = 0.06;
double right_turn_error_start = 0.18;
double right_turn_error_full = 0.40;
double right_turn_min_speed = 0.08;
double right_turn_wz_compensation = 0.12;
double right_turn_max_wz = 0.70;
double right_turn_odom_timeout = 0.25;
double right_turn_odom_response_ratio = 0.60;
int right_turn_odom_confirm_frames = 3;
double current_odom_angular_velocity_z = 0.0;
int right_turn_odom_underresponse_count = 0;
std::string vision_source = "gray";
double line_track_timeout = 0.25;
double line_track_min_confidence = 0.20;
int line_track_min_points = 20;
std::mutex line_track_mutex;
line_follower::LineTrack latest_line_track;
ros::Time last_line_track_receive_time;
bool has_line_track = false;
bool external_vision_lost = false;
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
bool lost_corner_search_enabled = true;
double lost_corner_search_timeout = 0.6;
double lost_corner_search_angular_speed = 0.25;
double lost_corner_search_linear_speed = 0.0;
bool lost_corner_search_active = false;
bool lost_corner_search_timed_out = false;
ros::Time lost_corner_search_start_time;

bool y_branch_mode_requested = false;
PathSelect pending_branch_path = PathSelect::MIDDLE;
double y_turn_angle_deg = 45.0;
double y_turn_angular_speed = 0.35;
double y_turn_pause_sec = 0.5;
int y_detect_min_id = 0;
int y_detect_max_id = 20;
int y_detect_confirm_frames = 2;
double y_extra_forward_dist = 0.20;
double y_hard_drive_speed = 0.15;
double y_hard_drive_odom_timeout = 0.25;
double y_hard_drive_max_duration = 8.0;
int y_guided_min_points = 20;
int y_guided_lost_confirm_frames = 2;
double y_guided_error_threshold = 0.18;
int y_guided_error_confirm_frames = 2;
double y_guided_odom_timeout = 0.25;
double y_guided_max_duration = 8.0;
double y_hard_heading_kp = 1.2;
double y_hard_heading_max_wz = 0.15;
double y_hard_heading_deadband_deg = 2.0;
double y_hard_heading_imu_timeout = 0.25;
double y_reacquire_speed = 0.08;
double y_reacquire_max_dist = 0.25;
double y_reacquire_odom_timeout = 0.25;
double y_reacquire_max_duration = 5.0;
int y_reacquire_min_points = 20;
int y_reacquire_confirm_frames = 2;
double y_crossbar_seek_speed = 0.08;
int y_crossbar_lost_confirm_frames = 3;
double y_crossbar_target_long_m = 0.25;
double y_crossbar_long_tolerance_m = 0.10;
double y_crossbar_max_abs_lat_m = 0.18;
int y_crossbar_confirm_frames = 2;
double y_crossbar_seek_max_odom = 0.40;

int y_detect_confirm_count = 0;
int y_crossbar_lost_count = 0;
int y_crossbar_confirm_count = 0;
float y_crossbar_seek_start_odom = 0.0f;
float y_hard_drive_start_odom = 0.0f;
double y_hard_drive_start_total_odom = 0.0;
double y_detected_long_m = 0.0;
double y_hard_drive_target_dist = 0.0;
double y_hard_drive_remaining_dist = 0.0;
float y_detected_y0_px = -1.0f;
float y_detected_y1_px = -1.0f;
ros::Time y_hard_drive_start_time;
bool y_hard_drive_aborted = false;
std::string y_hard_drive_abort_reason;
double y_guided_target_dist = 0.0;
double y_guided_start_total_odom = 0.0;
double y_guided_moved_dist = 0.0;
double y_guided_remaining_dist = 0.0;
double y_guided_visual_error = 0.0;
ros::Time y_guided_start_time;
int y_guided_lost_count = 0;
int y_guided_error_count = 0;
double y_last_reliable_yaw_deg = 0.0;
bool y_has_reliable_yaw = false;
double y_hard_heading_reference_deg = 0.0;
std::string y_hard_drive_trigger_reason;
std::string y_guided_hold_reason;
ros::Time y_turn_last_time;
bool y_turn_has_last_time = false;
double y_turn_integrated_angle_deg = 0.0;
ros::Time y_turn_pause_start;
double y_reacquire_start_x = 0.0;
double y_reacquire_start_y = 0.0;
double y_reacquire_moved_dist = 0.0;
double y_reacquire_remaining_dist = 0.0;
ros::Time y_reacquire_start_time;
int y_reacquire_confirm_count = 0;
int y_reacquire_visible_points = 0;
std::string y_reacquire_hold_reason;

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

void applyPathBiasParams(PathSelect path) {
    ros::NodeHandle private_nh("~");
    const std::string prefix = pathToString(path);

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

std::string motionStateToString(MotionState state) {
    switch (state) {
        case MotionState::IDLE: return "IDLE";
        case MotionState::ALIGNING_LEFT: return "ALIGNING_LEFT";
        case MotionState::ALIGNING_RIGHT: return "ALIGNING_RIGHT";
        case MotionState::ALIGN_PAUSE: return "ALIGN_PAUSE";
        case MotionState::FOLLOWING: return "FOLLOWING";
        case MotionState::FOLLOWING_STRAIGHT: return "FOLLOWING_STRAIGHT";
        case MotionState::Y_GUIDED_FORWARD: return "Y_GUIDED_FORWARD";
        case MotionState::Y_COORDINATE_FORWARD: return "Y_COORDINATE_FORWARD";
        case MotionState::Y_CROSSBAR_SEEK: return "Y_CROSSBAR_SEEK";
        case MotionState::Y_ALIGNING_LEFT: return "Y_ALIGNING_LEFT";
        case MotionState::Y_ALIGNING_RIGHT: return "Y_ALIGNING_RIGHT";
        case MotionState::Y_ALIGN_PAUSE: return "Y_ALIGN_PAUSE";
        case MotionState::Y_BRANCH_REACQUIRE: return "Y_BRANCH_REACQUIRE";
    }
    return "UNKNOWN";
}

void publishStatus(const std::string &state);
void publishDebugImage(const sensor_msgs::ImageConstPtr &source_msg = sensor_msgs::ImageConstPtr());

void resetMotionController() {
    motion_controller.reset();
    right_turn_odom_underresponse_count = 0;
}

void configureVisionSource(const std::string &source, double timeout,
                           double min_confidence, int min_points) {
    const std::string normalized = normalize(source);
    if (normalized == "gray" || normalized == "line_follower") {
        vision_source = normalized;
    } else {
        ROS_WARN("[VISION] Invalid vision_source '%s', fallback to gray", source.c_str());
        vision_source = "gray";
    }
    line_track_timeout = std::max(0.01, timeout);
    line_track_min_confidence = std::max(0.0, std::min(1.0, min_confidence));
    line_track_min_points = std::max(1, min_points);
}

void updateLineTrack(const line_follower::LineTrack::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(line_track_mutex);
    latest_line_track = *msg;
    last_line_track_receive_time = ros::Time::now();
    has_line_track = true;
}

void resetExternalVisionState() {
    std::lock_guard<std::mutex> lock(line_track_mutex);
    has_line_track = false;
    last_line_track_receive_time = ros::Time();
    external_vision_lost = false;
    right_turn_odom_underresponse_count = 0;
}

bool externalTrackSnapshot(line_follower::LineTrack &track,
                           double &receive_age, double &source_age,
                           std::string &reason) {
    ros::Time receive_time;
    {
        std::lock_guard<std::mutex> lock(line_track_mutex);
        if (!has_line_track) {
            reason = "no_message";
            return false;
        }
        track = latest_line_track;
        receive_time = last_line_track_receive_time;
    }

    const ros::Time now = ros::Time::now();
    receive_age = (now - receive_time).toSec();
    source_age = track.header.stamp.isZero()
                     ? std::numeric_limits<double>::infinity()
                     : (now - track.header.stamp).toSec();
    if (receive_age < 0.0 || receive_age > line_track_timeout) {
        reason = "receive_timeout";
        return false;
    }
    if (!std::isfinite(source_age) || source_age < -0.10 ||
        source_age > line_track_timeout) {
        reason = "source_timeout";
        return false;
    }
    if (!track.valid) {
        reason = "invalid_flag";
        return false;
    }
    if (track.confidence < line_track_min_confidence) {
        reason = "low_confidence";
        return false;
    }
    if (track.image_width != RESULT_COL || track.image_height != RESULT_ROW) {
        reason = "image_size";
        return false;
    }
    if (track.center_x_px.size() != track.center_y_px.size() ||
        static_cast<int>(track.center_x_px.size()) < line_track_min_points) {
        reason = "point_count";
        return false;
    }
    return true;
}

void configureRightTurnAssist(bool enabled, double min_aim_distance,
                              double error_start, double error_full,
                              double min_speed, double wz_compensation,
                              double max_wz, double odom_timeout,
                              double odom_response_ratio,
                              int odom_confirm_frames) {
    right_turn_assist_enabled = enabled;
    right_turn_min_aim_distance = std::max(0.01, min_aim_distance);
    right_turn_error_start = std::max(0.0, error_start);
    right_turn_error_full = std::max(right_turn_error_start + 0.001, error_full);
    right_turn_min_speed = std::max(0.0, min_speed);
    right_turn_wz_compensation = std::max(0.0, wz_compensation);
    right_turn_max_wz = std::max(0.0, max_wz);
    right_turn_odom_timeout = std::max(0.0, odom_timeout);
    right_turn_odom_response_ratio =
        std::max(0.0, std::min(1.0, odom_response_ratio));
    right_turn_odom_confirm_frames = std::max(1, odom_confirm_frames);
    right_turn_odom_underresponse_count = 0;
}

void resetYBranchState() {
    y_branch_mode_requested = false;
    pending_branch_path = PathSelect::MIDDLE;
    y_detect_confirm_count = 0;
    y_crossbar_lost_count = 0;
    y_crossbar_confirm_count = 0;
    y_crossbar_seek_start_odom = odom_dist;
    y_hard_drive_start_odom = odom_dist;
    y_hard_drive_start_total_odom = odom_total_distance;
    y_detected_long_m = 0.0;
    y_hard_drive_target_dist = 0.0;
    y_hard_drive_remaining_dist = 0.0;
    y_detected_y0_px = -1.0f;
    y_detected_y1_px = -1.0f;
    y_hard_drive_start_time = ros::Time();
    y_hard_drive_aborted = false;
    y_hard_drive_abort_reason.clear();
    y_guided_target_dist = 0.0;
    y_guided_start_total_odom = odom_total_distance;
    y_guided_moved_dist = 0.0;
    y_guided_remaining_dist = 0.0;
    y_guided_visual_error = 0.0;
    y_guided_start_time = ros::Time();
    y_guided_lost_count = 0;
    y_guided_error_count = 0;
    y_last_reliable_yaw_deg = current_yaw;
    y_has_reliable_yaw = false;
    y_hard_heading_reference_deg = current_yaw;
    y_hard_drive_trigger_reason.clear();
    y_guided_hold_reason.clear();
    y_turn_last_time = ros::Time();
    y_turn_has_last_time = false;
    y_turn_integrated_angle_deg = 0.0;
    y_turn_pause_start = ros::Time();
    y_reacquire_start_x = current_odom_position_x;
    y_reacquire_start_y = current_odom_position_y;
    y_reacquire_moved_dist = 0.0;
    y_reacquire_remaining_dist = 0.0;
    y_reacquire_start_time = ros::Time();
    y_reacquire_confirm_count = 0;
    y_reacquire_visible_points = 0;
    y_reacquire_hold_reason.clear();
    forward_crossbar_result = {false, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f};
    resetMotionController();
}

void resetLostCornerSearchState() {
    lost_corner_search_active = false;
    lost_corner_search_timed_out = false;
    lost_corner_search_start_time = ros::Time();
}

void resetParkingCornerState() {
    resetLostCornerSearchState();
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
        y_detect_confirm_count = 0;
        y_crossbar_lost_count = 0;
        y_crossbar_confirm_count = 0;
        y_turn_integrated_angle_deg = 0.0;
        y_turn_has_last_time = false;
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
        resetYBranchState();
        path_select = PathSelect::MIDDLE;
        pending_branch_path = PathSelect::LEFT;
        y_branch_mode_requested = true;
        track_type = TRACK_MIDDLE;
        resetLostCornerSearchState();
        return true;
    }
    if (value == "yright" || value == "y_right" || value == "yr") {
        resetYBranchState();
        path_select = PathSelect::MIDDLE;
        pending_branch_path = PathSelect::RIGHT;
        y_branch_mode_requested = true;
        track_type = TRACK_MIDDLE;
        resetLostCornerSearchState();
        return true;
    }
    if (value == "left" || value == "l") {
        resetYBranchState();
        path_select = PathSelect::LEFT;
        track_type = TRACK_LEFT;
        resetLostCornerSearchState();
        return true;
    }
    if (value == "middle" || value == "mid" || value == "center" || value == "centre" || value == "m") {
        resetYBranchState();
        path_select = PathSelect::MIDDLE;
        track_type = TRACK_MIDDLE;
        resetLostCornerSearchState();
        return true;
    }
    if (value == "right" || value == "r") {
        resetYBranchState();
        path_select = PathSelect::RIGHT;
        track_type = TRACK_RIGHT;
        resetLostCornerSearchState();
        return true;
    }
    return false;
}

void publishStop() {
    resetMotionController();
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

void detectCorners() {
    // 角点检测沿用原 follow_line.cpp 的思路：
    // 对左右边线的局部角度数组 rpts0a/rpts1a 做非极大值式判断，
    // 30~65 度认为可能是 Y 型岔口角点，70~110 度认为可能是 L 型直角点。
    // follow_test 主要保留 L 型停车逻辑，Y 点结果先作为调试信息输出。
    Ypt1_found = false;
    Lpt1_found = false;
    Ypt0_found = false;
    Lpt0_found = false;
    Ypt0_rpts0s_id = -1;
    Ypt1_rpts1s_id = -1;
    Lpt0_rpts0s_id = -1;
    Lpt1_rpts1s_id = -1;
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

bool calculateYCoordinateTarget(double &long_m) {
    double y_sum = 0.0;
    int valid_count = 0;
    y_detected_y0_px = -1.0f;
    y_detected_y1_px = -1.0f;

    if (Ypt0_found && Ypt0_rpts0s_id >= y_detect_min_id &&
        Ypt0_rpts0s_id <= y_detect_max_id &&
        Ypt0_rpts0s_id < rpts0s_num) {
        y_detected_y0_px = rpts0s[Ypt0_rpts0s_id][1];
        if (std::isfinite(y_detected_y0_px)) {
            y_sum += y_detected_y0_px;
            ++valid_count;
        }
    }
    if (Ypt1_found && Ypt1_rpts1s_id >= y_detect_min_id &&
        Ypt1_rpts1s_id <= y_detect_max_id &&
        Ypt1_rpts1s_id < rpts1s_num) {
        y_detected_y1_px = rpts1s[Ypt1_rpts1s_id][1];
        if (std::isfinite(y_detected_y1_px)) {
            y_sum += y_detected_y1_px;
            ++valid_count;
        }
    }
    if (valid_count == 0 || pixel_per_meter <= 1.0f) {
        return false;
    }

    const double average_y_px = y_sum / static_cast<double>(valid_count);
    const double ref_y = RESULT_ROW + 10.0;
    long_m = -(average_y_px - ref_y) / pixel_per_meter;
    // Y 点检测本身限制在约 0.8 m 的重采样边线范围内；这里留少量
    // IPM/曲线误差余量，同时拒绝车后方或明显失真的坐标。
    return std::isfinite(long_m) && long_m >= 0.0 && long_m <= 1.20;
}

bool calculateGrayControlError(double distance, double &error) {
    if (rpts_num <= 0 || rpts == nullptr) {
        return false;
    }
    const int aim_idx = clip(
        static_cast<int>(std::round(distance / sample_dist)),
        0, rpts_num - 1);
    const float cx = RESULT_COL / 2.0f;
    const float cy = RESULT_ROW + 10.0f;
    const float dx = rpts[aim_idx][0] - cx;
    const float dy = cy - rpts[aim_idx][1] +
        aim_y_bias_m * pixel_per_meter;
    error = -std::atan2(static_cast<double>(dx), static_cast<double>(dy));
    return std::isfinite(error);
}

void enterYTurn() {
    resetMotionController();
    motion_state = pending_branch_path == PathSelect::LEFT
        ? MotionState::Y_ALIGNING_LEFT
        : MotionState::Y_ALIGNING_RIGHT;
    y_turn_integrated_angle_deg = 0.0;
    y_turn_last_time = ros::Time::now();
    y_turn_has_last_time = true;
    pid.reset();
    publishStatus("Y_TURN_" + pathToString(pending_branch_path));
}

void enterYRemainingHardForward(const std::string &reason) {
    geometry_msgs::Twist stop_msg;
    pub.publish(stop_msg);
    y_guided_moved_dist = std::max(
        0.0, odom_total_distance - y_guided_start_total_odom);
    y_guided_remaining_dist = std::max(
        0.0, y_guided_target_dist - y_guided_moved_dist);
    if (y_guided_remaining_dist <= 1e-3) {
        ROS_WARN(
            "[Y_GUIDED] Target already reached; skip hard forward | reason=%s | target=%.3fm | guided=%.3fm",
            reason.c_str(), y_guided_target_dist, y_guided_moved_dist);
        enterYTurn();
        return;
    }

    resetMotionController();
    motion_state = MotionState::Y_COORDINATE_FORWARD;
    y_hard_drive_start_odom = odom_dist;
    y_hard_drive_start_total_odom = odom_total_distance;
    y_hard_drive_start_time = ros::Time::now();
    y_hard_drive_target_dist = y_guided_remaining_dist;
    y_hard_drive_remaining_dist = y_guided_remaining_dist;
    y_hard_drive_aborted = false;
    y_hard_drive_abort_reason.clear();
    y_hard_drive_trigger_reason = reason;
    y_hard_heading_reference_deg = y_has_reliable_yaw
        ? y_last_reliable_yaw_deg : current_yaw;
    publishStatus("Y_REMAINING_FORWARD_" + pathToString(pending_branch_path));
    ROS_WARN(
        "[Y_GUIDED] Switch to remaining hard forward | reason=%s | target=%.3fm | guided=%.3fm | remaining=%.3fm | ref_yaw=%.2fdeg | has_reliable_yaw=%d | speed=%.2fm/s",
        reason.c_str(), y_guided_target_dist, y_guided_moved_dist,
        y_guided_remaining_dist, y_hard_heading_reference_deg,
        y_has_reliable_yaw, y_hard_drive_speed);
}

bool handleYBranchFlow() {
    if (motion_state == MotionState::FOLLOWING_STRAIGHT) {
        const bool y0_in_range = Ypt0_found &&
            Ypt0_rpts0s_id >= y_detect_min_id &&
            Ypt0_rpts0s_id <= y_detect_max_id;
        const bool y1_in_range = Ypt1_found &&
            Ypt1_rpts1s_id >= y_detect_min_id &&
            Ypt1_rpts1s_id <= y_detect_max_id;
        const bool y_seen = y0_in_range || y1_in_range;
        const bool lost_all_lines =
            (rpts_num == 0 && rptsc0e_num == 0 && rptsc1e_num == 0);

        if (y_seen) {
            ++y_detect_confirm_count;
            y_crossbar_lost_count = 0;
        } else {
            y_detect_confirm_count = 0;
            y_crossbar_lost_count = lost_all_lines ? y_crossbar_lost_count + 1 : 0;
        }

        ROS_WARN_THROTTLE(
            0.5,
            "[Y_BRANCH] Searching | next_path=%s | Y0=%d(id=%d,in=%d) | Y1=%d(id=%d,in=%d) | id_range=%d~%d | y_seen=%d | y_confirm=%d/%d | lost=%d/%d",
            pathToString(pending_branch_path).c_str(),
            Ypt0_found, Ypt0_rpts0s_id, y0_in_range,
            Ypt1_found, Ypt1_rpts1s_id, y1_in_range,
            y_detect_min_id, y_detect_max_id,
            y_seen,
            y_detect_confirm_count, y_detect_confirm_frames,
            y_crossbar_lost_count, y_crossbar_lost_confirm_frames);

        if (y_detect_confirm_count >= y_detect_confirm_frames) {
            double detected_long_m = 0.0;
            if (!calculateYCoordinateTarget(detected_long_m)) {
                resetMotionController();
                motion_state = MotionState::Y_CROSSBAR_SEEK;
                y_crossbar_seek_start_odom = odom_dist;
                y_crossbar_confirm_count = 0;
                pid.reset();
                publishStatus("Y_CROSSBAR_SEEK_" + pathToString(pending_branch_path));
                ROS_ERROR(
                    "[Y_COORD] Invalid Y coordinate, use crossbar fallback | next_path=%s | Y0=%d(id=%d,y=%.1f) | Y1=%d(id=%d,y=%.1f) | ppm=%.1f",
                    pathToString(pending_branch_path).c_str(),
                    Ypt0_found, Ypt0_rpts0s_id, y_detected_y0_px,
                    Ypt1_found, Ypt1_rpts1s_id, y_detected_y1_px,
                    pixel_per_meter);
                return true;
            }

            y_detected_long_m = detected_long_m;
            y_guided_target_dist =
                y_detected_long_m + y_extra_forward_dist;
            resetMotionController();
            motion_state = MotionState::Y_GUIDED_FORWARD;
            y_guided_start_total_odom = odom_total_distance;
            y_guided_moved_dist = 0.0;
            y_guided_remaining_dist = y_guided_target_dist;
            y_guided_visual_error = 0.0;
            y_guided_start_time = ros::Time::now();
            y_guided_lost_count = 0;
            y_guided_error_count = 0;
            const double imu_age = last_imu_time.isZero()
                ? std::numeric_limits<double>::infinity()
                : (ros::Time::now() - last_imu_time).toSec();
            y_has_reliable_yaw = imu_age >= 0.0 &&
                imu_age <= y_hard_heading_imu_timeout;
            y_last_reliable_yaw_deg = current_yaw;
            y_guided_hold_reason.clear();
            y_crossbar_lost_count = 0;
            y_crossbar_confirm_count = 0;
            publishStatus("Y_GUIDED_FORWARD_" + pathToString(pending_branch_path));
            ROS_WARN(
                "[Y_GUIDED] Middle-guided target started | next_path=%s | Y0=%d(id=%d,y=%.1f) | Y1=%d(id=%d,y=%.1f) | long=%.3fm | extra=%.3fm | target=%.3fm | total_odom=%.3fm | yaw=%.2fdeg | imu_fresh=%d",
                pathToString(pending_branch_path).c_str(),
                Ypt0_found, Ypt0_rpts0s_id, y_detected_y0_px,
                Ypt1_found, Ypt1_rpts1s_id, y_detected_y1_px,
                y_detected_long_m, y_extra_forward_dist,
                y_guided_target_dist, y_guided_start_total_odom,
                current_yaw, y_has_reliable_yaw);
            return false;
        }

        if (y_crossbar_lost_count >= y_crossbar_lost_confirm_frames) {
            resetMotionController();
            motion_state = MotionState::Y_CROSSBAR_SEEK;
            y_crossbar_seek_start_odom = odom_dist;
            y_crossbar_confirm_count = 0;
            pid.reset();
            publishStatus("Y_CROSSBAR_SEEK_" + pathToString(pending_branch_path));
            ROS_WARN(
                "[Y_BRANCH] Lost Y and lines, seeking crossbar | next_path=%s | lost=%d/%d | odom=%.3fm",
                pathToString(pending_branch_path).c_str(),
                y_crossbar_lost_count, y_crossbar_lost_confirm_frames,
                odom_dist);
            return true;
        }
        return false;
    }

    if (motion_state == MotionState::Y_GUIDED_FORWARD) {
        const ros::Time now = ros::Time::now();
        y_guided_moved_dist = std::max(
            0.0, odom_total_distance - y_guided_start_total_odom);
        y_guided_remaining_dist = std::max(
            0.0, y_guided_target_dist - y_guided_moved_dist);
        const double odom_age = last_odom_time.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - last_odom_time).toSec();
        const bool odom_fresh = odom_age >= 0.0 &&
            odom_age <= y_guided_odom_timeout;
        const double elapsed = y_guided_start_time.isZero()
            ? 0.0 : (now - y_guided_start_time).toSec();

        std::string hold_reason;
        if (!odom_fresh) {
            hold_reason = "odom_stale";
        } else if (elapsed >= y_guided_max_duration) {
            hold_reason = "timeout";
        }
        if (!hold_reason.empty()) {
            geometry_msgs::Twist stop_msg;
            pub.publish(stop_msg);
            publishDebugImage();
            if (hold_reason != y_guided_hold_reason) {
                y_guided_hold_reason = hold_reason;
                publishStatus("Y_GUIDED_HOLD_" + hold_reason);
                ROS_ERROR(
                    "[Y_GUIDED] Holding stop | reason=%s | target=%.3fm | guided=%.3fm | remaining=%.3fm | elapsed=%.2fs/%.2fs | odom_age=%.3fs",
                    hold_reason.c_str(), y_guided_target_dist,
                    y_guided_moved_dist, y_guided_remaining_dist,
                    elapsed, y_guided_max_duration, odom_age);
            }
            return true;
        }
        if (!y_guided_hold_reason.empty()) {
            y_guided_hold_reason.clear();
            publishStatus("Y_GUIDED_FORWARD_" + pathToString(pending_branch_path));
        }

        if (y_guided_moved_dist >= y_guided_target_dist) {
            geometry_msgs::Twist stop_msg;
            pub.publish(stop_msg);
            ROS_WARN(
                "[Y_GUIDED] Target reached under normal middle control | target=%.3fm | guided=%.3fm | elapsed=%.2fs",
                y_guided_target_dist, y_guided_moved_dist, elapsed);
            enterYTurn();
            return true;
        }

        double visual_error = 0.0;
        const bool enough_points = rpts_num >= y_guided_min_points;
        const bool error_valid = enough_points &&
            calculateGrayControlError(aim_distance, visual_error);
        y_guided_visual_error = error_valid ? visual_error : 0.0;
        y_guided_lost_count = error_valid ? 0 : y_guided_lost_count + 1;
        const bool error_exceeded = error_valid &&
            std::abs(visual_error) > y_guided_error_threshold;
        y_guided_error_count = error_exceeded
            ? y_guided_error_count + 1 : 0;

        const double imu_age = last_imu_time.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - last_imu_time).toSec();
        const bool imu_fresh = imu_age >= 0.0 &&
            imu_age <= y_hard_heading_imu_timeout;
        if (error_valid && !error_exceeded && imu_fresh) {
            y_last_reliable_yaw_deg = current_yaw;
            y_has_reliable_yaw = true;
        }

        if (y_guided_lost_count >= y_guided_lost_confirm_frames) {
            enterYRemainingHardForward("line_lost");
            return true;
        }
        if (y_guided_error_count >= y_guided_error_confirm_frames) {
            enterYRemainingHardForward("visual_error");
            return true;
        }

        ROS_WARN_THROTTLE(
            0.5,
            "[Y_GUIDED] Middle control active | target=%.3fm | guided=%.3fm | remaining=%.3fm | points=%d/%d | error=%.3frad/%.3frad | lost=%d/%d | error_confirm=%d/%d | yaw=%.2fdeg | reliable_yaw=%.2fdeg(valid=%d) | odom_age=%.3fs | elapsed=%.2fs",
            y_guided_target_dist, y_guided_moved_dist,
            y_guided_remaining_dist, rpts_num, y_guided_min_points,
            y_guided_visual_error, y_guided_error_threshold,
            y_guided_lost_count, y_guided_lost_confirm_frames,
            y_guided_error_count, y_guided_error_confirm_frames,
            current_yaw, y_last_reliable_yaw_deg, y_has_reliable_yaw,
            odom_age, elapsed);
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
            ++y_crossbar_confirm_count;
        } else {
            y_crossbar_confirm_count = 0;
        }

        const bool reached_by_crossbar =
            y_crossbar_confirm_count >= y_crossbar_confirm_frames;
        const bool reached_by_max_odom =
            moved >= static_cast<float>(y_crossbar_seek_max_odom);
        if (reached_by_crossbar || reached_by_max_odom) {
            resetMotionController();
            motion_state = pending_branch_path == PathSelect::LEFT
                ? MotionState::Y_ALIGNING_LEFT
                : MotionState::Y_ALIGNING_RIGHT;
            y_turn_integrated_angle_deg = 0.0;
            y_turn_last_time = ros::Time::now();
            y_turn_has_last_time = true;
            pid.reset();
            publishStatus("Y_TURN_" + pathToString(pending_branch_path));
            ROS_WARN(
                "[Y_CROSSBAR] Trigger turn | reason=%s | next_path=%s | found=%d | center=(%d,%d) | long=%.3fm | lat=%.3fm | confirm=%d/%d | moved=%.3fm",
                reached_by_crossbar ? "crossbar" : "max_odom",
                pathToString(pending_branch_path).c_str(),
                found,
                forward_crossbar_result.center_x,
                forward_crossbar_result.center_y,
                forward_crossbar_result.long_m,
                forward_crossbar_result.lat_m,
                y_crossbar_confirm_count, y_crossbar_confirm_frames,
                moved);
            forward_crossbar_result.found = false;
            return true;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = y_crossbar_seek_speed;
        msg.angular.z = 0.0;
        pub.publish(msg);
        publishDebugImage();
        ROS_WARN_THROTTLE(
            0.5,
            "[Y_CROSSBAR] Seeking | found=%d | center=(%d,%d) | long=%.3fm | lat=%.3fm | confirm=%d/%d | moved=%.3fm | v=%.2f",
            found,
            forward_crossbar_result.center_x,
            forward_crossbar_result.center_y,
            forward_crossbar_result.long_m,
            forward_crossbar_result.lat_m,
            y_crossbar_confirm_count, y_crossbar_confirm_frames,
            moved, msg.linear.x);
        return true;
    }

    if (motion_state == MotionState::Y_COORDINATE_FORWARD) {
        const ros::Time now = ros::Time::now();
        const double moved = std::max(
            0.0, odom_total_distance - y_hard_drive_start_total_odom);
        y_hard_drive_remaining_dist = std::max(
            0.0, y_hard_drive_target_dist - moved);
        const double odom_age = last_odom_time.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - last_odom_time).toSec();
        const bool odom_fresh = odom_age >= 0.0 &&
            odom_age <= y_hard_drive_odom_timeout;
        const double imu_age = last_imu_time.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - last_imu_time).toSec();
        const bool imu_fresh = imu_age >= 0.0 &&
            imu_age <= y_hard_heading_imu_timeout;
        const double elapsed = y_hard_drive_start_time.isZero()
            ? 0.0 : (now - y_hard_drive_start_time).toSec();

        if (!y_hard_drive_aborted && !odom_fresh) {
            y_hard_drive_aborted = true;
            y_hard_drive_abort_reason = "odom_stale";
            publishStatus("Y_COORDINATE_ABORTED_ODOM");
            ROS_ERROR(
                "[Y_COORD] Abort hard forward: odometry missing/stale | age=%.3fs | timeout=%.3fs",
                odom_age, y_hard_drive_odom_timeout);
        }
        if (!y_hard_drive_aborted &&
            elapsed > y_hard_drive_max_duration) {
            y_hard_drive_aborted = true;
            y_hard_drive_abort_reason = "timeout";
            publishStatus("Y_COORDINATE_ABORTED_TIMEOUT");
            ROS_ERROR(
                "[Y_COORD] Abort hard forward: timeout | elapsed=%.2fs | limit=%.2fs | moved=%.3fm",
                elapsed, y_hard_drive_max_duration, moved);
        }
        if (!y_hard_drive_aborted && !imu_fresh) {
            y_hard_drive_aborted = true;
            y_hard_drive_abort_reason = "imu_stale";
            publishStatus("Y_COORDINATE_ABORTED_IMU");
            ROS_ERROR(
                "[Y_COORD] Abort remaining hard forward: IMU missing/stale | age=%.3fs | timeout=%.3fs",
                imu_age, y_hard_heading_imu_timeout);
        }

        if (y_hard_drive_aborted) {
            geometry_msgs::Twist stop_msg;
            pub.publish(stop_msg);
            publishDebugImage();
            ROS_ERROR_THROTTLE(
                1.0,
                "[Y_COORD] Holding stop after abort | reason=%s | trigger=%s | moved=%.3fm/%.3fm | remaining=%.3fm | odom_age=%.3fs | imu_age=%.3fs",
                y_hard_drive_abort_reason.c_str(),
                y_hard_drive_trigger_reason.c_str(), moved,
                y_hard_drive_target_dist, y_hard_drive_remaining_dist,
                odom_age, imu_age);
            return true;
        }

        if (moved >= y_hard_drive_target_dist) {
            geometry_msgs::Twist stop_msg;
            pub.publish(stop_msg);
            y_hard_drive_remaining_dist = 0.0;
            ROS_WARN(
                "[Y_COORD] Remaining target reached | next_path=%s | trigger=%s | overall_target=%.3fm | guided=%.3fm | hard_target=%.3fm | hard_moved=%.3fm | elapsed=%.2fs",
                pathToString(pending_branch_path).c_str(),
                y_hard_drive_trigger_reason.c_str(),
                y_guided_target_dist, y_guided_moved_dist,
                y_hard_drive_target_dist, moved, elapsed);
            enterYTurn();
            return true;
        }

        geometry_msgs::Twist msg;
        msg.linear.x = y_hard_drive_speed;
        const double heading_error_deg = normalizeAngleDeg(
            y_hard_heading_reference_deg - current_yaw);
        const double heading_error_rad =
            heading_error_deg * static_cast<double>(PI) / 180.0;
        msg.angular.z = std::abs(heading_error_deg) <=
                y_hard_heading_deadband_deg
            ? 0.0
            : std::max(-y_hard_heading_max_wz,
                       std::min(y_hard_heading_max_wz,
                                y_hard_heading_kp * heading_error_rad));
        pub.publish(msg);
        publishDebugImage();
        ROS_WARN_THROTTLE(
            0.5,
            "[Y_COORD] Remaining hard forward | next_path=%s | trigger=%s | overall_target=%.3fm | guided=%.3fm | hard_target=%.3fm | hard_moved=%.3fm | remaining=%.3fm | ref_yaw=%.2fdeg | yaw=%.2fdeg | yaw_error=%.2fdeg | odom_fresh=%d(age=%.3fs) | imu_fresh=%d(age=%.3fs) | v=%.2f | wz=%.3f",
            pathToString(pending_branch_path).c_str(),
            y_hard_drive_trigger_reason.c_str(),
            y_guided_target_dist, y_guided_moved_dist,
            y_hard_drive_target_dist, moved,
            y_hard_drive_remaining_dist,
            y_hard_heading_reference_deg, current_yaw, heading_error_deg,
            odom_fresh, odom_age, imu_fresh, imu_age,
            msg.linear.x, msg.angular.z);
        return true;
    }

    if (motion_state == MotionState::Y_ALIGNING_LEFT ||
        motion_state == MotionState::Y_ALIGNING_RIGHT) {
        const ros::Time now = ros::Time::now();
        double dt = y_turn_has_last_time ? (now - y_turn_last_time).toSec() : 0.0;
        y_turn_last_time = now;
        y_turn_has_last_time = true;
        if (dt > 0.0 && dt < 0.2) {
            y_turn_integrated_angle_deg +=
                std::abs(curent_wz) * dt * 180.0 / static_cast<double>(PI);
        }

        if (y_turn_integrated_angle_deg >= y_turn_angle_deg) {
            publishStop();
            motion_state = MotionState::Y_ALIGN_PAUSE;
            y_turn_pause_start = ros::Time::now();
            publishStatus("Y_TURN_PAUSE_" + pathToString(pending_branch_path));
            ROS_WARN(
                "[Y_TURN] Finished | next_path=%s | integrated_angle=%.2fdeg/%.2fdeg | wz=%.3f",
                pathToString(pending_branch_path).c_str(),
                y_turn_integrated_angle_deg, y_turn_angle_deg, curent_wz);
            return true;
        }

        geometry_msgs::Twist msg;
        msg.angular.z = motion_state == MotionState::Y_ALIGNING_LEFT
            ? std::abs(y_turn_angular_speed)
            : -std::abs(y_turn_angular_speed);
        pub.publish(msg);
        publishDebugImage();
        ROS_WARN_THROTTLE(
            0.5,
            "[Y_TURN] Turning | next_path=%s | integrated_angle=%.2fdeg/%.2fdeg | wz=%.3f | dt=%.3fs | direction=%s",
            pathToString(pending_branch_path).c_str(),
            y_turn_integrated_angle_deg, y_turn_angle_deg,
            curent_wz, dt,
            motion_state == MotionState::Y_ALIGNING_LEFT ? "LEFT" : "RIGHT");
        return true;
    }

    if (motion_state == MotionState::Y_ALIGN_PAUSE) {
        geometry_msgs::Twist stop_msg;
        pub.publish(stop_msg);
        publishDebugImage();
        const double elapsed = (ros::Time::now() - y_turn_pause_start).toSec();
        if (elapsed >= y_turn_pause_sec) {
            const PathSelect completed_branch = pending_branch_path;
            path_select = completed_branch;
            track_type = completed_branch == PathSelect::LEFT ? TRACK_LEFT : TRACK_RIGHT;
            applyPathBiasParams(completed_branch);
            resetMotionController();
            motion_state = MotionState::Y_BRANCH_REACQUIRE;
            y_reacquire_start_x = current_odom_position_x;
            y_reacquire_start_y = current_odom_position_y;
            y_reacquire_moved_dist = 0.0;
            y_reacquire_remaining_dist = y_reacquire_max_dist;
            y_reacquire_start_time = ros::Time::now();
            y_reacquire_confirm_count = 0;
            y_reacquire_visible_points = 0;
            y_reacquire_hold_reason.clear();
            resetParkingCornerState();
            publishStatus("Y_REACQUIRE_" + pathToString(completed_branch));
            ROS_WARN(
                "[Y_REACQUIRE] Started | path=%s | pause=%.2fs | speed=%.2fm/s | max_dist=%.3fm | max_duration=%.2fs | min_points=%d | confirm=%d | bias_left=%.1f | bias_right=%.1f | Time_local=%.2f",
                pathToString(completed_branch).c_str(), elapsed,
                y_reacquire_speed, y_reacquire_max_dist,
                y_reacquire_max_duration, y_reacquire_min_points,
                y_reacquire_confirm_frames,
                Dis_Bias_Left, Dis_Bias_Right, Time_local);
        }
        return true;
    }

    if (motion_state == MotionState::Y_BRANCH_REACQUIRE) {
        const ros::Time now = ros::Time::now();
        const double dx = current_odom_position_x - y_reacquire_start_x;
        const double dy = current_odom_position_y - y_reacquire_start_y;
        y_reacquire_moved_dist = std::hypot(dx, dy);
        y_reacquire_remaining_dist = std::max(
            0.0, y_reacquire_max_dist - y_reacquire_moved_dist);
        const double odom_age = last_odom_time.isZero()
            ? std::numeric_limits<double>::infinity()
            : (now - last_odom_time).toSec();
        const bool odom_fresh = odom_age >= 0.0 &&
            odom_age <= y_reacquire_odom_timeout;
        const double elapsed = y_reacquire_start_time.isZero()
            ? 0.0 : (now - y_reacquire_start_time).toSec();

        bool line_valid = false;
        std::string vision_reason = "gray_points";
        if (vision_source == "line_follower") {
            line_follower::LineTrack track;
            double receive_age = -1.0;
            double source_age = -1.0;
            line_valid = externalTrackSnapshot(
                track, receive_age, source_age, vision_reason);
            y_reacquire_visible_points = static_cast<int>(
                track.center_x_px.size());
            line_valid = line_valid &&
                y_reacquire_visible_points >= y_reacquire_min_points;
        } else {
            y_reacquire_visible_points = rpts_num;
            line_valid = rpts_num >= y_reacquire_min_points;
        }
        if (!odom_fresh) {
            line_valid = false;
            vision_reason = "odom_stale";
        }

        y_reacquire_confirm_count = line_valid
            ? y_reacquire_confirm_count + 1 : 0;
        if (y_reacquire_confirm_count >= y_reacquire_confirm_frames) {
            const PathSelect recovered_path = path_select;
            const double recovered_moved = y_reacquire_moved_dist;
            const int recovered_points = y_reacquire_visible_points;
            resetYBranchState();
            motion_state = MotionState::FOLLOWING;
            pid.reset();
            publishStatus("RUNNING_" + pathToString(recovered_path));
            ROS_WARN(
                "[Y_REACQUIRE] Recovered, branch follow takes control | path=%s | points=%d | moved=%.3fm | elapsed=%.2fs",
                pathToString(recovered_path).c_str(), recovered_points,
                recovered_moved, elapsed);
            return false;
        }

        std::string hold_reason;
        if (!odom_fresh) {
            hold_reason = "odom_stale";
        } else if (y_reacquire_moved_dist >= y_reacquire_max_dist) {
            hold_reason = "max_distance";
        } else if (elapsed >= y_reacquire_max_duration) {
            hold_reason = "timeout";
        }

        geometry_msgs::Twist msg;
        if (hold_reason.empty()) {
            msg.linear.x = y_reacquire_speed;
        }
        msg.angular.z = 0.0;
        pub.publish(msg);
        publishDebugImage();

        if (hold_reason != y_reacquire_hold_reason) {
            y_reacquire_hold_reason = hold_reason;
            if (!hold_reason.empty()) {
                publishStatus("Y_REACQUIRE_HOLD_" + hold_reason);
                ROS_WARN(
                    "[Y_REACQUIRE] Holding stop | path=%s | reason=%s | moved=%.3fm/%.3fm | elapsed=%.2fs/%.2fs | odom_age=%.3fs",
                    pathToString(path_select).c_str(), hold_reason.c_str(),
                    y_reacquire_moved_dist, y_reacquire_max_dist,
                    elapsed, y_reacquire_max_duration, odom_age);
            } else {
                publishStatus("Y_REACQUIRE_" + pathToString(path_select));
                ROS_WARN("[Y_REACQUIRE] Safety hold cleared; resuming low-speed forward");
            }
        }

        ROS_WARN_THROTTLE(
            0.5,
            "[Y_REACQUIRE] Searching | path=%s | vision=%s | points=%d/%d | valid=%d | confirm=%d/%d | moved=%.3fm | remaining=%.3fm | elapsed=%.2fs/%.2fs | odom_fresh=%d | odom_age=%.3fs | cmd_v=%.2f | cmd_wz=0.00 | hold=%s | vision_reason=%s",
            pathToString(path_select).c_str(), vision_source.c_str(),
            y_reacquire_visible_points, y_reacquire_min_points,
            line_valid, y_reacquire_confirm_count,
            y_reacquire_confirm_frames, y_reacquire_moved_dist,
            y_reacquire_remaining_dist, elapsed,
            y_reacquire_max_duration, odom_fresh, odom_age,
            msg.linear.x,
            y_reacquire_hold_reason.empty() ? "none" : y_reacquire_hold_reason.c_str(),
            vision_reason.c_str());
        return true;
    }

    return false;
}

bool handleParkingCorner() {
    // 停车逻辑保留原工程的“近距离 L 角点停车”能力。
    // 判断到靠近图像底部的 L 型角点后，先计算角点相对车体中心的距离，
    // 再用低速 S 弯（或兼容的 lateral 模式）把车挪到停车位置，最后发布 STOP。
    if (!parking_enabled) {
        return false;
    }

    float corner_dot[2] = {0.0f, 0.0f};
    bool is_stop_corner = false;
    const char *parking_line_type = "None";
    int parking_corner_id = -1;
    float parking_shape_forward = 0.0f;
    float parking_shape_lateral = 0.0f;
    float parking_corner_y = 0.0f;

    if ((path_select == PathSelect::RIGHT ||
         (parking_allow_either_l && path_select == PathSelect::LEFT && !Lpt0_found)) &&
        Lpt1_found && Lpt1_rpts1s_id >= 3 && Lpt1_rpts1s_id < rptsc1_num) {
        // 右侧线 L 角点：角点前后点在图像中形成“向左折”的趋势，
        // 且角点位于图像下方，说明停车点已经接近车体。
        int im1 = clip(Lpt1_rpts1s_id - (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        int ip1 = clip(Lpt1_rpts1s_id + (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        is_stop_corner = (rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1] > 20) &&
                         (rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0] < -20) &&
                         (rptsc1[Lpt1_rpts1s_id][1] > RESULT_ROW - 40);
        if (is_stop_corner) {
            parking_shape_forward = rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1];
            parking_shape_lateral = rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0];
            parking_corner_y = rptsc1[Lpt1_rpts1s_id][1];
            corner_move(rpts1s, corner_dot, Lpt1_rpts1s_id, -pixel_per_meter * ROAD_WIDTH / 2);
            parking_line_type = "Right_L";
            parking_corner_id = Lpt1_rpts1s_id;
        }
    } else if ((path_select == PathSelect::LEFT ||
                (parking_allow_either_l && path_select == PathSelect::RIGHT && !Lpt1_found)) &&
               Lpt0_found && Lpt0_rpts0s_id >= 3 && Lpt0_rpts0s_id < rptsc0_num) {
        // 左侧线 L 角点：判断条件和右侧线对称，横向方向相反。
        int im0 = clip(Lpt0_rpts0s_id - (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        int ip0 = clip(Lpt0_rpts0s_id + (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        is_stop_corner = (rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1] > 20) &&
                         (rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0] > 20) &&
                         (rptsc0[Lpt0_rpts0s_id][1] > RESULT_ROW - 40);
        if (is_stop_corner) {
            parking_shape_forward = rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1];
            parking_shape_lateral = rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0];
            parking_corner_y = rptsc0[Lpt0_rpts0s_id][1];
            corner_move(rpts0s, corner_dot, Lpt0_rpts0s_id, pixel_per_meter * ROAD_WIDTH / 2);
            parking_line_type = "Left_L";
            parking_corner_id = Lpt0_rpts0s_id;
        }
    }

    if (!is_stop_corner && parking_allow_either_l && path_select == PathSelect::LEFT &&
        Lpt1_found && Lpt1_rpts1s_id >= 3 && Lpt1_rpts1s_id < rptsc1_num) {
        int im1 = clip(Lpt1_rpts1s_id - (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        int ip1 = clip(Lpt1_rpts1s_id + (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
        is_stop_corner = (rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1] > 20) &&
                         (rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0] < -20) &&
                         (rptsc1[Lpt1_rpts1s_id][1] > RESULT_ROW - 40);
        if (is_stop_corner) {
            parking_shape_forward = rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1];
            parking_shape_lateral = rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0];
            parking_corner_y = rptsc1[Lpt1_rpts1s_id][1];
            corner_move(rpts1s, corner_dot, Lpt1_rpts1s_id, -pixel_per_meter * ROAD_WIDTH / 2);
            parking_line_type = "Right_L";
            parking_corner_id = Lpt1_rpts1s_id;
        }
    }

    if (!is_stop_corner && parking_allow_either_l && path_select == PathSelect::RIGHT &&
        Lpt0_found && Lpt0_rpts0s_id >= 3 && Lpt0_rpts0s_id < rptsc0_num) {
        int im0 = clip(Lpt0_rpts0s_id - (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        int ip0 = clip(Lpt0_rpts0s_id + (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
        is_stop_corner = (rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1] > 20) &&
                         (rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0] > 20) &&
                         (rptsc0[Lpt0_rpts0s_id][1] > RESULT_ROW - 40);
        if (is_stop_corner) {
            parking_shape_forward = rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1];
            parking_shape_lateral = rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0];
            parking_corner_y = rptsc0[Lpt0_rpts0s_id][1];
            corner_move(rpts0s, corner_dot, Lpt0_rpts0s_id, pixel_per_meter * ROAD_WIDTH / 2);
            parking_line_type = "Left_L";
            parking_corner_id = Lpt0_rpts0s_id;
        }
    }

    if (!is_stop_corner) {
        // 周期性打印角点检测状态（即使未检测到停车点）
        ROS_WARN_THROTTLE(2.0, "[PARKING] CornerDetect | path=%s | L0=%d(id=%d) | L1=%d(id=%d) | Y0=%d(id=%d) | Y1=%d(id=%d) | left_pts=%d | right_pts=%d | parking_enable=%d | allow_either_l=%d",
                  pathToString(path_select).c_str(),
                  Lpt0_found, Lpt0_rpts0s_id, Lpt1_found, Lpt1_rpts1s_id,
                  Ypt0_found, Ypt0_rpts0s_id, Ypt1_found, Ypt1_rpts1s_id,
                  rptsc0_num, rptsc1_num, parking_enabled, parking_allow_either_l);
        return false;
    }

    ROS_WARN("[PARKING] CornerDetect accepted | path=%s | L0=%d(id=%d) | L1=%d(id=%d) | "
             "left_pts=%d | right_pts=%d | line_type=%s | corner_id=%d | shape=(dy=%.1f,dx=%.1f,y=%.1f) | allow_either_l=%d",
             pathToString(path_select).c_str(),
             Lpt0_found, Lpt0_rpts0s_id,
             Lpt1_found, Lpt1_rpts1s_id,
             rptsc0_num, rptsc1_num,
             parking_line_type, parking_corner_id,
             parking_shape_forward, parking_shape_lateral, parking_corner_y,
             parking_allow_either_l);

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

    int parking_loop_count = 0;  // 停车循环计数器
    ros::Time parking_start_time = ros::Time::now();  // 停车开始时间
    const double parking_total_dist =
        std::max(0.001, std::abs(static_cast<double>(target_dis)) + parking_extra_dist);
    const double parking_target_lateral = parking_lateral_cmd_sign * target_dis_x;
    const bool use_s_curve = parking_motion_mode == "s_curve";
    const ParkingSCurvePlan s_curve_plan = makeParkingSCurvePlan(
        parking_total_dist, parking_target_lateral, parking_forward_speed,
        parking_lateral_deadband, parking_max_angular_speed);
    const double parking_path_length = use_s_curve
        ? s_curve_plan.total_length
        : parking_total_dist;
    const double parking_cmd_speed = use_s_curve
        ? s_curve_plan.forward_speed
        : parking_forward_speed;
    const double parking_start_yaw = current_yaw * PI / 180.0;
    const double yaw_tolerance_rad = parking_yaw_tolerance_deg * PI / 180.0;
    double parking_moved = 0.0;
    double legacy_lateral_remaining = parking_target_lateral;
    double last_print_moved = -1.0;
    ros::Rate parking_rate(30.0);

    ROS_WARN("[PARKING_PLAN] mode=%s | target=(%.3f,%.3f)m | curved=%d | radius=%.3fm | "
             "peak_yaw=%.1fdeg | path=%.3fm | cmd_v=%.3fm/s | ff_wz=%.3frad/s | "
             "wz_limit=(first=%.3f,second=%.3f)rad/s | start_yaw=%.1fdeg",
             parking_motion_mode.c_str(), parking_total_dist, parking_target_lateral,
             s_curve_plan.curved, s_curve_plan.radius,
             s_curve_plan.peak_yaw * 180.0 / PI, parking_path_length,
             parking_cmd_speed, s_curve_plan.feedforward_wz,
             parking_max_angular_speed,
             parking_second_arc_max_angular_speed, current_yaw);

    const auto abortParking = [&](const std::string &status, const char *reason) {
        publishStop();
        run_car = false;
        resetParkingCornerState();
        publishStatus(status);
        ROS_ERROR("[PARKING_ABORT] %s | moved=%.3fm/%.3fm | elapsed=%.2fs",
                  reason, parking_moved, parking_path_length,
                  (ros::Time::now() - parking_start_time).toSec());
    };

    while (ros::ok()) {
        ros::spinOnce();
        if (!run_car) {
            // beginCallback 已经为 Stop 指令发布零速度并切换为 IDLE。
            return true;
        }

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

        const double elapsed_sec = (now - parking_start_time).toSec();
        if (elapsed_sec > parking_timeout) {
            abortParking("PARKING_ABORTED_TIMEOUT", "parking timeout");
            return true;
        }
        if (last_odom_time.isZero() ||
            (now - last_odom_time).toSec() > parking_odom_timeout) {
            abortParking("PARKING_ABORTED_ODOM", "odometry missing or stale");
            return true;
        }

        parking_moved += std::abs(current_linear_velocity_x) * dt;
        const double traveled = std::min(parking_moved, parking_path_length);
        const double relative_yaw = normalizeAngle(current_yaw * PI / 180.0 - parking_start_yaw);
        const double reference_yaw = use_s_curve
            ? parkingSCurveReferenceYaw(s_curve_plan, traveled)
            : 0.0;
        const double yaw_error = normalizeAngle(reference_yaw - relative_yaw);
        const bool path_complete = parking_moved >= parking_path_length;

        local_msg.linear.x = path_complete ? 0.0 : parking_cmd_speed;
        local_msg.linear.y = 0.0;
        local_msg.angular.z = 0.0;

        if (use_s_curve) {
            const double feedforward_wz = parkingSCurveFeedforwardWz(s_curve_plan, traveled);
            const double requested_wz = feedforward_wz + parking_yaw_kp * yaw_error;
            const bool second_arc =
                traveled >= s_curve_plan.total_length * 0.5;
            const double active_max_wz = second_arc
                ? parking_second_arc_max_angular_speed
                : parking_max_angular_speed;
            local_msg.angular.z = std::max(-active_max_wz,
                                           std::min(active_max_wz, requested_wz));
        } else if (!path_complete &&
                   std::abs(legacy_lateral_remaining) >= parking_lateral_deadband) {
            local_msg.linear.y = legacy_lateral_remaining > 0.0
                ? parking_lateral_speed
                : -parking_lateral_speed;
            legacy_lateral_remaining -= local_msg.linear.y * dt;
        }

        if (elapsed_sec > 1.0f && parking_moved < 0.02f) {
            ROS_WARN_THROTTLE(1.0, "[PARKING_STUCK] cmd_vel is being published but moved=%.3fm/%.3fm | odom_vx=%.3fm/s",
                              parking_moved, parking_path_length, current_linear_velocity_x);
        }

        if (parking_moved - last_print_moved >= 0.02 ||
            (path_complete && parking_loop_count % 15 == 0)) {
            last_print_moved = parking_moved;
            const double remaining = std::max(0.0, parking_path_length - parking_moved);
            const double progress = 100.0 * traveled / std::max(0.001, parking_path_length);
            ROS_WARN("[PARKING_PROGRESS] progress=%.0f%% | remaining=%.3fm/%.3fm | "
                     "arc=%s | yaw=(ref=%.1f,actual=%.1f,error=%.1f)deg | "
                     "cmd=(%.2f,%.2f,%.2f) | "
                     "odom_age=%.3fs | loops=%d | elapsed=%.2fs",
                     std::max(0.0, std::min(progress, 100.0)),
                     remaining, parking_path_length,
                     !use_s_curve ? "lateral"
                         : (traveled >= s_curve_plan.total_length * 0.5
                                ? "second" : "first"),
                     reference_yaw * 180.0 / PI,
                     relative_yaw * 180.0 / PI,
                     yaw_error * 180.0 / PI,
                     local_msg.linear.x, local_msg.linear.y, local_msg.angular.z,
                     (now - last_odom_time).toSec(), parking_loop_count, elapsed_sec);
        }

        const bool heading_aligned = std::abs(yaw_error) <= yaw_tolerance_rad;
        if (path_complete && (!use_s_curve || heading_aligned)) {
            // 到达停车距离后，先发零速度，再向 end_topic 发布 STOP，
            // 这样外部上层逻辑可以知道本段巡线已经结束。
            ROS_WARN("[PARKING] Parking finished! | remaining=%.3fm | target_lat=%.3fm | "
                     "yaw_error=%.2fdeg | moved=%.3fm/%.3fm | total_loops=%d | total_time=%.2fs | line_type=%s",
                     std::max(0.0, parking_path_length - parking_moved),
                     parking_target_lateral,
                     yaw_error * 180.0 / PI,
                     parking_moved,
                     parking_path_length,
                     parking_loop_count,
                     elapsed_sec,
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

bool handleLostCornerSearch() {
    const bool has_control_path = rpts_num > 0;
    const bool has_l_corner = Lpt0_found || Lpt1_found;
    const bool can_search_path = path_select == PathSelect::LEFT || path_select == PathSelect::RIGHT;

    if (has_control_path || has_l_corner || !can_search_path) {
        if (lost_corner_search_active || lost_corner_search_timed_out) {
            ROS_WARN("[LOST_CORNER_SEARCH] Recovered | path=%s | rpts=%d | L0=%d | L1=%d",
                     pathToString(path_select).c_str(), rpts_num, Lpt0_found, Lpt1_found);
        }
        resetLostCornerSearchState();
        return false;
    }

    if (!lost_corner_search_enabled) {
        resetLostCornerSearchState();
        return false;
    }

    geometry_msgs::Twist msg;
    if (lost_corner_search_timed_out) {
        pub.publish(msg);
        publishDebugImage();
        ROS_WARN_THROTTLE(0.5,
                          "[LOST_CORNER_SEARCH] Timed out, holding stop | path=%s | timeout=%.2fs | L0=%d | L1=%d",
                          pathToString(path_select).c_str(),
                          lost_corner_search_timeout,
                          Lpt0_found,
                          Lpt1_found);
        return true;
    }

    const ros::Time now = ros::Time::now();
    if (!lost_corner_search_active) {
        lost_corner_search_active = true;
        lost_corner_search_start_time = now;
        publishStatus("LOST_CORNER_SEARCH_" + pathToString(path_select));
        ROS_WARN("[LOST_CORNER_SEARCH] Started | path=%s | dir=%s | timeout=%.2fs | wz=%.2f | vx=%.2f",
                 pathToString(path_select).c_str(),
                 path_select == PathSelect::LEFT ? "right" : "left",
                 lost_corner_search_timeout,
                 lost_corner_search_angular_speed,
                 lost_corner_search_linear_speed);
    }

    const double elapsed = (now - lost_corner_search_start_time).toSec();
    if (elapsed >= lost_corner_search_timeout) {
        lost_corner_search_active = false;
        lost_corner_search_timed_out = true;
        pub.publish(msg);
        publishDebugImage();
        ROS_WARN("[LOST_CORNER_SEARCH] Timeout stop | path=%s | elapsed=%.2fs/%.2fs | L0=%d | L1=%d",
                 pathToString(path_select).c_str(),
                 elapsed,
                 lost_corner_search_timeout,
                 Lpt0_found,
                 Lpt1_found);
        return true;
    }

    msg.linear.x = lost_corner_search_linear_speed;
    msg.angular.z = path_select == PathSelect::LEFT
        ? -std::abs(lost_corner_search_angular_speed)
        : std::abs(lost_corner_search_angular_speed);
    pub.publish(msg);
    publishDebugImage();
    ROS_WARN_THROTTLE(0.25,
                      "[LOST_CORNER_SEARCH] Searching | path=%s | elapsed=%.2fs/%.2fs | cmd_v=%.2f | cmd_wz=%.2f | L0=%d | L1=%d",
                      pathToString(path_select).c_str(),
                      elapsed,
                      lost_corner_search_timeout,
                      msg.linear.x,
                      msg.angular.z,
                      Lpt0_found,
                      Lpt1_found);
    return true;
}

void publishDebugImage(const sensor_msgs::ImageConstPtr &source_msg) {
    // 恢复历史 MONO8 调试图：ImageUsed 为底图，不再转换为 BGR 彩色图。
    // 灰度值约定：0=左目标线，80=右目标线，160=当前控制路径。
    for (int i = 0; i < RESULT_ROW; ++i) {
        for (int j = 0; j < RESULT_COL; ++j) {
            img_line_data[i][j] = ImageUsed[i][j];
        }
    }
    for (int i = 0; i < rptsc0e_num; ++i) {
        AT_IMAGE(&img_line,
                 clip(static_cast<int>(rptsc0e[i][0]), 0, img_line.width - 1),
                 clip(static_cast<int>(rptsc0e[i][1]), 0, img_line.height - 1)) = 0;
    }
    for (int i = 0; i < rptsc1e_num; ++i) {
        AT_IMAGE(&img_line,
                 clip(static_cast<int>(rptsc1e[i][0]), 0, img_line.width - 1),
                 clip(static_cast<int>(rptsc1e[i][1]), 0, img_line.height - 1)) = 80;
    }
    for (int i = 0; i < rpts_num; ++i) {
        AT_IMAGE(&img_line,
                 clip(static_cast<int>(rpts[i][0]), 0, img_line.width - 1),
                 clip(static_cast<int>(rpts[i][1]), 0, img_line.height - 1)) = 160;
    }

    cv::Mat debug_gray = convert2DArrayToMat(img_line_data);

    auto drawPointLabel = [&](float pts[][2], int pts_num, int idx,
                              const std::string &label, uint8_t gray,
                              bool cross_marker) {
        if (idx < 0 || idx >= pts_num) {
            return;
        }
        const int x = clip(static_cast<int>(std::round(pts[idx][0])),
                           0, RESULT_COL - 1);
        const int y = clip(static_cast<int>(std::round(pts[idx][1])),
                           0, RESULT_ROW - 1);
        const cv::Point point(x, y);
        const cv::Scalar color(gray);

        if (cross_marker) {
            cv::line(debug_gray, cv::Point(std::max(0, x - 6), y),
                     cv::Point(std::min(RESULT_COL - 1, x + 6), y), color, 2);
            cv::line(debug_gray, cv::Point(x, std::max(0, y - 6)),
                     cv::Point(x, std::min(RESULT_ROW - 1, y + 6)), color, 2);
        } else {
            cv::circle(debug_gray, point, 6, color, 2);
        }
        cv::putText(debug_gray, label,
                    cv::Point(std::min(RESULT_COL - 1, x + 8),
                              std::max(12, y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1);
    };

    if (Lpt0_found) {
        drawPointLabel(rpts0s, rpts0s_num, Lpt0_rpts0s_id,
                       "L0", 255, false);
    }
    if (Lpt1_found) {
        drawPointLabel(rpts1s, rpts1s_num, Lpt1_rpts1s_id,
                       "L1", 220, false);
    }
    if (Ypt0_found) {
        drawPointLabel(rpts0s, rpts0s_num, Ypt0_rpts0s_id,
                       "Y0", 200, true);
    }
    if (Ypt1_found) {
        drawPointLabel(rpts1s, rpts1s_num, Ypt1_rpts1s_id,
                       "Y1", 180, true);
    }

    if (motion_state == MotionState::Y_CROSSBAR_SEEK &&
        forward_crossbar_result.found) {
        const int crossbar_x = clip(
            static_cast<int>(std::round(forward_crossbar_result.map_x)),
            0, RESULT_COL - 1);
        const int crossbar_y = clip(
            static_cast<int>(std::round(forward_crossbar_result.map_y)),
            0, RESULT_ROW - 1);
        cv::drawMarker(debug_gray, cv::Point(crossbar_x, crossbar_y),
                       cv::Scalar(240), cv::MARKER_CROSS, 18, 2);
        cv::putText(debug_gray, "Y_BAR",
                    cv::Point(std::min(RESULT_COL - 90, crossbar_x + 8),
                              std::max(18, crossbar_y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(240), 1, cv::LINE_AA);
    }

    std::ostringstream state_line;
    state_line << "path=" << pathToString(path_select)
               << " state=" << motionStateToString(motion_state)
               << " vision=" << vision_source;
    if (external_vision_lost) {
        state_line << " EXTERNAL_VISION_LOST";
    }
    if (y_branch_mode_requested) {
        state_line << " next=" << pathToString(pending_branch_path);
    }
    cv::putText(debug_gray, state_line.str(), cv::Point(8, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0), 3, cv::LINE_AA);
    cv::putText(debug_gray, state_line.str(), cv::Point(8, 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(240), 1, cv::LINE_AA);

    if (motion_state == MotionState::Y_GUIDED_FORWARD) {
        std::ostringstream guided_line;
        guided_line << std::fixed << std::setprecision(2)
                    << "Y_GUIDED target=" << y_guided_target_dist
                    << " moved=" << y_guided_moved_dist
                    << " remain=" << y_guided_remaining_dist
                    << " err=" << y_guided_visual_error
                    << " lost=" << y_guided_lost_count
                    << "/" << y_guided_lost_confirm_frames
                    << " angle=" << y_guided_error_count
                    << "/" << y_guided_error_confirm_frames;
        if (!y_guided_hold_reason.empty()) {
            guided_line << " HOLD=" << y_guided_hold_reason;
        }
        cv::putText(debug_gray, guided_line.str(), cv::Point(8, 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0), 3,
                    cv::LINE_AA);
        cv::putText(debug_gray, guided_line.str(), cv::Point(8, 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(240), 1,
                    cv::LINE_AA);
    }

    if (motion_state == MotionState::Y_COORDINATE_FORWARD) {
        const int target_x = RESULT_COL / 2;
        const int target_y = static_cast<int>(std::round(
            RESULT_ROW + 10.0 -
            y_hard_drive_remaining_dist * pixel_per_meter));
        if (target_y >= 0 && target_y < RESULT_ROW) {
            cv::drawMarker(debug_gray, cv::Point(target_x, target_y),
                           cv::Scalar(250), cv::MARKER_TILTED_CROSS, 20, 2);
            cv::putText(debug_gray, "Y_TARGET",
                        cv::Point(std::min(RESULT_COL - 90, target_x + 10),
                                  std::max(18, target_y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.38,
                        cv::Scalar(250), 1, cv::LINE_AA);
        }
        std::ostringstream y_coord_line;
        y_coord_line << std::fixed << std::setprecision(2)
                     << "Y_REMAIN reason=" << y_hard_drive_trigger_reason
                     << " all=" << y_guided_target_dist
                     << " guided=" << y_guided_moved_dist
                     << " hard=" << y_hard_drive_target_dist
                     << " remain=" << y_hard_drive_remaining_dist;
        if (y_hard_drive_aborted) {
            y_coord_line << " ABORT=" << y_hard_drive_abort_reason;
        }
        cv::putText(debug_gray, y_coord_line.str(), cv::Point(8, 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0), 3,
                    cv::LINE_AA);
        cv::putText(debug_gray, y_coord_line.str(), cv::Point(8, 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(240), 1,
                    cv::LINE_AA);
    }

    if (motion_state == MotionState::Y_BRANCH_REACQUIRE) {
        std::ostringstream reacquire_line;
        reacquire_line << std::fixed << std::setprecision(2)
                       << "Y_REACQUIRE pts=" << y_reacquire_visible_points
                       << "/" << y_reacquire_min_points
                       << " confirm=" << y_reacquire_confirm_count
                       << "/" << y_reacquire_confirm_frames
                       << " moved=" << y_reacquire_moved_dist
                       << " remain=" << y_reacquire_remaining_dist;
        if (!y_reacquire_hold_reason.empty()) {
            reacquire_line << " HOLD=" << y_reacquire_hold_reason;
        }
        cv::putText(debug_gray, reacquire_line.str(), cv::Point(8, 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(0), 3,
                    cv::LINE_AA);
        cv::putText(debug_gray, reacquire_line.str(), cv::Point(8, 38),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(240), 1,
                    cv::LINE_AA);
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
                false
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
            if (vision_source == "line_follower") {
                geometry_msgs::Twist stop_msg;
                pub.publish(stop_msg);
                right_turn_odom_underresponse_count = 0;
                if (!external_vision_lost) {
                    external_vision_lost = true;
                    publishStatus("EXTERNAL_VISION_LOST");
                }
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[VISION] EXTERNAL_VISION_LOST reason=no_gray_auxiliary_frame; command zero");
            }
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

    const bool external_control = vision_source == "line_follower" &&
                                  motion_state == MotionState::FOLLOWING &&
                                  !y_branch_mode_requested;
    if (!external_control && handleLostCornerSearch()) {
        return 0;
    }

    float error = 0.0f;
    float v = 0.0f;
    double effective_aim_distance = aim_distance;
    double right_assist_strength = 0.0;
    double right_wz_compensation = 0.0;
    double odom_age = -1.0;
    bool odom_fresh = false;
    bool right_wz_compensation_active = false;
    if (external_control) {
        line_follower::LineTrack track;
        double receive_age = -1.0;
        double source_age = -1.0;
        std::string invalid_reason;
        if (!externalTrackSnapshot(track, receive_age, source_age, invalid_reason)) {
            geometry_msgs::Twist stop_msg;
            pub.publish(stop_msg);
            right_turn_odom_underresponse_count = 0;
            if (!external_vision_lost) {
                external_vision_lost = true;
                publishStatus("EXTERNAL_VISION_LOST");
            }
            ROS_ERROR_THROTTLE(
                1.0,
                "[VISION] EXTERNAL_VISION_LOST reason=%s receive_age=%.3fs source_age=%.3fs; command zero, no gray fallback",
                invalid_reason.c_str(), receive_age, source_age);
            publishDebugImage();
            return 0;
        }

        if (external_vision_lost) {
            external_vision_lost = false;
            publishStatus("RUNNING_" + pathToString(path_select));
            ROS_WARN("[VISION] External line track recovered");
        }

        size_t nominal_index = 0;
        double best_distance_sq = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < track.center_x_px.size(); ++i) {
            const double dx = track.center_x_px[i] - track.lookahead_x_px;
            const double dy = track.center_y_px[i] - track.lookahead_y_px;
            const double distance_sq = dx * dx + dy * dy;
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                nominal_index = i;
            }
        }

        const auto calculate_external_error = [&](size_t index) {
            const double cx = track.image_width * 0.5;
            const double dx = track.center_x_px[index] - cx;
            const double dy = std::max(1.0,
                static_cast<double>(track.image_height) - track.center_y_px[index]);
            return static_cast<float>(-std::atan2(dx, dy));
        };

        const float nominal_error = calculate_external_error(nominal_index);
        if (right_turn_assist_enabled && path_select == PathSelect::RIGHT) {
            right_assist_strength = std::max(
                0.0, std::min(1.0,
                              (std::abs(static_cast<double>(nominal_error)) -
                               right_turn_error_start) /
                                  (right_turn_error_full - right_turn_error_start)));
            const double min_aim = std::min(aim_distance, right_turn_min_aim_distance);
            effective_aim_distance =
                aim_distance + right_assist_strength * (min_aim - aim_distance);
        }

        const double aim_ratio = aim_distance > 1e-6
                                     ? effective_aim_distance / aim_distance
                                     : 1.0;
        const size_t effective_index = std::min(
            track.center_x_px.size() - 1,
            static_cast<size_t>(std::lround(nominal_index * aim_ratio)));
        error = calculate_external_error(effective_index);
        v = static_cast<float>(base_speed - std::abs(error) * base_speed);
        v = std::max(0.05f, v);

        ROS_WARN_THROTTLE(
            1.0,
            "[VISION] source=line_follower valid=1 confidence=%.2f points=%zu nominal_idx=%zu effective_idx=%zu heading_msg=%.3f heading_used=%.3f",
            track.confidence, track.center_x_px.size(), nominal_index,
            effective_index, track.heading_error_rad, error);
    } else if (rpts_num == 0) {
        // 连续丢线时逐步停车；偶发一帧丢线时仍低速前进，减少图像抖动影响。
        right_turn_odom_underresponse_count = 0;
        zeroCount++;
        if (zeroCount >= 2) {
            zero_flag = true;
        }
        error = 0.0f;
        v = zero_flag ? 0.0f : 0.15f;
    } else {
        zeroCount = 0;
        zero_flag = false;
        // 保持原控制几何，仅在普通 Right 模式的大偏差区间动态缩短前视距离。
        double nominal_error_value = 0.0;
        calculateGrayControlError(aim_distance, nominal_error_value);
        const float nominal_error = static_cast<float>(nominal_error_value);
        if (right_turn_assist_enabled && path_select == PathSelect::RIGHT) {
            right_assist_strength = std::max(
                0.0, std::min(1.0,
                              (std::abs(static_cast<double>(nominal_error)) -
                               right_turn_error_start) /
                                  (right_turn_error_full - right_turn_error_start)));
            const double min_aim = std::min(aim_distance, right_turn_min_aim_distance);
            effective_aim_distance =
                aim_distance + right_assist_strength * (min_aim - aim_distance);
        }

        double effective_error_value = 0.0;
        calculateGrayControlError(effective_aim_distance,
                                  effective_error_value);
        error = static_cast<float>(effective_error_value);
        v = static_cast<float>(base_speed - std::abs(error) * base_speed);
        v = std::max(0.05f, v);

    }

    if (right_turn_assist_enabled && path_select == PathSelect::RIGHT) {
        const double speed_limit =
            base_speed + right_assist_strength * (right_turn_min_speed - base_speed);
        v = std::min(v, static_cast<float>(speed_limit));

        const ros::Time now = ros::Time::now();
        if (!last_odom_time.isZero()) {
            odom_age = (now - last_odom_time).toSec();
            odom_fresh = odom_age >= 0.0 && odom_age <= right_turn_odom_timeout;
        }

        const double turn_sign = error >= 0.0f ? 1.0 : -1.0;
        const double directional_odom_wz = turn_sign * current_odom_angular_velocity_z;
        const bool odom_underresponding =
            right_assist_strength > 0.0 && odom_fresh &&
            directional_odom_wz <
                std::abs(static_cast<double>(error)) * right_turn_odom_response_ratio;

        if (odom_underresponding) {
            ++right_turn_odom_underresponse_count;
        } else {
            right_turn_odom_underresponse_count = 0;
        }

        if (right_turn_odom_underresponse_count >= right_turn_odom_confirm_frames) {
            right_wz_compensation =
                turn_sign * right_turn_wz_compensation * right_assist_strength;
            right_wz_compensation_active = true;
        }

        if (right_assist_strength > 0.0 && !odom_fresh) {
            ROS_WARN_THROTTLE(
                1.0,
                "[RIGHT_ASSIST] odom unavailable/stale; keep adaptive aim and speed limit without wz compensation | odom_age=%.3fs",
                odom_age);
        }
    } else {
        right_turn_odom_underresponse_count = 0;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = v;
    const double raw_wz = static_cast<double>(error) + right_wz_compensation;
    msg.angular.z = right_turn_assist_enabled && path_select == PathSelect::RIGHT
                        ? std::max(-right_turn_max_wz,
                                   std::min(right_turn_max_wz, raw_wz))
                        : static_cast<double>(error);

    pub.publish(msg);
    publishDebugImage();
// 主循环调试信息：输出当前选用的路径、路径点数量、是否退化、误差和速度，以及角点检测状态和丢线计数。
    ROS_WARN_THROTTLE(1.0, "[FOLLOW] Running | vision=%s | path=%s | rpts=%d | degraded=%d | error=%.3f rad | v=%.3f m/s | L0=%d | L1=%d | Y0=%d | Y1=%d | lost_line_count=%d | zero_flag=%d",
                  vision_source.c_str(), pathToString(path_select).c_str(), rpts_num, is_degraded_mode,
                  error, v,
                  Lpt0_found, Lpt1_found, Ypt0_found, Ypt1_found,
                  zeroCount, zero_flag);

    if (right_turn_assist_enabled && path_select == PathSelect::RIGHT) {
        ROS_WARN_THROTTLE(
            1.0,
            "[RIGHT_ASSIST] aim=%.3fm/%.3fm | strength=%.2f | error=%.3f | v=%.3f | base_wz=%.3f | odom_wz=%.3f | odom_fresh=%d | under=%d/%d | compensation=%.3f(active=%d) | cmd_wz=%.3f",
            effective_aim_distance, aim_distance, right_assist_strength,
            error, msg.linear.x, error, current_odom_angular_velocity_z,
            odom_fresh, right_turn_odom_underresponse_count,
            right_turn_odom_confirm_frames, right_wz_compensation,
            right_wz_compensation_active, msg.angular.z);
    }

    return 0;
}

std::string currentPathName() {
    return pathToString(path_select);
}

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double turn_angle_deg,
               double turn_angular_speed, int turn_rpts_threshold,
               double turn_pause_sec, double min_turn_pid_speed,
               bool allow_either_l, double extra_parking_dist,
               double forward_parking_speed, double lateral_parking_speed,
               double lateral_parking_deadband, double lateral_cmd_sign,
               const std::string &parking_mode,
               double max_parking_angular_speed,
               double second_arc_max_parking_angular_speed,
               double parking_heading_kp,
               double parking_heading_tolerance_deg,
               double parking_timeout_sec, double parking_odom_timeout_sec,
               bool enable_lost_corner_search,
               double lost_corner_timeout, double lost_corner_angular_speed,
               double lost_corner_linear_speed) {
    // 保存 launch 参数，供后续图像调试、停车开关和速度控制使用。
    publish_debug_image = publish_debug;
    show_window = show_debug_window;
    parking_enabled = enable_parking;
    parking_allow_either_l = allow_either_l;
    parking_extra_dist = std::max(0.0, extra_parking_dist);
    parking_forward_speed = std::max(0.0, forward_parking_speed);
    parking_lateral_speed = std::max(0.0, lateral_parking_speed);
    parking_lateral_deadband = std::max(0.0, lateral_parking_deadband);
    parking_lateral_cmd_sign = lateral_cmd_sign < 0.0 ? -1.0 : 1.0;
    parking_motion_mode = parking_mode;
    std::transform(parking_motion_mode.begin(), parking_motion_mode.end(),
                   parking_motion_mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (parking_motion_mode != "s_curve" && parking_motion_mode != "lateral") {
        ROS_WARN("Unknown parking_motion_mode '%s', fallback to s_curve.",
                 parking_motion_mode.c_str());
        parking_motion_mode = "s_curve";
    }
    parking_max_angular_speed = std::max(0.0, max_parking_angular_speed);
    parking_second_arc_max_angular_speed =
        std::max(0.0, second_arc_max_parking_angular_speed);
    parking_yaw_kp = std::max(0.0, parking_heading_kp);
    parking_yaw_tolerance_deg = std::max(0.0, parking_heading_tolerance_deg);
    parking_timeout = std::max(0.1, parking_timeout_sec);
    parking_odom_timeout = std::max(0.05, parking_odom_timeout_sec);
    base_speed = speed;
    aim_distance = distance;
    aim_y_bias_m = y_bias_m;
    initial_turn_enabled = enable_initial_turn;
    initial_turn_angle_deg = std::max(0.0, turn_angle_deg);
    initial_turn_angular_speed = std::max(0.0, turn_angular_speed);
    initial_turn_rpts_threshold = std::max(1, turn_rpts_threshold);
    initial_turn_pause_sec = std::max(0.0, turn_pause_sec);
    min_pid_speed = std::max(0.0, min_turn_pid_speed);
    lost_corner_search_enabled = enable_lost_corner_search;
    lost_corner_search_timeout = std::max(0.0, lost_corner_timeout);
    lost_corner_search_angular_speed = std::abs(lost_corner_angular_speed);
    lost_corner_search_linear_speed = std::max(0.0, lost_corner_linear_speed);
}

void configureYBranch(double turn_angle_deg,
                      double turn_angular_speed, double turn_pause_sec,
                      int detect_min_id, int detect_max_id,
                      int detect_confirm_frames,
                      double extra_forward_dist, double hard_drive_speed,
                      double hard_drive_odom_timeout,
                      double hard_drive_max_duration,
                      int guided_min_points,
                      int guided_lost_confirm_frames,
                      double guided_error_threshold,
                      int guided_error_confirm_frames,
                      double guided_odom_timeout,
                      double guided_max_duration,
                      double hard_heading_kp,
                      double hard_heading_max_wz,
                      double hard_heading_deadband_deg,
                      double hard_heading_imu_timeout,
                      double reacquire_speed,
                      double reacquire_max_dist,
                      double reacquire_odom_timeout,
                      double reacquire_max_duration,
                      int reacquire_min_points,
                      int reacquire_confirm_frames,
                      double crossbar_seek_speed,
                      int crossbar_lost_confirm_frames,
                      double crossbar_target_long_m,
                      double crossbar_long_tolerance_m,
                      double crossbar_max_abs_lat_m,
                      int crossbar_confirm_frames,
                      double crossbar_seek_max_odom) {
    y_turn_angle_deg = std::max(0.0, turn_angle_deg);
    y_turn_angular_speed = std::abs(turn_angular_speed);
    y_turn_pause_sec = std::max(0.0, turn_pause_sec);
    y_detect_min_id = std::max(0, detect_min_id);
    y_detect_max_id = std::max(y_detect_min_id, detect_max_id);
    y_detect_confirm_frames = std::max(1, detect_confirm_frames);
    y_extra_forward_dist = std::max(0.0, extra_forward_dist);
    y_hard_drive_speed = std::max(0.0, hard_drive_speed);
    y_hard_drive_odom_timeout = std::max(0.05, hard_drive_odom_timeout);
    y_hard_drive_max_duration = std::max(0.1, hard_drive_max_duration);
    y_guided_min_points = std::max(1, guided_min_points);
    y_guided_lost_confirm_frames = std::max(1, guided_lost_confirm_frames);
    y_guided_error_threshold = std::max(0.0, guided_error_threshold);
    y_guided_error_confirm_frames = std::max(1, guided_error_confirm_frames);
    y_guided_odom_timeout = std::max(0.05, guided_odom_timeout);
    y_guided_max_duration = std::max(0.1, guided_max_duration);
    y_hard_heading_kp = std::max(0.0, hard_heading_kp);
    y_hard_heading_max_wz = std::max(0.0, hard_heading_max_wz);
    y_hard_heading_deadband_deg = std::max(0.0, hard_heading_deadband_deg);
    y_hard_heading_imu_timeout = std::max(0.05, hard_heading_imu_timeout);
    y_reacquire_speed = std::max(0.0, reacquire_speed);
    y_reacquire_max_dist = std::max(0.0, reacquire_max_dist);
    y_reacquire_odom_timeout = std::max(0.05, reacquire_odom_timeout);
    y_reacquire_max_duration = std::max(0.1, reacquire_max_duration);
    y_reacquire_min_points = std::max(1, reacquire_min_points);
    y_reacquire_confirm_frames = std::max(1, reacquire_confirm_frames);
    y_crossbar_seek_speed = std::max(0.0, crossbar_seek_speed);
    y_crossbar_lost_confirm_frames = std::max(1, crossbar_lost_confirm_frames);
    y_crossbar_target_long_m = std::max(0.0, crossbar_target_long_m);
    y_crossbar_long_tolerance_m = std::max(0.0, crossbar_long_tolerance_m);
    y_crossbar_max_abs_lat_m = std::max(0.0, crossbar_max_abs_lat_m);
    y_crossbar_confirm_frames = std::max(1, crossbar_confirm_frames);
    y_crossbar_seek_max_odom = std::max(0.0, crossbar_seek_max_odom);
}

void configureMotionController(const MotionControlConfig &config) {
    motion_controller.configure(config);
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
    resetYBranchState();
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
