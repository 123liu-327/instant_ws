#ifndef FLOW_END_FOLLOW_LINE_TEST_H
#define FLOW_END_FOLLOW_LINE_TEST_H

#include <flow_end/follow.h>
#include <flow_end/follow_motion_controller.h>

#include <ros/ros.h>
#include <line_follower/LineTrack.h>

#include <string>

namespace flow_end {
namespace follow_test {

enum class PathSelect { LEFT, MIDDLE, RIGHT };
enum class MotionState {
    IDLE,
    ALIGNING_LEFT,
    ALIGNING_RIGHT,
    ALIGN_PAUSE,
    FOLLOWING,
    FOLLOWING_STRAIGHT,
    Y_GUIDED_FORWARD,
    Y_COORDINATE_FORWARD,
    Y_CROSSBAR_SEEK,
    Y_ALIGNING_LEFT,
    Y_ALIGNING_RIGHT,
    Y_ALIGN_PAUSE,
    Y_BRANCH_REACQUIRE
};

// Shared follow_test state. Callback_test.cpp updates these through this header;
// follow_line_test.cpp owns the definitions and the line-following behavior.
extern PathSelect path_select;
extern MotionState motion_state;
extern ros::Publisher debug_pub;
extern ros::Publisher status_pub;
extern float middle_path[POINTS_MAX_LEN][2];
extern int middle_path_num;

extern bool publish_debug_image;
extern bool show_window;
extern bool parking_enabled;
extern bool parking_allow_either_l;
extern double parking_extra_dist;
extern double parking_forward_speed;
extern double parking_lateral_speed;
extern double parking_lateral_deadband;
extern double parking_lateral_cmd_sign;
extern std::string parking_motion_mode;
extern double parking_max_angular_speed;
extern double parking_second_arc_max_angular_speed;
extern double parking_yaw_kp;
extern double parking_yaw_tolerance_deg;
extern double parking_timeout;
extern double parking_odom_timeout;
extern ros::Time last_odom_time;
extern ros::Time last_imu_time;
extern double current_odom_position_x;
extern double current_odom_position_y;
extern double odom_total_distance;
extern double base_speed;
extern double aim_distance;
extern double aim_y_bias_m;
extern bool right_turn_assist_enabled;
extern double right_turn_min_aim_distance;
extern double right_turn_error_start;
extern double right_turn_error_full;
extern double right_turn_min_speed;
extern double right_turn_wz_compensation;
extern double right_turn_max_wz;
extern double right_turn_odom_timeout;
extern double right_turn_odom_response_ratio;
extern int right_turn_odom_confirm_frames;
extern double current_odom_angular_velocity_z;
extern std::string vision_source;
extern double line_track_timeout;
extern double line_track_min_confidence;
extern int line_track_min_points;
extern bool initial_turn_enabled;
extern double initial_turn_angle_deg;
extern double initial_turn_angular_speed;
extern int initial_turn_rpts_threshold;
extern double initial_turn_pause_sec;
extern double initial_turn_integrated_angle_deg;
extern ros::Time initial_turn_last_time;
extern bool initial_turn_has_last_time;
extern double min_pid_speed;
extern ros::Time initial_turn_pause_start;
extern bool lost_corner_search_enabled;
extern double lost_corner_search_timeout;
extern double lost_corner_search_angular_speed;
extern double lost_corner_search_linear_speed;
extern double y_turn_angle_deg;
extern double y_turn_angular_speed;
extern double y_turn_pause_sec;
extern int y_detect_min_id;
extern int y_detect_max_id;
extern int y_detect_confirm_frames;
extern double y_extra_forward_dist;
extern double y_hard_drive_speed;
extern double y_hard_drive_odom_timeout;
extern double y_hard_drive_max_duration;
extern int y_guided_min_points;
extern int y_guided_lost_confirm_frames;
extern double y_guided_error_threshold;
extern int y_guided_error_confirm_frames;
extern double y_guided_odom_timeout;
extern double y_guided_max_duration;
extern double y_hard_heading_kp;
extern double y_hard_heading_max_wz;
extern double y_hard_heading_deadband_deg;
extern double y_hard_heading_imu_timeout;
extern double y_reacquire_speed;
extern double y_reacquire_max_dist;
extern double y_reacquire_odom_timeout;
extern double y_reacquire_max_duration;
extern int y_reacquire_min_points;
extern int y_reacquire_confirm_frames;
extern double y_crossbar_seek_speed;
extern int y_crossbar_lost_confirm_frames;
extern double y_crossbar_target_long_m;
extern double y_crossbar_long_tolerance_m;
extern double y_crossbar_max_abs_lat_m;
extern int y_crossbar_confirm_frames;
extern double y_crossbar_seek_max_odom;

// 视频保存相关配置
extern bool enable_video_record;
extern int video_fps;
extern std::string video_save_path;

bool setPathSelect(const std::string &raw_value);
std::string currentPathName();
std::string pathToString(PathSelect path);
void applyPathBiasParams(PathSelect path);

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double initial_turn_angle_deg,
               double initial_turn_angular_speed, int initial_turn_rpts_threshold,
               double initial_turn_pause_sec, double min_pid_speed,
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
               double lost_corner_linear_speed);

void configureVideo(bool enable_record, int fps, const std::string &save_path);
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
                      double crossbar_seek_max_odom);
void configureMotionController(const MotionControlConfig &config);
void configureRightTurnAssist(bool enabled, double min_aim_distance,
                              double error_start, double error_full,
                              double min_speed, double wz_compensation,
                              double max_wz, double odom_timeout,
                              double odom_response_ratio,
                              int odom_confirm_frames);
void resetMotionController();
void resetYBranchState();
void configureVisionSource(const std::string &source, double timeout,
                           double min_confidence, int min_points);
void updateLineTrack(const line_follower::LineTrack::ConstPtr &msg);
void resetExternalVisionState();

void initializeImagePipeline();
void startInitialTurnIfNeeded();
void resetParkingCornerState();
void publishStatus(const std::string &state);
void publishStop();
int followLineTestOnce();
bool shouldExit();
void shutdown();

}  // namespace follow_test
}  // namespace flow_end

#endif  // FLOW_END_FOLLOW_LINE_TEST_H
