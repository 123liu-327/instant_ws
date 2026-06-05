#ifndef FLOW_END_FOLLOW_LINE_TEST_H
#define FLOW_END_FOLLOW_LINE_TEST_H

#include <flow_end/follow.h>
#include <flow_end/follow_motion_controller.h>

#include <ros/ros.h>

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
    Y_APPROACH,
    Y_CENTER_APPROACH,
    Y_CROSSBAR_SEEK,
    Y_ALIGNING_LEFT,
    Y_ALIGNING_RIGHT,
    Y_ALIGN_PAUSE
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
extern double base_speed;
extern double aim_distance;
extern double aim_y_bias_m;
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
extern double y_approach_dist;
extern double y_turn_angle_deg;
extern double y_turn_angular_speed;
extern double y_turn_pause_sec;
extern int y_detect_min_id;
extern int y_detect_max_id;
extern int y_detect_confirm_frames;
extern double y_center_aim_dist;
extern double y_approach_speed;
extern double y_center_max_wz;
extern int y_lost_confirm_frames;
extern double y_entry_min_odom;
extern double y_entry_max_odom;
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

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double initial_turn_angle_deg,
               double initial_turn_angular_speed, int initial_turn_rpts_threshold,
               double initial_turn_pause_sec, double min_pid_speed);

void configure(bool publish_debug, bool show_debug_window, bool enable_parking,
               double speed, double distance, double y_bias_m,
               bool enable_initial_turn, double initial_turn_angle_deg,
               double initial_turn_angular_speed, int initial_turn_rpts_threshold,
               double initial_turn_pause_sec, double min_pid_speed,
               double y_approach_dist, double y_turn_angle_deg,
               double y_turn_angular_speed, double y_turn_pause_sec,
               int y_detect_min_id, int y_detect_max_id,
               int y_detect_confirm_frames, double y_center_aim_dist,
               double y_approach_speed, double y_center_max_wz,
               int y_lost_confirm_frames, double y_entry_min_odom,
               double y_entry_max_odom, bool parking_allow_either_l,
               double parking_extra_dist, double parking_forward_speed,
               double y_crossbar_seek_speed,
               int y_crossbar_lost_confirm_frames,
               double y_crossbar_target_long_m,
               double y_crossbar_long_tolerance_m,
               double y_crossbar_max_abs_lat_m,
               int y_crossbar_confirm_frames,
               double y_crossbar_seek_max_odom);

void configureVideo(bool enable_record, int fps, const std::string &save_path);
void configureMotionController(const MotionControlConfig &config);
void resetMotionController();

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
