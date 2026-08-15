#ifndef FLOW_END_FOLLOW_MOTION_CONTROLLER_H
#define FLOW_END_FOLLOW_MOTION_CONTROLLER_H

#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <string>

namespace flow_end {
namespace follow_test {

// MotionControlConfig 是巡线运动控制器的全部可调参数。
// 这些参数只影响“控制路径点 -> /cmd_vel”的过程，不参与图像处理、
// L/Y 角点检测、停车状态机和预转角状态机。
struct MotionControlConfig {
    // 控制路径点平滑：
    // path_smooth_window 做当前帧内相邻点均值，抑制单个路径点跳动；
    // path_ema_alpha 做帧间低通，越小越稳，越大越跟手。
    int path_smooth_window = 2;
    double path_ema_alpha = 0.35;

    // 误差滤波与死区：
    // error_filter_alpha 过滤前视点角度误差；
    // yaw_deadband 把很小的左右抖动归零，减少直线段左右摆。
    double error_filter_alpha = 0.65;
    double yaw_deadband = 0.015;

    // 航向 PID 参数。默认 Ki 为 0，实际主要是 P + D：
    // P 负责纠偏力度，D 负责抑制快速变化导致的过冲；
    // integral_* 只在启用 Ki 时有明显作用。
    double kp_yaw = 1.00;
    double ki_yaw = 0.00;
    double kd_yaw = 0.08;
    double integral_limit = 0.30;
    double integral_error_threshold = 0.25;

    // 大误差自适应增强。弯道或偏离较大时临时提高 P/D，
    // 让车能更快拉回；直线小误差时保持温和。
    double adaptive_error_threshold = 0.35;
    double adaptive_kp_scale = 1.25;
    double adaptive_kd_scale = 1.40;

    // 角速度约束：
    // soft_wz_limit 是常规软限幅，max_wz 是最终硬限幅；
    // max_wz_rate 限制角速度每秒变化量，让底盘转向更连续。
    double max_wz = 0.65;
    double soft_wz_limit = 0.55;
    double max_wz_rate = 1.80;

    // 线速度规划：
    // 转弯误差越大，速度越低；退化巡线时再乘 degraded_speed_scale。
    double turn_slowdown = 0.65;
    double slow_error = 0.45;
    double min_speed = 0.05;
    double degraded_speed_scale = 0.75;

    // 速度输出平滑：
    // max_accel/max_decel 限制 linear.x 的变化；
    // cmd_filter_alpha 对最终 v/wz 做低通。
    double max_accel = 0.35;
    double max_decel = 0.80;
    double cmd_filter_alpha = 0.45;

    // 普通巡线短暂丢线的续行策略。
    // Y 岔路入口等需要把“丢线”当状态机信号的场景，可以在 input 中关闭。
    double lost_line_coast_sec = 0.15;
    double lost_line_coast_speed_scale = 0.60;
};

// MotionControlInput 是每一帧传给控制器的观测和运行上下文。
// 调用者负责先选好控制路径 rpts；控制器只负责把它变成 Twist。
struct MotionControlInput {
    // path 指向当前控制路径点数组，形如 rpts/rptsc0e/rptsc1e/middle_path。
    // path_num 为有效点数。path_key 用于识别 left/right/middle 切换，
    // 切换时控制器会自动重置路径平滑历史。
    float (*path)[2] = nullptr;
    int path_num = 0;
    int path_key = 0;
    std::string path_name = "unknown";
    bool degraded = false;

    // 当前帧的基础巡线参数。base_speed 可由普通巡线或 Y 靠近阶段传入不同值。
    // aim_distance/sample_dist 决定选第几个前视路径点；
    // aim_y_bias_m 用于保持原 follow_test 的左右偏置习惯。
    double base_speed = 0.30;
    double aim_distance = 0.10;
    double aim_y_bias_m = 0.20;
    double sample_dist = 0.01;
    double pixel_per_meter = 200.0;
    int image_width = 640;
    int image_height = 480;

    // allow_lost_coast 控制丢线时是否短暂沿用上一条有效命令。
    // max_wz_override 用于 Y_CENTER_APPROACH 等特殊阶段单独限制角速度。
    bool allow_lost_coast = true;
    double max_wz_override = -1.0;
};

// MotionControlOutput 除了最终 Twist，还保留中间量，方便日志分析和实车调参。
struct MotionControlOutput {
    geometry_msgs::Twist cmd;
    double raw_error = 0.0;
    double filtered_error = 0.0;
    double target_v = 0.0;
    bool lost = false;
    bool coasting = false;
};

class FollowMotionController {
public:
    // 写入并钳制配置参数。可在 rosparam 刷新后调用。
    void configure(const MotionControlConfig &config);

    // 清空所有历史状态。路径切换、停车、预转角、Stop 时都应调用，
    // 避免上一段巡线速度/误差残留到下一段动作。
    void reset();

    // 根据当前路径点和上下文计算底盘速度命令。
    // 这是控制器主入口，调用频率跟 follow_test 主循环一致。
    MotionControlOutput compute(const MotionControlInput &input);

private:
    MotionControlConfig config_;
    bool has_last_time_ = false;
    bool has_filtered_error_ = false;
    bool has_smoothed_path_ = false;
    bool has_last_valid_cmd_ = false;
    int last_path_key_ = -1;
    int last_path_num_ = 0;

    ros::Time last_time_;
    ros::Time last_valid_time_;
    double filtered_error_ = 0.0;
    double last_error_ = 0.0;
    double integral_error_ = 0.0;
    double last_cmd_v_ = 0.0;
    double last_cmd_wz_ = 0.0;
    geometry_msgs::Twist last_valid_cmd_;

    static constexpr int kMaxPoints = 300;
    float smoothed_path_[kMaxPoints][2] = {};

    // 只重置路径平滑历史，不清空速度/误差控制历史。
    void resetPathHistory();

    // 对输入路径做当前帧空间平滑 + 帧间 EMA，并返回可用点数。
    int smoothPath(const MotionControlInput &input);

    // 对目标 v/wz 做限加速度、限角速度变化率和最终低通。
    geometry_msgs::Twist smoothCommand(double target_v, double target_wz, double dt);
};

}  // namespace follow_test
}  // namespace flow_end

#endif  // FLOW_END_FOLLOW_MOTION_CONTROLLER_H
