#include <flow_end/follow_motion_controller.h>

#include <algorithm>
#include <cmath>

namespace flow_end {
namespace follow_test {
namespace {

// 小工具：把数值限制在 [low, high]。控制器里所有限幅都走这个函数，
// 避免参数异常时输出过大的速度或角速度。
double clampDouble(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

// 一阶低通/EMA：
// alpha 越大越相信 current，响应越快；alpha 越小越相信 previous，越平滑。
// 这里同时用于路径点、角度误差和最终速度命令。
double blend(double previous, double current, double alpha) {
    const double a = clampDouble(alpha, 0.0, 1.0);
    return a * current + (1.0 - a) * previous;
}

}  // namespace

// 配置控制器参数，并做基础合法化处理。
// 这里不假设 launch 传进来的值一定合理：
// 1. alpha 类参数限制在 0~1；
// 2. 速度、角速度、时间类参数保证非负；
// 3. slow_error 防止除 0。
void FollowMotionController::configure(const MotionControlConfig &config) {
    config_ = config;
    config_.path_smooth_window = std::max(0, config_.path_smooth_window);
    config_.path_ema_alpha = clampDouble(config_.path_ema_alpha, 0.0, 1.0);
    config_.error_filter_alpha = clampDouble(config_.error_filter_alpha, 0.0, 1.0);
    config_.cmd_filter_alpha = clampDouble(config_.cmd_filter_alpha, 0.0, 1.0);
    config_.slow_error = std::max(0.001, config_.slow_error);
    config_.max_accel = std::max(0.0, config_.max_accel);
    config_.max_decel = std::max(0.0, config_.max_decel);
    config_.max_wz_rate = std::max(0.0, config_.max_wz_rate);
    config_.max_wz = std::max(0.0, config_.max_wz);
    config_.soft_wz_limit = std::max(0.0, config_.soft_wz_limit);
    config_.lost_line_coast_sec = std::max(0.0, config_.lost_line_coast_sec);
    config_.lost_line_coast_speed_scale = clampDouble(config_.lost_line_coast_speed_scale, 0.0, 1.0);
}

// 清空控制器所有历史状态。
// 使用场景：
// - /follow_begin 切换路径；
// - Stop；
// - 起步预转角/Y 二次预转角接管；
// - 停车接管。
// 这样做的目的是避免上一段巡线的滤波速度、角速度和误差影响下一段动作。
void FollowMotionController::reset() {
    has_last_time_ = false;
    has_filtered_error_ = false;
    has_last_valid_cmd_ = false;
    filtered_error_ = 0.0;
    last_error_ = 0.0;
    integral_error_ = 0.0;
    last_cmd_v_ = 0.0;
    last_cmd_wz_ = 0.0;
    last_path_key_ = -1;
    last_path_num_ = 0;
    last_valid_cmd_ = geometry_msgs::Twist();
    resetPathHistory();
}

// 只清掉路径平滑历史，不清速度控制历史。
// 当当前帧没有路径点，或者路径来源发生明显变化时使用。
void FollowMotionController::resetPathHistory() {
    has_smoothed_path_ = false;
    last_path_num_ = 0;
}

// 对输入控制路径做两层平滑：
// 1. 空间平滑：同一帧里对每个点和它前后的邻居求均值，抑制单个坏点；
// 2. 时间平滑：同一索引点跨帧做 EMA，减少视觉抖动引起的方向跳变。
//
// 注意：
// - 这里不修改 follow_test 的原始 rpts，只写到 smoothed_path_；
// - path_key 变化表示 left/right/middle 或阶段切换，需要重置历史；
// - 点数突变过大时也重置，避免旧路径尾巴混入新路径。
int FollowMotionController::smoothPath(const MotionControlInput &input) {
    const int count = std::max(0, std::min(input.path_num, kMaxPoints));
    if (count <= 0 || input.path == nullptr) {
        resetPathHistory();
        return 0;
    }

    if (last_path_key_ != input.path_key || !has_smoothed_path_ ||
        std::abs(count - last_path_num_) > 20) {
        has_smoothed_path_ = false;
    }

    float spatial[kMaxPoints][2] = {};
    const int window = config_.path_smooth_window;
    for (int i = 0; i < count; ++i) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int samples = 0;
        const int start = std::max(0, i - window);
        const int end = std::min(count - 1, i + window);
        for (int j = start; j <= end; ++j) {
            sum_x += input.path[j][0];
            sum_y += input.path[j][1];
            ++samples;
        }
        spatial[i][0] = static_cast<float>(sum_x / std::max(1, samples));
        spatial[i][1] = static_cast<float>(sum_y / std::max(1, samples));
    }

    if (!has_smoothed_path_) {
        for (int i = 0; i < count; ++i) {
            smoothed_path_[i][0] = spatial[i][0];
            smoothed_path_[i][1] = spatial[i][1];
        }
        has_smoothed_path_ = true;
    } else {
        for (int i = 0; i < count; ++i) {
            smoothed_path_[i][0] = static_cast<float>(
                blend(smoothed_path_[i][0], spatial[i][0], config_.path_ema_alpha));
            smoothed_path_[i][1] = static_cast<float>(
                blend(smoothed_path_[i][1], spatial[i][1], config_.path_ema_alpha));
        }
    }

    last_path_key_ = input.path_key;
    last_path_num_ = count;
    return count;
}

// 对目标速度命令做最终输出平滑。
// 这一步作用在 Twist 上，和前面的误差/路径滤波不同：
// - linear.x 用 max_accel/max_decel 限制加减速度；
// - angular.z 用 max_wz_rate 限制角速度变化率；
// - 最后再用 cmd_filter_alpha 做一层低通。
//
// dt 异常或首帧时直接采用目标值，避免用错误时间差算出巨大变化量。
geometry_msgs::Twist FollowMotionController::smoothCommand(double target_v,
                                                           double target_wz,
                                                           double dt) {
    geometry_msgs::Twist cmd;

    if (dt <= 0.0) {
        last_cmd_v_ = target_v;
        last_cmd_wz_ = target_wz;
    } else {
        double dv = target_v - last_cmd_v_;
        if (dv > 0.0) {
            dv = std::min(dv, config_.max_accel * dt);
        } else {
            dv = std::max(dv, -config_.max_decel * dt);
        }
        double limited_v = last_cmd_v_ + dv;

        double dw = target_wz - last_cmd_wz_;
        if (config_.max_wz_rate > 0.0) {
            dw = clampDouble(dw, -config_.max_wz_rate * dt, config_.max_wz_rate * dt);
        }
        double limited_wz = last_cmd_wz_ + dw;

        last_cmd_v_ = blend(last_cmd_v_, limited_v, config_.cmd_filter_alpha);
        last_cmd_wz_ = blend(last_cmd_wz_, limited_wz, config_.cmd_filter_alpha);
    }

    cmd.linear.x = last_cmd_v_;
    cmd.angular.z = last_cmd_wz_;
    return cmd;
}

// 主控制入口：把前视路径点转换成底盘 Twist。
//
// 处理流程：
// 1. 计算 dt，用于 D 项、积分项和速度变化率限制；
// 2. 平滑路径点；
// 3. 如果丢线，普通巡线可短暂沿用最后有效命令，否则平滑停车；
// 4. 根据 aim_distance 选择前视点，计算 raw_error；
// 5. 对 error 做 EMA 和死区；
// 6. 用 P/I/D 得到目标角速度，并做软/硬限幅；
// 7. 根据误差大小规划线速度，退化巡线时降速；
// 8. 对最终 v/wz 做输出平滑，并保存为 last_valid_cmd_。
//
// 符号约定保持 follow_test 当前习惯：
// raw_error/filtered_error 为正时，angular.z 也倾向为正，即左转；
// 为负时右转。
MotionControlOutput FollowMotionController::compute(const MotionControlInput &input) {
    MotionControlOutput output;

    // dt 只在合理范围内使用。图像卡顿、ROS 时间跳变等情况下跳过 D/积分时间，
    // 避免一帧异常导致角速度突变。
    const ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (has_last_time_) {
        dt = (now - last_time_).toSec();
        if (dt < 0.0 || dt > 0.2) {
            dt = 0.0;
        }
    }
    last_time_ = now;
    has_last_time_ = true;

    const int path_count = smoothPath(input);
    output.lost = (path_count <= 0);

    // 丢线策略：
    // 普通巡线 allow_lost_coast=true 时，短时间继续发布最后有效命令的衰减版本，
    // 减少偶发一帧丢线造成的急刹。Y 入口等场景可关闭，保留丢线作为状态机信号。
    if (output.lost) {
        integral_error_ = 0.0;
        if (input.allow_lost_coast && has_last_valid_cmd_) {
            const double lost_elapsed = (now - last_valid_time_).toSec();
            if (lost_elapsed >= 0.0 && lost_elapsed <= config_.lost_line_coast_sec) {
                output.coasting = true;
                output.target_v = last_valid_cmd_.linear.x * config_.lost_line_coast_speed_scale;
                output.cmd = smoothCommand(output.target_v,
                                           last_valid_cmd_.angular.z * config_.lost_line_coast_speed_scale,
                                           dt);
                return output;
            }
        }
        output.cmd = smoothCommand(0.0, 0.0, dt);
        return output;
    }

    // 根据米制前视距离换算到采样点下标。sample_dist 是路径点间距，
    // aim_idx 越大看得越远，车辆更稳但转弯响应会慢。
    const int aim_idx = std::max(0, std::min(
        static_cast<int>(std::round(input.aim_distance / std::max(0.001, input.sample_dist))),
        path_count - 1));

    // 控制点坐标以图像中心下方作为车体参考点。
    // dy 中加入 aim_y_bias_m，可以保留原 follow_test 中用于贴左/贴右的纵向偏置习惯。
    const double cx = input.image_width / 2.0;
    const double cy = input.image_height + 10.0;
    const double dx = smoothed_path_[aim_idx][0] - cx;
    const double dy = cy - smoothed_path_[aim_idx][1] + input.aim_y_bias_m * input.pixel_per_meter;
    output.raw_error = -std::atan2(dx, dy);

    // 误差低通 + 死区。低通减少视觉点跳变，死区减少直线段轻微左右摆。
    if (!has_filtered_error_) {
        filtered_error_ = output.raw_error;
        has_filtered_error_ = true;
    } else {
        filtered_error_ = blend(filtered_error_, output.raw_error, config_.error_filter_alpha);
    }

    if (std::abs(filtered_error_) < config_.yaw_deadband) {
        filtered_error_ = 0.0;
    } else {
        filtered_error_ += filtered_error_ > 0.0 ? -config_.yaw_deadband : config_.yaw_deadband;
    }
    output.filtered_error = filtered_error_;

    const bool adaptive = std::abs(filtered_error_) >= config_.adaptive_error_threshold;
    const double kp = config_.kp_yaw * (adaptive ? config_.adaptive_kp_scale : 1.0);
    const double kd = config_.kd_yaw * (adaptive ? config_.adaptive_kd_scale : 1.0);

    // 积分分离：只有误差不大时才积累 I 项。
    // 大误差时积分容易越积越偏，反而导致转弯后过冲。
    if (std::abs(filtered_error_) <= config_.integral_error_threshold) {
        integral_error_ += filtered_error_ * std::max(0.0, dt);
        integral_error_ = clampDouble(integral_error_, -config_.integral_limit, config_.integral_limit);
    } else {
        integral_error_ = 0.0;
    }

    double derivative = 0.0;
    if (dt > 0.0) {
        derivative = (filtered_error_ - last_error_) / dt;
    }
    last_error_ = filtered_error_;

    // P/I/D 生成目标角速度。默认 Ki 为 0，因此实际主要依靠 P 纠偏、D 抑制跳变。
    double target_wz = kp * filtered_error_ + config_.ki_yaw * integral_error_ + kd * derivative;

    // 特殊阶段可通过 max_wz_override 单独限制角速度，例如 Y_CENTER_APPROACH。
    const double max_wz = input.max_wz_override > 0.0 ? input.max_wz_override : config_.max_wz;
    const double soft_limit = std::min(max_wz, config_.soft_wz_limit);
    target_wz = clampDouble(target_wz, -soft_limit, soft_limit);
    target_wz = clampDouble(target_wz, -max_wz, max_wz);

    // 误差越大越降速。这样直线可以保持 base_speed，弯道自动慢下来。
    // degraded=true 表示当前路径不是首选线，额外降速以提高容错。
    const double turn_ratio = clampDouble(std::abs(filtered_error_) / config_.slow_error, 0.0, 1.0);
    output.target_v = input.base_speed * (1.0 - config_.turn_slowdown * turn_ratio);
    if (input.degraded) {
        output.target_v *= config_.degraded_speed_scale;
    }
    output.target_v = std::max(config_.min_speed, output.target_v);

    // 最后一层命令平滑，并保存有效命令供短暂丢线续行使用。
    output.cmd = smoothCommand(output.target_v, target_wz, dt);
    last_valid_cmd_ = output.cmd;
    last_valid_time_ = now;
    has_last_valid_cmd_ = true;
    return output;
}

}  // namespace follow_test
}  // namespace flow_end
