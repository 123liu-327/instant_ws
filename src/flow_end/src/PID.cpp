#include "flow_end/PID.h"

PIDController::PIDController(const Params &params) : params_(params), integral_(0), prev_error_(0)
{
    last_time_ = std::chrono::steady_clock::now();
}

void PIDController::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
    last_time_ = std::chrono::steady_clock::now();
}

double PIDController::compute(double target, double measurement)
{
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count() / 1000.0;
    dt = std::max(dt, 0.001); 

    double error = target - measurement;

    if (std::abs(error) < params_.dead_zone)
    {
        error = 0.0;
    }

    double P = params_.Kp * error;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -params_.max_integral, params_.max_integral);
    double I = params_.Ki * integral_;

    double derivative = (error - prev_error_) / dt;
    double D = params_.Kd * derivative;

    double output = P + I + D;
    output = std::clamp(output, -params_.max_output, params_.max_output);

    prev_error_ = error;
    last_time_ = now;

    return output;
}

void PIDController::update_params(const Params &params)
{
    params_ = params;
    integral_ = std::clamp(integral_, -params_.max_integral, params_.max_integral);
}

// 变量定义与初始化
PIDController::Params params{
    0.05,  // Kp
    0.00,  // Ki
    0.0,   // Kd
    0.5,   // max_output
    0.5,   // max_integral
    0.01   // dead_zone
};

// PID 参数配置（可以根据需要调整）
PIDController::Params linear_params{
    0.1,  // Kp
    0.01, // Ki
    0.005, // Kd
    1.0,  // max_output
    0.5,  // max_integral
    0.01  // dead_zone
};

PIDController::Params angular_params{
    0.2,  // Kp
    0.01, // Ki
    0.005, // Kd
    1.0,  // max_output
    0.5,  // max_integral
    0.01  // dead_zone
};

// PID 控制器实例
PIDController linear_pid(linear_params);
PIDController angular_pid(angular_params);

PIDController pid(params);
