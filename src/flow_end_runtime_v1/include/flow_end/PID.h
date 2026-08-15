#ifndef FLOW_END_PID_H
#define FLOW_END_PID_H

#include <chrono>
#include <algorithm>
#include <cmath>

class PIDController
{
public:
    struct Params
    {
        double Kp;          
        double Ki;           
        double Kd;           
        double max_output;   
        double max_integral; 
        double dead_zone;    
    };

    PIDController(const Params &params);

    void reset();

    double compute(double target, double measurement);

    void update_params(const Params &params);

private:
    Params params_;
    double integral_;
    double prev_error_;
    std::chrono::steady_clock::time_point last_time_;
};

// 声明外部变量，由 cpp 文件定义
extern PIDController::Params params;
extern PIDController::Params linear_params;
extern PIDController::Params angular_params;
extern PIDController pid;
extern PIDController linear_pid;
extern PIDController angular_pid;
#endif
