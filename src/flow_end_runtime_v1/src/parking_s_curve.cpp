#include <flow_end/parking_s_curve.h>

#include <algorithm>
#include <cmath>

namespace flow_end {
namespace follow_test {

ParkingSCurvePlan makeParkingSCurvePlan(double longitudinal,
                                        double lateral,
                                        double requested_speed,
                                        double lateral_deadband,
                                        double max_angular_speed) {
    ParkingSCurvePlan plan;
    plan.longitudinal = std::max(0.001, longitudinal);
    plan.lateral = lateral;
    plan.forward_speed = std::max(0.0, requested_speed);
    plan.total_length = plan.longitudinal;

    const double abs_lateral = std::abs(lateral);
    if (abs_lateral < std::max(0.0, lateral_deadband)) {
        return plan;
    }

    plan.curved = true;
    plan.direction = lateral > 0.0 ? 1.0 : -1.0;
    plan.peak_yaw = 2.0 * std::atan2(abs_lateral, plan.longitudinal);
    plan.radius = (plan.longitudinal * plan.longitudinal + abs_lateral * abs_lateral) /
                  (4.0 * abs_lateral);
    plan.total_length = 2.0 * plan.radius * plan.peak_yaw;

    const double max_wz = std::max(0.0, max_angular_speed);
    if (plan.radius > 0.0 && max_wz > 0.0) {
        plan.forward_speed = std::min(plan.forward_speed, max_wz * plan.radius);
        plan.feedforward_wz = plan.forward_speed / plan.radius;
    } else {
        plan.forward_speed = 0.0;
        plan.feedforward_wz = 0.0;
    }
    return plan;
}

double parkingSCurveReferenceYaw(const ParkingSCurvePlan &plan,
                                 double traveled) {
    if (!plan.curved || plan.radius <= 0.0 || plan.total_length <= 0.0) {
        return 0.0;
    }

    const double progress = std::max(0.0, std::min(traveled, plan.total_length));
    const double half_length = plan.total_length * 0.5;
    if (progress <= half_length) {
        return plan.direction * progress / plan.radius;
    }
    return plan.direction * (plan.total_length - progress) / plan.radius;
}

double parkingSCurveFeedforwardWz(const ParkingSCurvePlan &plan,
                                  double traveled) {
    if (!plan.curved || traveled >= plan.total_length) {
        return 0.0;
    }
    return traveled < plan.total_length * 0.5
        ? plan.direction * plan.feedforward_wz
        : -plan.direction * plan.feedforward_wz;
}

double normalizeAngle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

}  // namespace follow_test
}  // namespace flow_end
