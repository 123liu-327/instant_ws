#ifndef FLOW_END_PARKING_S_CURVE_H
#define FLOW_END_PARKING_S_CURVE_H

namespace flow_end {
namespace follow_test {

struct ParkingSCurvePlan {
    bool curved = false;
    double longitudinal = 0.0;
    double lateral = 0.0;
    double direction = 0.0;
    double radius = 0.0;
    double peak_yaw = 0.0;
    double total_length = 0.0;
    double forward_speed = 0.0;
    double feedforward_wz = 0.0;
};

ParkingSCurvePlan makeParkingSCurvePlan(double longitudinal,
                                        double lateral,
                                        double requested_speed,
                                        double lateral_deadband,
                                        double max_angular_speed);

double parkingSCurveReferenceYaw(const ParkingSCurvePlan &plan,
                                 double traveled);

double parkingSCurveFeedforwardWz(const ParkingSCurvePlan &plan,
                                  double traveled);

double normalizeAngle(double angle);

}  // namespace follow_test
}  // namespace flow_end

#endif  // FLOW_END_PARKING_S_CURVE_H
