#include <flow_end/parking_s_curve.h>

#include <gtest/gtest.h>

#include <cmath>

namespace flow_end {
namespace follow_test {
namespace {

TEST(ParkingSCurve, MirrorsLeftAndRight) {
    const ParkingSCurvePlan left = makeParkingSCurvePlan(0.40, 0.10, 0.15, 0.03, 0.35);
    const ParkingSCurvePlan right = makeParkingSCurvePlan(0.40, -0.10, 0.15, 0.03, 0.35);

    ASSERT_TRUE(left.curved);
    ASSERT_TRUE(right.curved);
    EXPECT_DOUBLE_EQ(left.radius, right.radius);
    EXPECT_DOUBLE_EQ(left.total_length, right.total_length);
    EXPECT_GT(parkingSCurveReferenceYaw(left, left.total_length * 0.5), 0.0);
    EXPECT_LT(parkingSCurveReferenceYaw(right, right.total_length * 0.5), 0.0);
    EXPECT_GT(parkingSCurveFeedforwardWz(left, left.total_length * 0.25), 0.0);
    EXPECT_LT(parkingSCurveFeedforwardWz(left, left.total_length * 0.75), 0.0);
    EXPECT_LT(parkingSCurveFeedforwardWz(right, right.total_length * 0.25), 0.0);
    EXPECT_GT(parkingSCurveFeedforwardWz(right, right.total_length * 0.75), 0.0);
}

TEST(ParkingSCurve, DrivesStraightInsideDeadband) {
    const ParkingSCurvePlan plan = makeParkingSCurvePlan(0.40, 0.02, 0.15, 0.03, 0.35);

    EXPECT_FALSE(plan.curved);
    EXPECT_DOUBLE_EQ(plan.total_length, 0.40);
    EXPECT_DOUBLE_EQ(parkingSCurveReferenceYaw(plan, 0.20), 0.0);
    EXPECT_DOUBLE_EQ(parkingSCurveFeedforwardWz(plan, 0.20), 0.0);
}

TEST(ParkingSCurve, ReducesSpeedToPreserveLimitedCurvature) {
    const ParkingSCurvePlan plan = makeParkingSCurvePlan(0.20, 0.10, 0.30, 0.03, 0.20);

    ASSERT_TRUE(plan.curved);
    EXPECT_LT(plan.forward_speed, 0.30);
    EXPECT_NEAR(plan.feedforward_wz, 0.20, 1e-9);
    EXPECT_NEAR(plan.forward_speed / plan.radius, plan.feedforward_wz, 1e-9);
}

TEST(ParkingSCurve, ReturnsReferenceHeadingToZero) {
    const ParkingSCurvePlan plan = makeParkingSCurvePlan(0.40, 0.10, 0.15, 0.03, 0.35);

    ASSERT_TRUE(plan.curved);
    EXPECT_NEAR(parkingSCurveReferenceYaw(plan, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(std::abs(parkingSCurveReferenceYaw(plan, plan.total_length * 0.5)),
                plan.peak_yaw, 1e-9);
    EXPECT_NEAR(parkingSCurveReferenceYaw(plan, plan.total_length), 0.0, 1e-9);
    EXPECT_NEAR(parkingSCurveFeedforwardWz(plan, plan.total_length), 0.0, 1e-9);
}

TEST(ParkingSCurve, SymmetricArcsReachRequestedOffset) {
    const ParkingSCurvePlan plan = makeParkingSCurvePlan(0.40, -0.10, 0.15, 0.03, 0.35);

    ASSERT_TRUE(plan.curved);
    const double endpoint_x = 2.0 * plan.radius * std::sin(plan.peak_yaw);
    const double endpoint_y = plan.direction * 2.0 * plan.radius *
                              (1.0 - std::cos(plan.peak_yaw));
    EXPECT_NEAR(endpoint_x, plan.longitudinal, 1e-9);
    EXPECT_NEAR(endpoint_y, plan.lateral, 1e-9);
}

}  // namespace
}  // namespace follow_test
}  // namespace flow_end

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
