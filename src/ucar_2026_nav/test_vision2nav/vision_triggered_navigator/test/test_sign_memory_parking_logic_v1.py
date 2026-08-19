#!/usr/bin/env python3

import math
import unittest

from sign_memory_parking_logic_v1 import (
    alignment_reacquire_is_usable,
    alternate_cardinal_yaw,
    choose_lateral_dodge,
    fit_front_wall,
    front_obstacle_is_localized,
    lateral_velocity_for_image_error,
    local_forward_displacement,
    local_lateral_displacement,
    remembered_target_lateral_travel,
    nearest_cardinal_yaw,
    projected_lateral_offset,
    should_run_mid_recenter,
)


class SignMemoryParkingLogicTest(unittest.TestCase):
    def test_alignment_reacquire_accepts_visible_sign_near_cardinal(self):
        self.assertTrue(alignment_reacquire_is_usable(
            -0.58, math.radians(-110.0), 0.65, math.radians(25.0)))

    def test_alignment_reacquire_rejects_edge_or_skewed_view(self):
        self.assertFalse(alignment_reacquire_is_usable(
            -0.72, math.radians(-110.0), 0.65, math.radians(25.0)))
        self.assertFalse(alignment_reacquire_is_usable(
            -0.20, math.radians(-122.0), 0.65, math.radians(25.0)))

    def test_sign_on_right_moves_robot_right(self):
        self.assertLess(lateral_velocity_for_image_error(
            0.4, 0.2, 0.02, 0.1), 0.0)

    def test_sign_on_left_moves_robot_left(self):
        self.assertGreater(lateral_velocity_for_image_error(
            -0.4, 0.2, 0.02, 0.1), 0.0)

    def test_wall_fit_returns_right_turn_for_left_yaw(self):
        yaw_error = 0.18
        points = []
        for index in range(41):
            y = -0.5 + index * 0.025
            x = 0.62 / math.cos(yaw_error) + math.tan(yaw_error) * y
            points.append((x, y))
        fit = fit_front_wall(points)
        self.assertIsNotNone(fit)
        self.assertAlmostEqual(fit["heading_error"], -yaw_error, places=2)

    def test_short_cone_cluster_is_not_a_wall(self):
        points = [(0.35 + index * 0.002, -0.04 + index * 0.004)
                  for index in range(20)]
        self.assertIsNone(fit_front_wall(points))

    def test_body_frame_displacements(self):
        start = (1.0, 2.0, math.pi * 0.5)
        current = (0.8, 2.3, math.pi * 0.5)
        self.assertAlmostEqual(local_forward_displacement(
            start, current, math.pi * 0.5), 0.3, places=6)
        self.assertAlmostEqual(local_lateral_displacement(
            start, current, math.pi * 0.5), 0.2, places=6)

    def test_nearest_cardinal(self):
        self.assertAlmostEqual(nearest_cardinal_yaw(math.radians(82.0)),
                               math.pi * 0.5)

    def test_projected_sign_on_right_is_rightward(self):
        offset = projected_lateral_offset(
            0.7, 0.49, math.radians(70.0), 0.88)
        self.assertLess(offset, 0.0)
        self.assertAlmostEqual(offset, -0.211, places=2)

    def test_remembered_projection_matches_pixel_only_without_turn(self):
        travel = remembered_target_lateral_travel(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0,
            0.4, 0.50, math.radians(70.0), 0.88, -1.0)
        self.assertAlmostEqual(travel, projected_lateral_offset(
            0.4, 0.50, math.radians(70.0), 0.88), places=6)

    def test_remembered_projection_includes_cardinal_turn(self):
        travel = remembered_target_lateral_travel(
            (0.0, 0.0, math.pi),
            (0.0, 0.0, math.radians(150.0)),
            math.pi, 0.036, 0.564, math.radians(70.0), 0.88, -1.0)
        self.assertLess(travel, -0.25)
        self.assertGreater(travel, -0.40)

    def test_clockwise_primary_retries_counterclockwise(self):
        target, direction, correction = alternate_cardinal_yaw(
            math.radians(35.0), 0.0)
        self.assertLess(correction, 0.0)
        self.assertGreater(direction, 0.0)
        self.assertAlmostEqual(target, math.pi * 0.5)

    def test_counterclockwise_primary_retries_clockwise(self):
        target, direction, correction = alternate_cardinal_yaw(
            math.radians(55.0), math.pi * 0.5)
        self.assertGreater(correction, 0.0)
        self.assertLess(direction, 0.0)
        self.assertAlmostEqual(target, 0.0)

    def test_front_cone_is_distinguished_from_far_wall(self):
        self.assertTrue(front_obstacle_is_localized(
            0.14, 0.48, 0.20, 0.16, 0.10))

    def test_close_wall_is_not_treated_as_cone(self):
        self.assertFalse(front_obstacle_is_localized(
            0.14, 0.18, 0.20, 0.16, 0.10))

    def test_lateral_dodge_uses_more_open_side(self):
        self.assertEqual(choose_lateral_dodge(0.38, 0.24, 0.18), 1)
        self.assertEqual(choose_lateral_dodge(0.20, 0.35, 0.18), -1)
        self.assertEqual(choose_lateral_dodge(0.15, 0.16, 0.18), 0)

    def test_mid_recenter_runs_once_in_approach_window(self):
        self.assertTrue(should_run_mid_recenter(
            False, 0.47, 0.48, 0.20))
        self.assertFalse(should_run_mid_recenter(
            False, 0.62, 0.48, 0.20))
        self.assertFalse(should_run_mid_recenter(
            True, 0.47, 0.48, 0.20))

    def test_mid_recenter_does_not_start_at_final_wall_stop(self):
        self.assertFalse(should_run_mid_recenter(
            False, 0.23, 0.48, 0.20))


if __name__ == "__main__":
    unittest.main()
