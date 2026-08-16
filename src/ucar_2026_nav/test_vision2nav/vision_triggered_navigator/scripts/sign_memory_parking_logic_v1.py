#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pure geometry helpers for sign-memory parking.

The ROS navigator owns the state machine.  This module deliberately has no
dependency on the previous parking implementation so direction and wall
alignment can be unit-tested in isolation.
"""

import math


def clamp(value, lower, upper):
    return max(float(lower), min(float(upper), float(value)))


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def nearest_cardinal_yaw(yaw):
    quarter_turn = math.pi * 0.5
    return normalize_angle(round(float(yaw) / quarter_turn) * quarter_turn)


def alternate_cardinal_yaw(initial_yaw, primary_yaw):
    """Choose the adjacent cardinal by reversing the primary turn direction."""
    correction = normalize_angle(float(primary_yaw) - float(initial_yaw))
    reverse_direction = -1.0 if correction > 0.0 else 1.0
    target = normalize_angle(
        float(primary_yaw) + reverse_direction * math.pi * 0.5)
    return target, reverse_direction, correction


def lateral_velocity_for_image_error(error, gain, minimum, maximum):
    """Return base_link lateral velocity for a normalized image-x error.

    Positive image error means the sign is to the image right.  A rightward
    chassis translation (negative base_link y) moves the sign back toward the
    image center, hence the intentional minus sign.
    """
    error = float(error)
    if abs(error) <= 1e-9:
        return 0.0
    magnitude = clamp(abs(error) * abs(float(gain)),
                      abs(float(minimum)), abs(float(maximum)))
    return -math.copysign(magnitude, error)


def projected_lateral_offset(error, wall_distance, horizontal_fov,
                             projection_gain=1.0):
    """Project normalized image-x error onto a front wall.

    For a pinhole camera, normalized image displacement multiplied by
    ``tan(horizontal_fov/2)`` is the ray's lateral/forward ratio.  The minus
    sign preserves the chassis direction rule used above.
    """
    error = clamp(float(error), -1.5, 1.5)
    wall_distance = max(0.05, float(wall_distance))
    half_fov = clamp(abs(float(horizontal_fov)) * 0.5,
                     math.radians(10.0), math.radians(70.0))
    return (-float(projection_gain) * wall_distance * error *
            math.tan(half_fov))


def front_obstacle_is_localized(front_min, front_median, wall_stop,
                                hard_stop, separation_margin):
    """Distinguish a narrow foreground obstacle from the parking wall.

    A cone close to the nose lowers the minimum range but leaves the median of
    the front sector substantially farther away.  A real wall makes both
    measurements short.  Unknown data is deliberately not classified as a
    cone so the caller keeps the conservative stop behaviour.
    """
    if front_min is None or front_median is None:
        return False
    front_min = float(front_min)
    front_median = float(front_median)
    if not math.isfinite(front_min) or not math.isfinite(front_median):
        return False
    return (front_min <= float(hard_stop) and
            front_median > float(wall_stop) + float(separation_margin) and
            front_median - front_min >= float(separation_margin))


def choose_lateral_dodge(left_clearance, right_clearance, minimum_clearance):
    """Choose the safer short lateral dodge: +1 left, -1 right, 0 blocked."""
    left = (float("inf") if left_clearance is None else
            float(left_clearance))
    right = (float("inf") if right_clearance is None else
             float(right_clearance))
    threshold = float(minimum_clearance)
    left_ok = math.isfinite(left) and left > threshold
    right_ok = math.isfinite(right) and right > threshold
    if not left_ok and not right_ok:
        return 0
    if left_ok and not right_ok:
        return 1
    if right_ok and not left_ok:
        return -1
    return 1 if left >= right else -1


def _least_squares_x_from_y(points):
    count = float(len(points))
    mean_y = sum(point[1] for point in points) / count
    mean_x = sum(point[0] for point in points) / count
    denominator = sum((point[1] - mean_y) ** 2 for point in points)
    if denominator <= 1e-9:
        return None
    slope = sum((point[1] - mean_y) * (point[0] - mean_x)
                for point in points) / denominator
    intercept = mean_x - slope * mean_y
    return slope, intercept


def fit_front_wall(points, min_points=12, min_span=0.28,
                   inlier_threshold=0.025, max_residual=0.025,
                   max_abs_heading_deg=45.0):
    """Robustly fit a long front wall as ``x = slope*y + intercept``.

    Deterministic pair sampling is used instead of a random RANSAC seed.  A
    short cone cluster cannot win because candidates need both enough inliers
    and a substantial lateral span.
    """
    valid = [
        (float(x), float(y)) for x, y in points
        if math.isfinite(float(x)) and math.isfinite(float(y))
        and 0.12 <= float(x) <= 1.40 and abs(float(y)) <= 0.85
    ]
    if len(valid) < int(min_points):
        return None
    valid.sort(key=lambda point: point[1])

    sample_count = min(28, len(valid))
    if sample_count == len(valid):
        sampled = valid
    else:
        sampled = [valid[int(round(index * (len(valid) - 1) /
                                  float(sample_count - 1)))]
                   for index in range(sample_count)]

    max_slope = math.tan(math.radians(abs(float(max_abs_heading_deg))))
    best = None
    for first_index in range(len(sampled) - 1):
        x1, y1 = sampled[first_index]
        for second_index in range(first_index + 1, len(sampled)):
            x2, y2 = sampled[second_index]
            delta_y = y2 - y1
            if abs(delta_y) < float(min_span):
                continue
            slope = (x2 - x1) / delta_y
            if abs(slope) > max_slope:
                continue
            intercept = x1 - slope * y1
            scale = math.sqrt(1.0 + slope * slope)
            inliers = [
                point for point in valid
                if abs(point[0] - (slope * point[1] + intercept)) / scale
                <= float(inlier_threshold)
            ]
            if len(inliers) < int(min_points):
                continue
            span = max(point[1] for point in inliers) - min(
                point[1] for point in inliers)
            if span < float(min_span):
                continue
            residual = sum(
                abs(point[0] - (slope * point[1] + intercept)) / scale
                for point in inliers) / float(len(inliers))
            score = span * 3.0 + len(inliers) * 0.01 - residual * 8.0
            if best is None or score > best[0]:
                best = (score, inliers)

    if best is None:
        return None
    refined = _least_squares_x_from_y(best[1])
    if refined is None:
        return None
    slope, intercept = refined
    scale = math.sqrt(1.0 + slope * slope)
    residuals = [
        abs(point[0] - (slope * point[1] + intercept)) / scale
        for point in best[1]
    ]
    mean_residual = sum(residuals) / float(len(residuals))
    span = max(point[1] for point in best[1]) - min(
        point[1] for point in best[1])
    if (mean_residual > float(max_residual) or
            abs(slope) > max_slope or intercept <= 0.0):
        return None

    # If x grows toward the robot's left, the chassis is yawed left relative
    # to the wall normal and must rotate right, hence -atan(slope).
    return {
        "heading_error": -math.atan(slope),
        "distance": intercept,
        "span": span,
        "residual": mean_residual,
        "inlier_count": len(best[1]),
        "slope": slope,
    }


def local_lateral_displacement(start_pose, current_pose, reference_yaw):
    dx = float(current_pose[0]) - float(start_pose[0])
    dy = float(current_pose[1]) - float(start_pose[1])
    return -math.sin(float(reference_yaw)) * dx + math.cos(
        float(reference_yaw)) * dy


def local_forward_displacement(start_pose, current_pose, reference_yaw):
    dx = float(current_pose[0]) - float(start_pose[0])
    dy = float(current_pose[1]) - float(start_pose[1])
    return math.cos(float(reference_yaw)) * dx + math.sin(
        float(reference_yaw)) * dy
