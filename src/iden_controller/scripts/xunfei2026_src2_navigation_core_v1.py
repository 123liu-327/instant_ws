#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS-independent navigation primitives ported from ``~/src2``.

This file deliberately contains only the geometry and controller math needed
by the new dual-stage room manager.  Keeping it separate makes the new path
auditable and leaves every legacy competition script untouched.
"""

from __future__ import division

import math


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def build_quadrilateral_walls(corners):
    if len(corners) != 4:
        raise ValueError("vision_rect_corners must contain four points")
    points = [(float(point[0]), float(point[1])) for point in corners]
    top_left, top_right, bottom_left, bottom_right = points
    centroid = (
        sum(point[0] for point in points) / 4.0,
        sum(point[1] for point in points) / 4.0,
    )
    segments = [
        ("left", top_left, bottom_left),
        ("right", top_right, bottom_right),
        ("bottom", bottom_left, bottom_right),
        ("top", top_left, top_right),
    ]
    walls = []
    for name, start, end in segments:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            raise ValueError("wall {} has zero length".format(name))
        normal = (-dy / length, dx / length)
        midpoint = ((start[0] + end[0]) * 0.5,
                    (start[1] + end[1]) * 0.5)
        toward_center = (centroid[0] - midpoint[0],
                         centroid[1] - midpoint[1])
        if normal[0] * toward_center[0] + normal[1] * toward_center[1] < 0.0:
            normal = (-normal[0], -normal[1])
        walls.append((name, start, end, normal))
    return walls


def ray_segment_intersection(origin, direction, start, end):
    ox, oy = [float(value) for value in origin]
    dx, dy = [float(value) for value in direction]
    ax, ay = [float(value) for value in start]
    bx, by = [float(value) for value in end]
    vx = bx - ax
    vy = by - ay
    denominator = -vx * dy + vy * dx
    if abs(denominator) < 1.0e-9:
        return None
    wx = ox - ax
    wy = oy - ay
    ray_t = (vx * wy - vy * wx) / denominator
    segment_u = (-wx * dy + wy * dx) / denominator
    if ray_t <= 1.0e-9 or segment_u < -1.0e-6 or segment_u > 1.0 + 1.0e-6:
        return None
    return ray_t


def parking_goal_from_wall(wall_point, inward_normal, offset,
                           normal_offset=0.0, tangent_offset=0.0):
    ix, iy = [float(value) for value in wall_point]
    nx, ny = [float(value) for value in inward_normal]
    length = math.hypot(nx, ny)
    if length <= 1.0e-9:
        raise ValueError("inward normal must be non-zero")
    nx /= length
    ny /= length
    tx, ty = -ny, nx
    normal_distance = float(offset) + float(normal_offset)
    return (
        ix + nx * normal_distance + tx * float(tangent_offset),
        iy + ny * normal_distance + ty * float(tangent_offset),
        math.atan2(-ny, -nx),
    )


def bounded_axis_command(error, tolerance, gain, maximum, minimum=0.0):
    error = float(error)
    if abs(error) <= abs(float(tolerance)):
        return 0.0
    magnitude = min(abs(float(maximum)), abs(float(gain)) * abs(error))
    magnitude = max(min(abs(float(minimum)), abs(float(maximum))), magnitude)
    return math.copysign(magnitude, error)


def wall_frame_docking_command(normal_error, tangent_error, yaw_error,
                               normal_tolerance, tangent_tolerance,
                               yaw_tolerance, max_x, max_y, max_yaw,
                               min_yaw=0.15):
    """src2 three-phase docking: rotate, translate along wall, then advance."""
    if abs(float(yaw_error)) > abs(float(yaw_tolerance)):
        return (0.0, 0.0, bounded_axis_command(
            yaw_error, yaw_tolerance, 1.5, max_yaw, min_yaw))
    if abs(float(tangent_error)) > abs(float(tangent_tolerance)):
        return (0.0, bounded_axis_command(
            tangent_error, tangent_tolerance, 1.0, max_y, 0.025), 0.0)
    return (bounded_axis_command(
        normal_error, normal_tolerance, 0.8, max_x, 0.03), 0.0, 0.0)


def docking_within_tolerance(errors, normal_tolerance,
                             tangent_tolerance, yaw_tolerance):
    normal, tangent, yaw = [abs(float(value)) for value in errors]
    return (normal <= abs(float(normal_tolerance)) and
            tangent <= abs(float(tangent_tolerance)) and
            yaw <= abs(float(yaw_tolerance)))


def fit_wall_line(points, min_points=12, min_span=0.25,
                  max_residual=0.015):
    """Robust deterministic line fit copied from src2's parking core."""
    pts = [(float(x), float(y)) for x, y in points
           if math.isfinite(float(x)) and math.isfinite(float(y))]
    min_points = max(2, int(min_points))
    if len(pts) < min_points:
        return None
    if len(pts) > 80:
        step = float(len(pts) - 1) / 79.0
        pts = [pts[int(round(index * step))] for index in range(80)]
    hypothesis_step = max(1, int(math.ceil(len(pts) / 24.0)))
    indices = list(range(0, len(pts), hypothesis_step))
    if indices[-1] != len(pts) - 1:
        indices.append(len(pts) - 1)
    threshold = max(1.0e-4, float(max_residual) * 1.5)
    best = []
    for offset, index_i in enumerate(indices[:-1]):
        ax, ay = pts[index_i]
        for index_j in indices[offset + 1:]:
            bx, by = pts[index_j]
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            if length < float(min_span) * 0.5:
                continue
            nx, ny = -dy / length, dx / length
            support = [
                point for point in pts
                if abs((point[0] - ax) * nx +
                       (point[1] - ay) * ny) <= threshold
            ]
            if len(support) < min_points:
                continue
            projections = [
                (point[0] - ax) * dx / length +
                (point[1] - ay) * dy / length for point in support
            ]
            if max(projections) - min(projections) < abs(float(min_span)):
                continue
            if len(support) > len(best):
                best = support
    if len(best) < min_points:
        return None

    cx = sum(point[0] for point in best) / len(best)
    cy = sum(point[1] for point in best) / len(best)
    sxx = sum((point[0] - cx) ** 2 for point in best)
    syy = sum((point[1] - cy) ** 2 for point in best)
    sxy = sum((point[0] - cx) * (point[1] - cy) for point in best)
    tangent_angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    tx, ty = math.cos(tangent_angle), math.sin(tangent_angle)
    nx, ny = -ty, tx
    if nx * cx + ny * cy < 0.0:
        nx, ny = -nx, -ny
    residuals = [
        abs((point[0] - cx) * nx + (point[1] - cy) * ny)
        for point in best
    ]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    projections = [
        (point[0] - cx) * tx + (point[1] - cy) * ty for point in best
    ]
    span = max(projections) - min(projections)
    distance = cx * nx + cy * ny
    if (span < abs(float(min_span)) or
            rms > abs(float(max_residual)) or distance <= 0.0):
        return None
    return {
        "distance": distance,
        "normal_angle": math.atan2(ny, nx),
        "normal": (nx, ny),
        "span": span,
        "residual": rms,
        "inliers": len(best),
    }


def wall_fit_matches_expected(fit, expected_normal_angle, maximum_error):
    return bool(fit) and abs(normalize_angle(
        float(fit["normal_angle"]) - float(expected_normal_angle))) <= abs(
            float(maximum_error))


def wall_fit_is_continuous(current, previous, maximum_distance_jump,
                           maximum_normal_jump):
    return bool(current and previous) and (
        abs(float(current["distance"]) - float(previous["distance"])) <=
        abs(float(maximum_distance_jump)) and
        abs(normalize_angle(float(current["normal_angle"]) -
                            float(previous["normal_angle"]))) <=
        abs(float(maximum_normal_jump)))


def parking_footprint_margins(pose, wall_point, inward_normal,
                              box_width, box_depth,
                              footprint_half_length, footprint_half_width,
                              margin=0.0):
    px, py, yaw = [float(value) for value in pose]
    wx, wy = [float(value) for value in wall_point]
    nx, ny = [float(value) for value in inward_normal]
    length = math.hypot(nx, ny)
    if length <= 1.0e-9:
        return {"inside": False, "error": "zero inward normal", "corners": []}
    nx, ny = nx / length, ny / length
    tx, ty = -ny, nx
    half_length = abs(float(footprint_half_length))
    half_width = abs(float(footprint_half_width))
    width_limit = abs(float(box_width)) * 0.5 - max(0.0, float(margin))
    depth_min = max(0.0, float(margin))
    depth_max = abs(float(box_depth)) - max(0.0, float(margin))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    corners = []
    for local_x in (-half_length, half_length):
        for local_y in (-half_width, half_width):
            x = px + local_x * cosine - local_y * sine
            y = py + local_x * sine + local_y * cosine
            dx, dy = x - wx, y - wy
            normal_distance = dx * nx + dy * ny
            tangent_distance = dx * tx + dy * ty
            corners.append({
                "x": x, "y": y,
                "normal": normal_distance,
                "tangent": tangent_distance,
                "near_margin": normal_distance - depth_min,
                "far_margin": depth_max - normal_distance,
                "side_margin": width_limit - abs(tangent_distance),
            })
    normal_min = min(item["normal"] for item in corners)
    normal_max = max(item["normal"] for item in corners)
    tangent_min = min(item["tangent"] for item in corners)
    tangent_max = max(item["tangent"] for item in corners)
    tangent_abs_max = max(abs(item["tangent"]) for item in corners)
    near_margin = normal_min - depth_min
    far_margin = depth_max - normal_max
    side_margin = width_limit - tangent_abs_max
    return {
        "inside": near_margin >= 0.0 and far_margin >= 0.0 and side_margin >= 0.0,
        "error": "",
        "normal_min": normal_min,
        "normal_max": normal_max,
        "tangent_min": tangent_min,
        "tangent_max": tangent_max,
        "tangent_abs_max": tangent_abs_max,
        "normal_error": (normal_min + normal_max) * 0.5 - abs(float(box_depth)) * 0.5,
        "tangent_error": (tangent_min + tangent_max) * 0.5,
        "near_margin": near_margin,
        "far_margin": far_margin,
        "side_margin": side_margin,
        "corners": corners,
    }
