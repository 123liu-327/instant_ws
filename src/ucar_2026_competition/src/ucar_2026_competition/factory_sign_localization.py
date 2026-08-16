#!/usr/bin/env python3
"""Pure geometry and fusion logic for factory-sign localization.

The runtime controller owns ROS subscriptions and mission state.  This module
intentionally has no ROS imports so its rejection and locking rules can be
tested deterministically on the development machine.
"""

from __future__ import annotations

import math
from collections import defaultdict


def normalize_angle(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _finite(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def bbox_metrics(bbox, image_width, image_height, edge_margin_ratio=0.02):
    """Return normalized geometry for a four-point OCR box."""
    width = _finite(image_width)
    height = _finite(image_height)
    if width is None or height is None or width <= 1.0 or height <= 1.0:
        return None
    points = []
    for point in bbox or ():
        try:
            x = float(point[0])
            y = float(point[1])
        except (TypeError, ValueError, IndexError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        points.append((x, y))
    if len(points) != 4:
        return None
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    box_width = x1 - x0
    box_height = y1 - y0
    if box_width <= 0.0 or box_height <= 0.0:
        return None
    center_x = 0.5 * (x0 + x1)
    center_y = 0.5 * (y0 + y1)
    edge_margin = min(x0 / width, y0 / height,
                      (width - x1) / width, (height - y1) / height)
    center_error_x = abs(center_x / width - 0.5) * 2.0
    center_error_y = abs(center_y / height - 0.5) * 2.0
    return {
        "width_ratio": box_width / width,
        "height_ratio": box_height / height,
        "area_ratio": box_width * box_height / (width * height),
        "edge_margin_ratio": edge_margin,
        "complete": edge_margin >= max(0.0, float(edge_margin_ratio)),
        "center_x": center_x,
        "center_y": center_y,
        "center_error_x": center_error_x,
        "center_error_y": center_error_y,
    }


def bbox_passes_gate(metrics, min_width=0.09, min_height=0.06,
                     min_area=0.006):
    return bool(
        metrics and metrics.get("complete") and
        metrics["width_ratio"] >= float(min_width) and
        metrics["height_ratio"] >= float(min_height) and
        metrics["area_ratio"] >= float(min_area)
    )


def observation_gate(anchor_id, coverage_state, stationary, metrics,
                     min_width=0.09, min_height=0.06, min_area=0.006):
    """Allow coordinate evidence only at a stationary deliberate scan pose."""
    try:
        valid_anchor = int(anchor_id) > 0
    except (TypeError, ValueError):
        valid_anchor = False
    return bool(
        valid_anchor and
        str(coverage_state or "").strip().lower() in ("scanning", "covered") and
        stationary and
        bbox_passes_gate(metrics, min_width, min_height, min_area)
    )


def camera_bearing(center_x, image_width, bearing_sign=-1.0,
                   boresight_offset=0.0, fx=None, cx=None,
                   horizontal_fov=math.radians(70.0)):
    """Convert a horizontal pixel coordinate into a base-relative bearing."""
    image_width = float(image_width)
    center_x = float(center_x)
    focal = _finite(fx)
    principal = _finite(cx)
    calibrated = bool(focal is not None and focal > 1.0 and principal is not None)
    if not calibrated:
        focal = image_width / (2.0 * math.tan(0.5 * float(horizontal_fov)))
        principal = 0.5 * image_width
    pixel_angle = math.atan2(center_x - principal, focal)
    bearing = normalize_angle(
        float(boresight_offset) + float(bearing_sign) * pixel_angle)
    return bearing, calibrated


def scan_points_in_sector(ranges, angle_min, angle_increment, range_min,
                          range_max, target_bearing, half_angle,
                          laser_x=0.08, laser_y=0.0, laser_yaw=0.0):
    """Convert fresh laser ranges into base-frame points near a target ray."""
    points = []
    target_bearing = float(target_bearing)
    half_angle = abs(float(half_angle))
    laser_yaw = float(laser_yaw)
    for index, raw_range in enumerate(ranges or ()):
        distance = _finite(raw_range)
        if distance is None or distance <= max(0.0, float(range_min)):
            continue
        if float(range_max) > 0.0 and distance >= float(range_max):
            continue
        scan_angle = float(angle_min) + index * float(angle_increment)
        base_angle = normalize_angle(scan_angle + laser_yaw)
        if abs(normalize_angle(base_angle - target_bearing)) > half_angle:
            continue
        points.append((
            float(laser_x) + distance * math.cos(base_angle),
            float(laser_y) + distance * math.sin(base_angle),
        ))
    return points


def fit_wall_line(points, min_points=12, min_span=0.25,
                  max_residual=0.02):
    """Robustly fit the longest line support and reject compact cone clusters."""
    pts = [(float(x), float(y)) for x, y in points or ()
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
    threshold = max(1e-4, abs(float(max_residual)) * 1.5)
    best = []
    for index_i in range(len(indices) - 1):
        i = indices[index_i]
        ax, ay = pts[i]
        for j in indices[index_i + 1:]:
            bx, by = pts[j]
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            if length < abs(float(min_span)) * 0.5:
                continue
            nx, ny = -dy / length, dx / length
            support = [point for point in pts if abs(
                (point[0] - ax) * nx + (point[1] - ay) * ny) <= threshold]
            projections = [
                (point[0] - ax) * dx / length +
                (point[1] - ay) * dy / length for point in support]
            if (len(support) < min_points or not projections or
                    max(projections) - min(projections) < abs(float(min_span))):
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
        tx, ty = -tx, -ty
    residuals = [abs((point[0] - cx) * nx + (point[1] - cy) * ny)
                 for point in best]
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    projections = [(point[0] - cx) * tx + (point[1] - cy) * ty
                   for point in best]
    span = max(projections) - min(projections)
    distance = cx * nx + cy * ny
    if (span < abs(float(min_span)) or
            rms > abs(float(max_residual)) or distance <= 0.0):
        return None
    return {
        "distance": distance,
        "normal_angle": math.atan2(ny, nx),
        "normal": (nx, ny),
        "tangent": (tx, ty),
        "point": (cx, cy),
        "projection_min": min(projections),
        "projection_max": max(projections),
        "span": span,
        "residual": rms,
        "inliers": len(best),
    }


def ray_wall_intersection(origin, bearing, wall_fit, support_margin=0.10):
    """Intersect a camera ray with a fitted line, bounded by measured support."""
    if not wall_fit:
        return None
    ox, oy = float(origin[0]), float(origin[1])
    dx, dy = math.cos(float(bearing)), math.sin(float(bearing))
    nx, ny = wall_fit["normal"]
    denominator = nx * dx + ny * dy
    if denominator <= 1e-4:
        return None
    distance = (float(wall_fit["distance"]) - nx * ox - ny * oy) / denominator
    if distance <= 0.0:
        return None
    point = (ox + distance * dx, oy + distance * dy)
    cx, cy = wall_fit["point"]
    tx, ty = wall_fit["tangent"]
    projection = (point[0] - cx) * tx + (point[1] - cy) * ty
    margin = abs(float(support_margin))
    if (projection < wall_fit["projection_min"] - margin or
            projection > wall_fit["projection_max"] + margin):
        return None
    return point


def transform_point(point, pose):
    """Transform a base-frame point by a planar (x, y, yaw) pose."""
    px, py = float(point[0]), float(point[1])
    x, y, yaw = [float(value) for value in pose]
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (x + cosine * px - sine * py,
            y + sine * px + cosine * py)


def inverse_transform_point(point, pose):
    """Transform a world-frame point into the planar pose's local frame."""
    px, py = float(point[0]), float(point[1])
    x, y, yaw = [float(value) for value in pose]
    dx, dy = px - x, py - y
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (cosine * dx + sine * dy,
            -sine * dx + cosine * dy)


def transform_vector(vector, yaw):
    vx, vy = float(vector[0]), float(vector[1])
    cosine, sine = math.cos(float(yaw)), math.sin(float(yaw))
    return (cosine * vx - sine * vy, sine * vx + cosine * vy)


def build_room_walls(corners):
    """Build clockwise wall segments from UL, UR, LL, LR room corners."""
    if not isinstance(corners, (list, tuple)) or len(corners) != 4:
        return []
    ul, ur, ll, lr = [tuple(map(float, point[:2])) for point in corners]
    center = (sum(point[0] for point in (ul, ur, ll, lr)) / 4.0,
              sum(point[1] for point in (ul, ur, ll, lr)) / 4.0)
    raw = (("north", ul, ur), ("east", ur, lr),
           ("south", lr, ll), ("west", ll, ul))
    walls = []
    for name, start, end in raw:
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        normals = ((-dy / length, dx / length), (dy / length, -dx / length))
        midpoint = (0.5 * (start[0] + end[0]),
                    0.5 * (start[1] + end[1]))
        normal = max(normals, key=lambda item:
                     item[0] * (center[0] - midpoint[0]) +
                     item[1] * (center[1] - midpoint[1]))
        walls.append({"id": name, "start": start, "end": end,
                      "normal": normal})
    return walls


def _ray_segment_intersection(origin, direction, start, end):
    ox, oy = origin
    dx, dy = direction
    sx, sy = start
    ex, ey = end
    wx, wy = ex - sx, ey - sy
    determinant = dx * wy - dy * wx
    if abs(determinant) <= 1e-9:
        return None
    qx, qy = sx - ox, sy - oy
    ray_t = (qx * wy - qy * wx) / determinant
    segment_t = (qx * dy - qy * dx) / determinant
    if ray_t <= 0.0 or segment_t < 0.0 or segment_t > 1.0:
        return None
    return ray_t, segment_t


def map_wall_fallback(map_pose, base_bearing, walls, endpoint_margin=0.25,
                      corner_tie_distance=0.15):
    """Return a conservative fixed-map wall intersection or an ambiguity."""
    x, y, yaw = [float(value) for value in map_pose]
    direction_yaw = normalize_angle(yaw + float(base_bearing))
    direction = (math.cos(direction_yaw), math.sin(direction_yaw))
    hits = []
    for wall in walls or ():
        result = _ray_segment_intersection(
            (x, y), direction, wall["start"], wall["end"])
        if result is not None:
            hits.append((result[0], result[1], wall))
    hits.sort(key=lambda item: item[0])
    if not hits:
        return None
    ambiguous = len(hits) > 1 and abs(hits[1][0] - hits[0][0]) <= abs(
        float(corner_tie_distance))
    ray_t, segment_t, wall = hits[0]
    wall_length = math.hypot(
        wall["end"][0] - wall["start"][0],
        wall["end"][1] - wall["start"][1])
    endpoint_distance = min(segment_t, 1.0 - segment_t) * wall_length
    ambiguous = ambiguous or endpoint_distance < abs(float(endpoint_margin))
    point = (x + ray_t * direction[0], y + ray_t * direction[1])
    return {"point_map": point, "wall_id": wall["id"],
            "normal_map": wall["normal"], "ambiguous": ambiguous}


def observation_quality(metrics, category_score, source, wall_fit=None,
                        calibrated=False):
    if not metrics:
        return 0.0
    category_score = max(0.0, float(category_score or 0.0))
    category_quality = category_score / (1.0 + category_score)
    source_bonus = 0.8 if source == "lidar_wall" else 0.25
    calibration_bonus = 0.20 if calibrated else 0.0
    wall_bonus = 0.0
    if wall_fit:
        wall_bonus = min(1.0, float(wall_fit.get("span", 0.0)))
        wall_bonus += max(0.0, 1.0 - float(wall_fit.get("residual", 1.0)) / 0.02)
    return (
        7.0 * math.sqrt(max(0.0, metrics["area_ratio"])) +
        1.25 * max(0.0, 1.0 - metrics["center_error_x"]) +
        0.20 * category_quality + source_bonus + calibration_bonus +
        0.35 * wall_bonus
    )


def weighted_median(values):
    ordered = sorted((float(value), max(1e-6, float(weight)))
                     for value, weight in values)
    if not ordered:
        raise ValueError("weighted median needs at least one value")
    halfway = 0.5 * sum(weight for _value, weight in ordered)
    total = 0.0
    for value, weight in ordered:
        total += weight
        if total >= halfway:
            return value
    return ordered[-1][0]


class FactorySignFusion:
    """Keep per-category observations, best viewpoints and lock decisions."""

    def __init__(self, max_samples=24):
        self.max_samples = max(4, int(max_samples))
        self.samples = defaultdict(list)
        self.best_views = {}

    def add(self, sample):
        category = str(sample.get("category") or "").strip().lower()
        if not category:
            return None
        item = dict(sample)
        item["category"] = category
        quality = float(item.get("quality", 0.0))
        previous = self.best_views.get(category)
        if previous is None or quality > float(previous.get("quality", -1.0)):
            self.best_views[category] = dict(item)
        bucket = self.samples[category]
        bucket.append(item)
        del bucket[:-self.max_samples]
        return self.estimate(category)

    def estimate(self, category):
        bucket = list(self.samples.get(str(category).lower(), ()))
        if not bucket:
            return None
        consistent = self._largest_consistent_group(bucket)
        if not consistent:
            consistent = [max(bucket, key=lambda item: item.get("quality", 0.0))]
        point_x = weighted_median([
            (item["point_odom"][0], item.get("quality", 1.0))
            for item in consistent])
        point_y = weighted_median([
            (item["point_odom"][1], item.get("quality", 1.0))
            for item in consistent])
        uncertainty = max(math.hypot(
            item["point_odom"][0] - point_x,
            item["point_odom"][1] - point_y) for item in consistent)
        locked = self._high_quality_lock(consistent) or self._two_view_lock(consistent)
        representative = max(consistent, key=lambda item: item.get("quality", 0.0))
        fused_point_map = representative.get("point_map")
        map_pose = representative.get("map_pose")
        odom_pose = representative.get("odom_pose")
        if map_pose is not None and odom_pose is not None:
            fused_local = inverse_transform_point(
                (point_x, point_y), odom_pose)
            fused_map = transform_point(fused_local, map_pose)
            fused_point_map = {"x": fused_map[0], "y": fused_map[1]}
        confidence = min(0.99, 0.35 + 0.08 * len(consistent) +
                         0.06 * float(representative.get("quality", 0.0)))
        result = {
            "schema_version": 1,
            "category": str(category).lower(),
            "status": "locked" if locked else "provisional",
            "source": representative.get("source", "unknown"),
            "point_odom": {"x": point_x, "y": point_y},
            "point_base": representative.get("point_base"),
            "point_map": fused_point_map,
            "wall_id": representative.get("wall_id"),
            "wall_normal": representative.get("wall_normal"),
            "wall_tangent_coordinate": representative.get(
                "wall_tangent_coordinate"),
            "confidence": confidence,
            "uncertainty_m": uncertainty,
            "best_anchor": int(self.best_views[str(category).lower()].get(
                "anchor_id", 0)),
            "best_yaw": self.best_views[str(category).lower()].get("odom_yaw"),
            "observation_count": len(bucket),
            "consistent_observation_count": len(consistent),
            "stamp": representative.get("stamp"),
        }
        return result

    @staticmethod
    def _compatible(first, second):
        if first.get("wall_id") and second.get("wall_id"):
            if first["wall_id"] != second["wall_id"]:
                return False
        limit = 0.20 if (first.get("source") == "map_wall_fallback" or
                         second.get("source") == "map_wall_fallback") else 0.15
        return math.hypot(
            first["point_odom"][0] - second["point_odom"][0],
            first["point_odom"][1] - second["point_odom"][1]) <= limit

    def _largest_consistent_group(self, bucket):
        groups = []
        for seed in bucket:
            group = [item for item in bucket if self._compatible(seed, item)]
            groups.append(group)
        return max(groups, key=lambda group: (
            len(group), sum(item.get("quality", 0.0) for item in group)))

    @staticmethod
    def _high_quality_lock(group):
        high = [item for item in group if (
            item.get("source") == "lidar_wall" and
            item.get("metrics", {}).get("area_ratio", 0.0) >= 0.02 and
            item.get("metrics", {}).get("center_error_x", 1.0) <= 0.12 and
            item.get("wall_fit", {}).get("span", 0.0) >= 0.45 and
            item.get("wall_fit", {}).get("residual", 1.0) <= 0.012 and
            not item.get("ambiguous", False))]
        if len(high) < 2:
            return False
        return any(first.get("anchor_id") == second.get("anchor_id")
                   for index, first in enumerate(high)
                   for second in high[index + 1:])

    @staticmethod
    def _two_view_lock(group):
        if len(group) < 2:
            return False
        for index, first in enumerate(group):
            if first.get("ambiguous", False):
                continue
            for second in group[index + 1:]:
                if second.get("ambiguous", False):
                    continue
                yaw_gap = abs(normalize_angle(
                    float(first.get("odom_yaw", 0.0)) -
                    float(second.get("odom_yaw", 0.0))))
                if yaw_gap >= math.radians(8.0):
                    return True
        return False
