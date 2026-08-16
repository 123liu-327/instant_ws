import math

from ucar_2026_competition.factory_sign_localization import (
    FactorySignFusion,
    bbox_metrics,
    bbox_passes_gate,
    build_room_walls,
    camera_bearing,
    fit_wall_line,
    map_wall_fallback,
    observation_gate,
    ray_wall_intersection,
    scan_points_in_sector,
)


def test_bbox_gate_accepts_complete_large_box_and_rejects_edge_box():
    metrics = bbox_metrics(
        [[220, 180], [420, 180], [420, 260], [220, 260]], 640, 480)
    assert metrics["complete"]
    assert bbox_passes_gate(metrics)

    partial = bbox_metrics(
        [[0, 180], [180, 180], [180, 260], [0, 260]], 640, 480)
    assert not partial["complete"]
    assert not bbox_passes_gate(partial)


def test_observation_gate_never_accepts_transit_or_motion():
    metrics = bbox_metrics(
        [[220, 180], [420, 180], [420, 260], [220, 260]], 640, 480)
    assert observation_gate(3, "scanning", True, metrics)
    assert not observation_gate(3, "navigating", True, metrics)
    assert not observation_gate(3, "scanning", False, metrics)
    assert not observation_gate(0, "scanning", True, metrics)


def test_camera_bearing_uses_calibration_then_fov_fallback():
    bearing, calibrated = camera_bearing(320, 640, fx=500, cx=320)
    assert calibrated
    assert abs(bearing) < 1e-9

    bearing, calibrated = camera_bearing(
        480, 640, bearing_sign=-1.0, horizontal_fov=math.radians(70))
    assert not calibrated
    assert bearing < 0.0


def test_scan_sector_and_wall_fit_reject_compact_cone_cluster():
    wall = [(1.0 + 0.002 * math.sin(index), -0.45 + index * 0.04)
            for index in range(24)]
    cone = [(0.55 + 0.005 * index, -0.03 + 0.008 * index)
            for index in range(7)]
    fit = fit_wall_line(wall + cone, min_points=12, min_span=0.25,
                        max_residual=0.02)
    assert fit is not None
    assert fit["span"] > 0.7
    assert fit["inliers"] >= 20
    assert abs(fit["distance"] - 1.0) < 0.03


def test_ray_intersects_measured_wall_support():
    wall = [(1.0, -0.5 + index * 0.05) for index in range(21)]
    fit = fit_wall_line(wall, min_points=12, min_span=0.25,
                        max_residual=0.02)
    point = ray_wall_intersection((0.0, 0.0), 0.0, fit)
    assert point is not None
    assert abs(point[0] - 1.0) < 0.01
    assert abs(point[1]) < 0.01


def test_map_fallback_marks_near_corner_ambiguous():
    walls = build_room_walls([
        [-2.0, -1.0], [2.0, -1.0], [-2.0, -3.0], [2.0, -3.0]])
    result = map_wall_fallback(
        (0.0, -2.0, 0.0), 0.0, walls, endpoint_margin=0.25)
    assert result["wall_id"] == "east"
    assert not result["ambiguous"]

    corner = map_wall_fallback(
        (0.0, -2.0, math.radians(26.565)), 0.0, walls,
        endpoint_margin=0.30)
    assert corner is not None
    assert corner["ambiguous"]


def _sample(yaw, x=1.0, source="lidar_wall", quality=3.0,
            area=0.025, center_error=0.05, anchor=2):
    return {
        "category": "food",
        "source": source,
        "point_odom": (x, 0.1),
        "point_base": {"x": 1.0, "y": 0.1},
        "point_map": {"x": x, "y": 0.1},
        "wall_id": "east",
        "quality": quality,
        "anchor_id": anchor,
        "odom_yaw": yaw,
        "metrics": {"area_ratio": area, "center_error_x": center_error},
        "wall_fit": {"span": 0.55, "residual": 0.008},
        "ambiguous": False,
        "stamp": 100.0,
    }


def test_high_quality_same_dwell_locks_after_two_frames():
    fusion = FactorySignFusion()
    assert fusion.add(_sample(0.0))["status"] == "provisional"
    estimate = fusion.add(_sample(math.radians(1.0), x=1.01))
    assert estimate["status"] == "locked"
    assert estimate["best_anchor"] == 2
    assert estimate["uncertainty_m"] <= 0.02


def test_normal_quality_requires_distinct_view_and_keeps_best_anchor():
    fusion = FactorySignFusion()
    first = _sample(0.0, quality=2.0, area=0.012, anchor=3)
    second = _sample(math.radians(10), x=1.04, quality=4.0,
                     area=0.012, anchor=5)
    assert fusion.add(first)["status"] == "provisional"
    estimate = fusion.add(second)
    assert estimate["status"] == "locked"
    assert estimate["best_anchor"] == 5


def test_low_quality_observation_does_not_replace_best_view():
    fusion = FactorySignFusion()
    fusion.add(_sample(0.0, quality=5.0, area=0.012, anchor=6))
    estimate = fusion.add(_sample(
        math.radians(10), x=1.02, quality=1.0, area=0.012, anchor=7))
    assert estimate["best_anchor"] == 6


def test_map_jump_does_not_move_odom_fusion():
    fusion = FactorySignFusion()
    first = _sample(0.0, area=0.012)
    second = _sample(math.radians(10), x=1.03, area=0.012)
    first["point_map"] = {"x": 1.0, "y": 0.1}
    second["point_map"] = {"x": 2.0, "y": 1.1}
    fusion.add(first)
    estimate = fusion.add(second)
    assert estimate["status"] == "locked"
    assert 1.0 <= estimate["point_odom"]["x"] <= 1.03
    assert estimate["uncertainty_m"] < 0.05


def test_fused_odom_point_is_reprojected_with_consistent_pose_pair():
    fusion = FactorySignFusion()
    first = _sample(0.0, area=0.012)
    second = _sample(math.radians(10), x=1.02, area=0.012)
    for sample in (first, second):
        sample["odom_pose"] = (0.0, 0.0, 0.0)
        sample["map_pose"] = (2.0, -1.0, 0.0)
    fusion.add(first)
    estimate = fusion.add(second)
    assert 3.0 <= estimate["point_map"]["x"] <= 3.02
    assert abs(estimate["point_map"]["y"] + 0.9) < 1e-6


def test_map_fallback_does_not_lock_from_repeated_same_heading():
    fusion = FactorySignFusion()
    fusion.add(_sample(0.0, source="map_wall_fallback", area=0.012))
    estimate = fusion.add(_sample(
        math.radians(2), x=1.03, source="map_wall_fallback", area=0.012))
    assert estimate["status"] == "provisional"


def test_scan_points_respect_target_sector():
    ranges = [1.0] * 9
    points = scan_points_in_sector(
        ranges, -0.4, 0.1, 0.08, 10.0, 0.0, 0.11,
        laser_x=0.08)
    assert len(points) == 3
    assert all(point[0] > 1.0 for point in points)
