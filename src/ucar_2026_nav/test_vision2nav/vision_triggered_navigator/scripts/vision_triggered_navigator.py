#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf
import json
import math
import os
import threading
import sys

from dynamic_reconfigure.msg import BoolParameter, DoubleParameter
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger, TriggerResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from navigator_logic import (
    build_quadrilateral_walls,
    coverage_anchor_order,
    coverage_motion_is_rotation_stall,
    coverage_position_needs_yaw_alignment,
    coverage_timeout_decision,
    docking_command,
    docking_pose_errors,
    docking_within_tolerance,
    fit_wall_line,
    lidar_base_wall_distance,
    lidar_requires_stop,
    center_step_angle,
    costmap_value_at,
    exact_observation_target,
    footprint_max_cost,
    latch_trigger,
    normalize_angle,
    parking_footprint_margins,
    parking_goal_from_wall,
    ray_segment_intersection,
    scan_dwell_deadline,
    sensor_is_fresh,
    should_skip_coverage_anchor,
    split_scan_angle,
    staging_pose_reached,
    staging_motion_is_rotation_stall,
    target_sample_is_fresh,
    wall_fit_matches_expected,
    wall_fit_is_continuous,
    wall_frame_docking_command,
    wall_normal_distance,
)


def euler_to_quaternion(yaw):
    """将 yaw 角转换为 geometry_msgs/Quaternion"""
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def quaternion_to_yaw(q):
    """从四元数提取 yaw 角，支持 geometry_msgs/Quaternion 或 (x, y, z, w) 列表/元组"""
    if isinstance(q, (list, tuple)):
        x, y, z, w = q
    else:
        x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def coverage_fallback_candidates(x, y, robot_x, robot_y, radii):
    """Generate nearby anchor alternatives, preferring the robot-facing side."""
    toward_robot = math.atan2(robot_y - y, robot_x - x)
    angle_offsets = (
        0.0, math.pi / 4.0, -math.pi / 4.0,
        math.pi / 2.0, -math.pi / 2.0,
        3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0, math.pi,
    )
    candidates = []
    for radius in radii:
        radius = abs(float(radius))
        if radius <= 1e-6:
            continue
        for offset in angle_offsets:
            angle = toward_robot + offset
            candidates.append((
                float(x) + radius * math.cos(angle),
                float(y) + radius * math.sin(angle),
            ))
    return candidates


def select_corner_aware_wall(origin, direction, walls, tie_distance):
    """Select the front-facing wall when a ray reaches two walls at a corner."""
    candidates = []
    px, py = origin
    dx, dy = direction
    for wall_name, start, end, inward_normal in walls:
        distance = ray_segment_intersection(origin, direction, start, end)
        endpoint_proxy = False
        if distance is None:
            endpoint_options = []
            for endpoint in (start, end):
                rel_x = float(endpoint[0]) - px
                rel_y = float(endpoint[1]) - py
                along = rel_x * dx + rel_y * dy
                perpendicular = abs(rel_x * dy - rel_y * dx)
                if (along >= 0.0 and
                        perpendicular <= max(0.0, float(tie_distance))):
                    endpoint_options.append((perpendicular, along, endpoint))
            if not endpoint_options:
                continue
            _miss, distance, point = min(endpoint_options)
            endpoint_proxy = True
        else:
            point = (px + distance * dx, py + distance * dy)
        outward = (-inward_normal[0], -inward_normal[1])
        facing = max(0.0, dx * outward[0] + dy * outward[1])
        candidates.append({
            "name": wall_name,
            "start": start,
            "end": end,
            "normal": inward_normal,
            "distance": distance,
            "point": point,
            "facing": facing,
            "endpoint_proxy": endpoint_proxy,
        })
    if not candidates:
        return None
    nearest = min(item["distance"] for item in candidates)
    tied = [
        item for item in candidates
        if item["distance"] <= nearest + max(0.0, float(tie_distance))
    ]
    return max(tied, key=lambda item: (item["facing"], -item["distance"]))


def clamp_point_to_wall_segment(point, start, end, endpoint_margin):
    """Clamp a wall point away from corners so a physical parking box can fit."""
    vx = float(end[0]) - float(start[0])
    vy = float(end[1]) - float(start[1])
    length = math.hypot(vx, vy)
    if length <= 1e-9:
        return float(point[0]), float(point[1]), False
    projection = (
        (float(point[0]) - float(start[0])) * vx +
        (float(point[1]) - float(start[1])) * vy
    ) / (length * length)
    fraction_margin = min(
        max(0.0, float(endpoint_margin)) / length,
        max(0.0, 0.5 - 1e-3),
    )
    clamped = min(max(projection, fraction_margin), 1.0 - fraction_margin)
    return (
        float(start[0]) + clamped * vx,
        float(start[1]) + clamped * vy,
        abs(clamped - projection) > 1e-6,
    )


def target_bearing_yaw(robot_yaw, normalized_error, horizontal_fov,
                       bearing_sign, boresight_offset=0.0):
    """Convert a normalized image x error into the target map bearing."""
    error = max(-1.0, min(1.0, float(normalized_error)))
    bearing = (
        float(robot_yaw) + float(boresight_offset) +
        float(bearing_sign) * error * 0.5 * abs(float(horizontal_fov))
    )
    return normalize_angle(bearing)


class VisionTriggeredNavigator(object):
    """
    三阶段导航节点：
      1. 巡航（依次访问预标点，costmap 实时判断可行性，跳点，到站自转）
      2. 视觉触发（键盘或视觉话题触发，射线与长方形围墙求交，内法向偏移 0.4m）
      3. 导航到结束点
    """

    def __init__(self):
        rospy.init_node("vision_triggered_navigator")

        # ---------- 参数读取 ----------
        # 参考坐标系
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # 视觉触发长方形围墙（4 个角点，带手标误差，程序取 min/max 包围盒）
        # 目标点位于墙内侧 vision_offset 处，车头垂直指向墙外
        self.vision_rect_corners = rospy.get_param(
            "~vision_rect_corners",
            [[-2.2311, -1.2505],
             [2.8000, -1.1940],
             [-2.2197, -3.2746],
             [2.7739, -3.2186]])
        self._build_rect_bounds()

        # 视觉目标安全偏移距离
        self.vision_offset = rospy.get_param("~vision_offset", 0.4)

        # 触发模式："keyboard" 或 "vision"
        self.trigger_mode = rospy.get_param("~trigger_mode", "keyboard")
        self.vision_topic = rospy.get_param("~vision_topic", "/vision/detected")
        self.status_topic = rospy.get_param(
            "~status_topic", "/vision_triggered_navigator/status")
        self.trigger_service_name = rospy.get_param(
            "~trigger_service", "/vision_triggered_navigator/trigger_target")
        self.start_paused = bool(rospy.get_param("~start_paused", False))
        self.start_navigation_service_name = rospy.get_param(
            "~start_navigation_service",
            "/vision_triggered_navigator/start_navigation",
        )
        self.coverage_observation_topic = rospy.get_param(
            "~coverage_observation_topic",
            "/vision_triggered_navigator/coverage_observation",
        )
        self.preferred_coverage_anchor = max(
            0, int(rospy.get_param("~preferred_coverage_anchor", 0)))
        self.navigation_start_event = threading.Event()
        if not self.start_paused:
            self.navigation_start_event.set()
        self.navigate_to_end_after_trigger = rospy.get_param(
            "~navigate_to_end_after_trigger", True)

        # 任务2专用的覆盖优先模式。默认关闭，保证原独立导航行为不变。
        self.coverage_search_mode = rospy.get_param("~coverage_search_mode", False)
        self.target_topic = rospy.get_param("~target_topic", "/vision/target")
        legacy_coverage_timeout = float(rospy.get_param(
            "~coverage_goal_timeout_sec", 25.0))
        self.coverage_goal_soft_timeout = max(0.1, float(rospy.get_param(
            "~coverage_goal_soft_timeout_sec", legacy_coverage_timeout)))
        self.coverage_goal_hard_timeout = max(
            self.coverage_goal_soft_timeout,
            float(rospy.get_param("~coverage_goal_hard_timeout_sec", 40.0)))
        self.coverage_goal_progress_window = max(0.5, float(rospy.get_param(
            "~coverage_goal_progress_window_sec", 5.0)))
        self.coverage_goal_min_progress = max(0.0, float(rospy.get_param(
            "~coverage_goal_min_progress", 0.03)))
        self.coverage_rotation_watchdog_window = max(1.0, float(rospy.get_param(
            "~coverage_rotation_watchdog_window_sec", 5.0)))
        self.coverage_rotation_min_progress = max(0.0, float(rospy.get_param(
            "~coverage_rotation_min_progress", 0.03)))
        self.coverage_rotation_max_yaw = math.radians(abs(float(rospy.get_param(
            "~coverage_rotation_max_yaw_deg", 90.0))))
        self.coverage_goal_retry_count = max(0, int(rospy.get_param(
            "~coverage_goal_retry_count", 1)))
        self.coverage_anchor_position_tolerance = max(0.01, float(rospy.get_param(
            "~coverage_anchor_position_tolerance", 0.15)))
        self.coverage_anchor_yaw_tolerance = math.radians(abs(float(rospy.get_param(
            "~coverage_anchor_yaw_tolerance_deg", math.degrees(0.06)))))
        self.coverage_anchor_yaw_hold = max(0.0, float(rospy.get_param(
            "~coverage_anchor_yaw_hold_sec", 0.5)))
        self.coverage_anchor_yaw_timeout = max(1.0, float(rospy.get_param(
            "~coverage_anchor_yaw_timeout_sec", 20.0)))
        self.coverage_no_progress_timeout = max(1.0, float(rospy.get_param(
            "~coverage_no_progress_timeout_sec", 5.5)))
        self.coverage_fallback_enabled = bool(rospy.get_param(
            "~coverage_fallback_enabled", True))
        raw_fallback_radii = rospy.get_param(
            "~coverage_fallback_radii_m", [0.18, 0.30, 0.42])
        try:
            self.coverage_fallback_radii = [
                abs(float(value)) for value in raw_fallback_radii
                if abs(float(value)) > 1e-6
            ]
        except (TypeError, ValueError):
            self.coverage_fallback_radii = [0.18, 0.30, 0.42]
        self.coverage_fallback_make_plan_service = rospy.get_param(
            "~coverage_fallback_make_plan_service", "/move_base/make_plan")
        self.coverage_fallback_make_plan_tolerance = max(0.0, float(
            rospy.get_param("~coverage_fallback_make_plan_tolerance_m", 0.12)))
        self.max_coverage_anchors = int(rospy.get_param("~max_coverage_anchors", 0))
        self.center_only = rospy.get_param("~center_only", False)
        self.coverage_scan_settle = rospy.get_param("~coverage_scan_settle_sec", 0.35)
        self.coverage_scan_step_angle = math.radians(max(
            1.0, float(rospy.get_param("~coverage_scan_step_deg", 20.0))))
        self.coverage_scan_angular_speed = abs(float(rospy.get_param(
            "~coverage_scan_angular_speed", 0.35)))
        self.coverage_scan_dwell = max(0.0, float(rospy.get_param(
            "~coverage_scan_dwell_sec", 0.65)))
        self.coverage_candidate_hold = max(0.0, float(rospy.get_param(
            "~coverage_candidate_hold_sec", 1.2)))
        self.coverage_scan_max_dwell = max(
            self.coverage_scan_dwell,
            float(rospy.get_param("~coverage_scan_max_dwell_sec", 2.0)))
        self.coverage_scan_pose_timeout = max(0.1, float(rospy.get_param(
            "~coverage_scan_pose_timeout_sec", 0.5)))
        self.coverage_scan_step_timeout_margin = max(0.1, float(rospy.get_param(
            "~coverage_scan_step_timeout_margin_sec", 2.0)))
        self.robot_footprint_radius = rospy.get_param("~robot_footprint_radius", 0.215)
        self.lethal_cost = int(rospy.get_param("~lethal_cost", 253))

        # OCR命中后先将目标框居中，再使用车头射线计算最终停泊点。
        self.target_center_tolerance = rospy.get_param("~target_center_tolerance", 0.08)
        self.target_center_required_hits = int(rospy.get_param(
            "~target_center_required_hits", 2))
        self.target_center_timeout = rospy.get_param("~target_center_timeout_sec", 12.0)
        self.target_bbox_stale = rospy.get_param("~target_bbox_stale_sec", 0.8)
        self.target_center_min_speed = rospy.get_param("~target_center_min_speed", 0.08)
        self.target_center_max_speed = rospy.get_param("~target_center_max_speed", 0.18)
        self.target_center_steering_sign = rospy.get_param(
            "~target_center_steering_sign", -1.0)
        self.target_center_coarse_step = math.radians(abs(float(rospy.get_param(
            "~target_center_coarse_step_deg", 4.0))))
        self.target_center_fine_step = math.radians(abs(float(rospy.get_param(
            "~target_center_fine_step_deg", 2.0))))
        self.target_center_fine_threshold = abs(float(rospy.get_param(
            "~target_center_fine_threshold", 0.20)))
        self.target_center_start_speed = abs(float(rospy.get_param(
            "~target_center_start_speed", 0.20)))
        self.target_center_step_max_speed = max(
            self.target_center_start_speed,
            abs(float(rospy.get_param("~target_center_step_max_speed", 0.35))))
        self.target_center_speed_increment = abs(float(rospy.get_param(
            "~target_center_speed_increment", 0.05)))
        self.target_center_motion_window = max(0.1, float(rospy.get_param(
            "~target_center_motion_window_sec", 0.6)))
        self.target_center_min_progress = math.radians(abs(float(rospy.get_param(
            "~target_center_min_progress_deg", 0.5))))
        self.target_center_settle = max(0.0, float(rospy.get_param(
            "~target_center_settle_sec", 0.25)))
        self.target_center_reverse_threshold = abs(float(rospy.get_param(
            "~target_center_reverse_threshold", 0.03)))
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.odom_stale = max(0.1, float(rospy.get_param(
            "~odom_stale_sec", 0.5)))
        self.camera_boresight_yaw_offset = rospy.get_param(
            "~camera_boresight_yaw_offset", 0.0)
        self.camera_horizontal_fov = math.radians(abs(float(rospy.get_param(
            "~camera_horizontal_fov_deg", 70.0))))
        self.camera_bearing_sign = float(rospy.get_param(
            "~camera_bearing_sign", self.target_center_steering_sign))
        self.arrival_hold_sec = rospy.get_param("~arrival_hold_sec", 0.6)

        # 任务2专用的50cm停车框。独立导航默认不启用验证并继续使用vision_offset。
        self.validate_parking_box = rospy.get_param("~validate_parking_box", False)
        self.parking_box_width = abs(float(rospy.get_param("~parking_box_width", 0.50)))
        self.parking_box_depth = abs(float(rospy.get_param("~parking_box_depth", 0.50)))
        self.parking_goal_offset = abs(float(rospy.get_param(
            "~parking_goal_offset", self.vision_offset)))
        self.parking_staging_offset = abs(float(rospy.get_param(
            "~parking_staging_offset", 0.55)))
        self.parking_staging_timeout = max(1.0, float(rospy.get_param(
            "~parking_staging_timeout_sec", 20.0)))
        self.parking_staging_acceptance = max(0.01, float(rospy.get_param(
            "~parking_staging_position_tolerance", 0.10)))
        self.parking_staging_yaw_tolerance = abs(float(rospy.get_param(
            "~parking_staging_yaw_tolerance", 0.10)))
        self.parking_staging_watchdog_window = max(0.5, float(rospy.get_param(
            "~parking_staging_watchdog_window_sec", 2.0)))
        self.parking_staging_no_progress_timeout = max(1.0, float(rospy.get_param(
            "~parking_staging_no_progress_timeout_sec", 4.0)))
        self.parking_staging_min_progress = max(0.0, float(rospy.get_param(
            "~parking_staging_min_progress", 0.03)))
        self.parking_staging_max_rotation = math.radians(abs(float(rospy.get_param(
            "~parking_staging_max_rotation_deg", 45.0))))
        self.parking_corner_tie_distance = max(0.0, float(rospy.get_param(
            "~parking_corner_tie_distance_m", 0.18)))
        self.parking_wall_endpoint_margin = max(0.0, float(rospy.get_param(
            "~parking_wall_endpoint_margin_m", 0.30)))
        raw_staging_normal_offsets = rospy.get_param(
            "~parking_staging_normal_offsets_m", [0.55, 0.70, 0.85])
        raw_staging_tangent_offsets = rospy.get_param(
            "~parking_staging_tangent_offsets_m", [0.0, 0.15, -0.15])
        try:
            self.parking_staging_normal_offsets = [
                max(self.parking_staging_offset, abs(float(value)))
                for value in raw_staging_normal_offsets
            ]
        except (TypeError, ValueError):
            self.parking_staging_normal_offsets = [
                self.parking_staging_offset, 0.70, 0.85]
        try:
            self.parking_staging_tangent_offsets = [
                float(value) for value in raw_staging_tangent_offsets]
        except (TypeError, ValueError):
            self.parking_staging_tangent_offsets = [0.0, 0.15, -0.15]
        self.parking_staging_max_attempts = max(1, int(rospy.get_param(
            "~parking_staging_max_attempts", 3)))
        self.parking_docking_timeout = max(1.0, float(rospy.get_param(
            "~parking_docking_timeout_sec", 15.0)))
        self.parking_dock_max_x = abs(float(rospy.get_param(
            "~parking_dock_max_x", 0.12)))
        self.parking_dock_max_y = abs(float(rospy.get_param(
            "~parking_dock_max_y", 0.08)))
        self.parking_dock_max_yaw = abs(float(rospy.get_param(
            "~parking_dock_max_yaw", 0.22)))
        self.parking_dock_min_yaw = min(
            self.parking_dock_max_yaw,
            abs(float(rospy.get_param("~parking_dock_min_yaw", 0.05))))
        self.parking_dock_normal_tolerance = abs(float(rospy.get_param(
            "~parking_dock_normal_tolerance", 0.035)))
        self.parking_dock_tangent_tolerance = abs(float(rospy.get_param(
            "~parking_dock_tangent_tolerance", 0.015)))
        self.parking_dock_yaw_tolerance = abs(float(rospy.get_param(
            "~parking_dock_yaw_tolerance", 0.05)))
        self.parking_dock_stable_sec = max(0.1, float(rospy.get_param(
            "~parking_dock_stable_sec", 0.25)))
        self.parking_dock_translation_yaw_gate = abs(float(rospy.get_param(
            "~parking_dock_translation_yaw_gate", 0.22)))
        self.parking_dock_forward_yaw_gate = abs(float(rospy.get_param(
            "~parking_dock_forward_yaw_gate", 0.14)))
        self.parking_dock_forward_tangent_gate = abs(float(rospy.get_param(
            "~parking_dock_forward_tangent_gate", 0.12)))
        self.parking_min_wall_distance = abs(float(rospy.get_param(
            "~parking_min_wall_distance", 0.19)))
        self.parking_lidar_stop_distance = abs(float(rospy.get_param(
            "~parking_lidar_stop_distance", 0.15)))
        self.parking_lidar_forward_offset = float(rospy.get_param(
            "~parking_lidar_forward_offset", 0.08))
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.scan_stale = max(0.1, float(rospy.get_param(
            "~scan_stale_sec", 0.5)))
        self.scan_front_half_angle = math.radians(abs(float(rospy.get_param(
            "~scan_front_half_angle_deg", 15.0))))
        self.parking_recenter_tolerance = abs(float(rospy.get_param(
            "~parking_recenter_tolerance", 0.04)))
        self.parking_recenter_timeout = max(1.0, float(rospy.get_param(
            "~parking_recenter_timeout_sec", 4.0)))
        self.parking_recenter_initial_wait = max(0.0, float(rospy.get_param(
            "~parking_recenter_initial_wait_sec", 1.0)))
        self.parking_recenter_lateral_kp = abs(float(rospy.get_param(
            "~parking_recenter_lateral_kp", 0.16)))
        self.parking_recenter_lateral_sign = (
            1.0 if float(rospy.get_param(
                "~parking_recenter_lateral_sign", 1.0)) >= 0.0 else -1.0)
        self.parking_recenter_min_lateral = abs(float(rospy.get_param(
            "~parking_recenter_min_lateral", 0.015)))
        self.parking_recenter_max_lateral = abs(float(rospy.get_param(
            "~parking_recenter_max_lateral", 0.065)))
        self.parking_recenter_max_travel = abs(float(rospy.get_param(
            "~parking_recenter_max_travel", 0.30)))
        self.parking_recenter_yaw_kp = abs(float(rospy.get_param(
            "~parking_recenter_yaw_kp", 1.0)))
        self.parking_recenter_yaw_tolerance = math.radians(abs(float(
            rospy.get_param("~parking_recenter_yaw_tolerance_deg", 2.0))))
        self.parking_recenter_max_yaw = abs(float(rospy.get_param(
            "~parking_recenter_max_yaw", 0.18)))
        self.parking_recenter_stable_sec = max(0.05, float(rospy.get_param(
            "~parking_recenter_stable_sec", 0.25)))
        self.parking_recenter_required_hits = max(2, int(rospy.get_param(
            "~parking_recenter_required_hits", 3)))
        self.parking_recenter_side_half_angle = math.radians(abs(float(
            rospy.get_param("~parking_recenter_side_half_angle_deg", 25.0))))
        self.parking_recenter_side_stop = abs(float(rospy.get_param(
            "~parking_recenter_side_stop_m", 0.18)))
        self.parking_recenter_side_slow = max(
            self.parking_recenter_side_stop + 0.01,
            abs(float(rospy.get_param("~parking_recenter_side_slow_m", 0.28))))
        self.parking_wall_fit_half_angle = math.radians(abs(float(rospy.get_param(
            "~parking_wall_fit_half_angle_deg", 35.0))))
        self.parking_wall_fit_min_points = max(2, int(rospy.get_param(
            "~parking_wall_fit_min_points", 12)))
        self.parking_wall_fit_min_span = abs(float(rospy.get_param(
            "~parking_wall_fit_min_span", 0.25)))
        self.parking_wall_fit_near_min_span = abs(float(rospy.get_param(
            "~parking_wall_fit_near_min_span", 0.18)))
        self.parking_wall_fit_max_distance_jump = abs(float(rospy.get_param(
            "~parking_wall_fit_max_distance_jump", 0.05)))
        self.parking_wall_fit_max_normal_jump = math.radians(abs(float(
            rospy.get_param("~parking_wall_fit_max_normal_jump_deg", 8.0))))
        self.parking_wall_fit_max_residual = abs(float(rospy.get_param(
            "~parking_wall_fit_max_residual", 0.015)))
        self.parking_wall_fit_max_normal_error = math.radians(abs(float(
            rospy.get_param("~parking_wall_fit_max_normal_error_deg", 20.0))))
        self.parking_wall_fit_grace = max(0.5, float(rospy.get_param(
            "~parking_wall_fit_grace_sec", 1.5)))
        self.parking_wall_fit_filter_alpha = min(1.0, max(0.05, float(
            rospy.get_param("~parking_wall_fit_filter_alpha", 0.45))))
        self.parking_normal_offset = float(rospy.get_param(
            "~parking_normal_offset", 0.0))
        self.parking_tangent_offset = float(rospy.get_param(
            "~parking_tangent_offset", 0.0))
        self.parking_xy_tolerance = abs(float(rospy.get_param(
            "~parking_xy_tolerance", 0.04)))
        self.parking_yaw_tolerance = abs(float(rospy.get_param(
            "~parking_yaw_tolerance", 0.06)))
        self.parking_validation_margin = max(0.0, float(rospy.get_param(
            "~parking_validation_margin", 0.01)))
        self.parking_required_margin = max(0.0, float(rospy.get_param(
            "~parking_required_margin", 0.02)))
        self.footprint_half_length = abs(float(rospy.get_param(
            "~footprint_half_length", 0.171)))
        self.footprint_half_width = abs(float(rospy.get_param(
            "~footprint_half_width", 0.128)))
        self.local_planner_reconfigure_ns = rospy.get_param(
            "~local_planner_reconfigure_ns", "/move_base/TebLocalPlannerROS")
        self.move_base_reconfigure_ns = rospy.get_param(
            "~move_base_reconfigure_ns", "/move_base")

        # move_base 与 costmap
        self.move_base_server = rospy.get_param("~move_base_server", "/move_base")
        self.costmap_topic = rospy.get_param("~costmap_topic", "/move_base/local_costmap/costmap")
        self.cost_threshold = rospy.get_param("~cost_threshold", 100)
        self.feasibility_check_rate = rospy.get_param("~feasibility_check_rate", 1.0)

        # 自转参数
        self.rotation_speed = abs(rospy.get_param("~rotation_speed", 0.5))  # 左转为正，rad/s
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # 巡航点与结束点
        self.patrol_points = rospy.get_param("~patrol_points", [])
        self.end_goal = rospy.get_param("~end_goal",
                                        {"x": 0.3195, "y": -3.2703, "yaw": -1.5596})

        # 初始位姿（用于 AMCL 2D Pose Estimate）
        self.publish_initial_pose = rospy.get_param("~publish_initial_pose", True)
        self.initial_pose = rospy.get_param("~initial_pose", {"x": 0.0, "y": 0.0, "yaw": 0.0})

        # ---------- ROS 通信 ----------
        self.tf_listener = tf.TransformListener()
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True)
        self.coverage_observation_pub = rospy.Publisher(
            self.coverage_observation_topic, String, queue_size=10, latch=True)

        self.costmap = None
        self.costmap_received_at = 0.0
        # Full OccupancyGrid messages must never build an unbounded callback
        # backlog.  Retaining only the newest map also avoids multi-GB allocator
        # growth during a long OCR patrol.
        self.costmap_sub = rospy.Subscriber(
            self.costmap_topic,
            OccupancyGrid,
            self._costmap_cb,
            queue_size=1,
            buff_size=4 * 1024 * 1024,
            tcp_nodelay=True,
        )
        self.odom_yaw = None
        self.odom_pose = None
        self.odom_frame_from_msg = self.odom_frame
        self.odom_received_at = 0.0
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        self.scan_front_min = None
        self.scan_left_min = None
        self.scan_right_min = None
        self.scan_wall_points = []
        self.scan_received_at = 0.0
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)

        self.target_error = None
        self.target_payload_at = 0.0
        self.last_target_payload = None
        if self.coverage_search_mode:
            rospy.Subscriber(self.target_topic, String, self._target_cb, queue_size=10)

        self.triggered = False
        self.trigger_lock = threading.Lock()

        # 连接 move_base action server
        self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
        rospy.loginfo("[vision_triggered_navigator] 等待 move_base 服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("[vision_triggered_navigator] move_base 服务器已连接.")

        # 当前目标记录（用于定时器检查）
        self.current_goal_x = None
        self.current_goal_y = None
        self.current_goal_infeasible = False
        self.current_goal_timed_out = False
        self.current_goal_rotation_stall = False
        self.current_goal_needs_yaw_alignment = False
        self.current_goal_pose_accepted = False
        self.current_goal_no_progress = False
        self.current_goal_pose_accepted = False
        self.current_goal_no_progress = False
        self.parking_wall_point = None
        self.parking_inward_normal = None
        self.parking_wall_name = None
        self.parking_final_wall_fit = None
        self.parking_final_tangent_error = None
        self.parking_last_wall_fit = None
        self.parking_last_wall_fit_at = 0.0
        self.parking_last_wall_fit_pose = None
        self.parking_failure_status = "parking_docking_failed"
        self._saved_planner_tolerances = None
        self._saved_move_base_recovery = None
        self._saved_teb_oscillation_recovery = None
        rospy.on_shutdown(self._restore_final_tolerances)
        rospy.on_shutdown(self._restore_move_base_recovery)
        self._publish_status("ready")
        # 只有在 action client 和全部状态完成初始化后才接收触发，避免启动竞态。
        self.trigger_service = rospy.Service(
            self.trigger_service_name, Trigger, self._trigger_service_cb)
        self.start_navigation_service = rospy.Service(
            self.start_navigation_service_name,
            Trigger,
            self._start_navigation_service_cb,
        )
        rospy.loginfo("[vision_triggered_navigator] 可靠触发服务已就绪: %s",
                      self.trigger_service_name)
        if self.trigger_mode == "vision":
            rospy.Subscriber(self.vision_topic, Bool, self._vision_cb)
            rospy.loginfo("[vision_triggered_navigator] 触发模式：视觉话题 <%s>", self.vision_topic)
        elif self.trigger_mode == "keyboard":
            t = threading.Thread(target=self._keyboard_thread)
            t.daemon = True
            t.start()
            rospy.loginfo("[vision_triggered_navigator] 触发模式：键盘回车")
        else:
            rospy.logerr("[vision_triggered_navigator] 未知触发模式 '%s'，仅支持 keyboard/vision", self.trigger_mode)

    # ------------------------------------------------------------------
    # 回调与工具函数
    # ------------------------------------------------------------------
    def _costmap_cb(self, msg):
        """保存最新 costmap"""
        self.costmap = msg
        self.costmap_received_at = rospy.get_time()

    def _odom_cb(self, msg):
        """Keep a fresh odom pose for centering and final docking."""
        self.odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            self.odom_yaw,
        )
        if msg.header.frame_id:
            self.odom_frame_from_msg = msg.header.frame_id
        self.odom_received_at = rospy.get_time()

    def _scan_cb(self, msg):
        """Store the nearest range and front-sector points in base coordinates."""
        nearest = None
        left_nearest = None
        right_nearest = None
        wall_points = []
        angle = float(msg.angle_min)
        for value in msg.ranges:
            base_angle = normalize_angle(angle)
            distance = float(value)
            if (math.isfinite(distance) and
                    distance >= float(msg.range_min) and
                    distance <= float(msg.range_max)):
                if abs(base_angle) <= self.scan_front_half_angle:
                    nearest = distance if nearest is None else min(nearest, distance)
                if abs(normalize_angle(base_angle - math.pi * 0.5)) <= self.parking_recenter_side_half_angle:
                    left_nearest = (distance if left_nearest is None else
                                    min(left_nearest, distance))
                if abs(normalize_angle(base_angle + math.pi * 0.5)) <= self.parking_recenter_side_half_angle:
                    right_nearest = (distance if right_nearest is None else
                                     min(right_nearest, distance))
                if abs(base_angle) <= self.parking_wall_fit_half_angle:
                    wall_points.append((
                        self.parking_lidar_forward_offset +
                        distance * math.cos(base_angle),
                        distance * math.sin(base_angle),
                    ))
            angle += float(msg.angle_increment)
        self.scan_front_min = nearest
        self.scan_left_min = left_nearest
        self.scan_right_min = right_nearest
        self.scan_wall_points = wall_points
        self.scan_received_at = rospy.get_time()

    def _publish_status(self, status):
        """发布简洁、稳定的流程状态，供比赛总控监听。"""
        self.status_pub.publish(String(data=status))

    def _publish_coverage_observation(self, state, anchor_index, point):
        """Publish the active calibrated anchor for a later direct revisit."""
        payload = {
            "state": str(state),
            "anchor_index": int(anchor_index) + 1,
            "x": float(point.get("x", 0.0)),
            "y": float(point.get("y", 0.0)),
            "yaw": float(point.get("yaw", 0.0)),
            "stamp": rospy.get_time(),
        }
        self.coverage_observation_pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))

    def _accept_trigger(self, source):
        """Idempotently latch a target trigger and cancel active navigation."""
        with self.trigger_lock:
            self.triggered, accepted = latch_trigger(self.triggered)
            if not accepted:
                rospy.loginfo("[vision_triggered_navigator] %s触发重复到达，保持已锁存状态.",
                              source)
                return False
        rospy.loginfo("[vision_triggered_navigator] 收到%s触发，打断当前导航.", source)
        self._publish_status("triggered")
        self.cancel_goal()
        return True

    def _vision_cb(self, msg):
        """视觉触发回调"""
        if msg.data:
            self._accept_trigger("视觉话题")

    def _trigger_service_cb(self, _request):
        """Reliably acknowledge competition target lock requests."""
        accepted = self._accept_trigger("目标服务")
        if accepted:
            return TriggerResponse(True, "target trigger accepted and latched")
        return TriggerResponse(True, "target trigger was already latched")

    def _start_navigation_service_cb(self, _request):
        """Release a fully initialized navigator without restarting its process."""
        if self.navigation_start_event.is_set():
            return TriggerResponse(True, "navigation was already released")
        self.navigation_start_event.set()
        self._publish_status("start_released")
        return TriggerResponse(True, "navigation released")

    def _target_cb(self, msg):
        """保存当前目标厂牌在完整图像中的水平位置。"""
        try:
            payload = json.loads(msg.data)
            center_x = float(payload.get("target_center_x"))
            image_width = float(payload.get("image_width"))
            if image_width <= 1.0:
                return
            self.target_error = (center_x - image_width * 0.5) / (image_width * 0.5)
            self.target_payload_at = rospy.get_time()
            self.last_target_payload = payload
        except (TypeError, ValueError, KeyError):
            return

    def publish_initial_pose_to_amcl(self):
        """发布 /initialpose 给 AMCL 做初始定位"""
        if not self.publish_initial_pose:
            return

        x = self.initial_pose.get("x", 0.0)
        y = self.initial_pose.get("y", 0.0)
        yaw = self.initial_pose.get("yaw", 0.0)

        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.loginfo("[vision_triggered_navigator] 等待 /initialpose 订阅者...")
        wait_start = rospy.Time.now()
        while pub.get_num_connections() == 0:
            if (rospy.Time.now() - wait_start).to_sec() > 5.0:
                rospy.logwarn("[vision_triggered_navigator] 5秒内没有 /initialpose 订阅者，继续执行...")
                break
            rospy.sleep(0.1)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = euler_to_quaternion(yaw)
        msg.pose.covariance = [
            0.1, 0,   0,   0,   0,   0,
            0,   0.1, 0,   0,   0,   0,
            0,   0,   0,   0,   0,   0,
            0,   0,   0,   0,   0,   0,
            0,   0,   0,   0,   0,   0,
            0,   0,   0,   0,   0,   0.04
        ]

        pub.publish(msg)
        rospy.loginfo("[vision_triggered_navigator] 已发布初始位姿: x=%.4f, y=%.4f, yaw=%.4f", x, y, yaw)
        rospy.sleep(3)

    def _keyboard_thread(self):
        """键盘回车触发线程"""
        # 兼容 Python 2/3
        if sys.version_info[0] >= 3:
            input_func = input
        else:
            input_func = raw_input

        rospy.loginfo("[vision_triggered_navigator] 按回车键触发视觉导航...")
        while not rospy.is_shutdown() and not self.triggered:
            try:
                input_func()
                if not self.triggered:
                    self._accept_trigger("键盘")
            except EOFError:
                rospy.sleep(0.5)

    def _build_rect_bounds(self):
        """Build a safety AABB plus the four measured, possibly skewed walls."""
        xs = [p[0] for p in self.vision_rect_corners] # 去四个角点的 x 坐标
        ys = [p[1] for p in self.vision_rect_corners] # 去四个角点的 y 坐标
        self.rect_x_min = min(xs)
        self.rect_x_max = max(xs)
        self.rect_y_min = min(ys)
        self.rect_y_max = max(ys)

        # 最终停泊使用实测四边形墙段；AABB不参与观察点或停车目标生成。
        self.walls = build_quadrilateral_walls(self.vision_rect_corners)

    def _get_robot_pose(self, frame_id):
        """查询 map -> frame_id 的位姿，返回 (x, y, yaw)；失败返回 None"""
        try:
            self.tf_listener.waitForTransform(self.map_frame, frame_id, rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, frame_id, rospy.Time(0))
            yaw = quaternion_to_yaw(rot)
            return trans[0], trans[1], yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "[vision_triggered_navigator] TF 查询 %s -> %s 失败: %s",
                                   self.map_frame, frame_id, str(e))
            return None

    def _get_cost_at(self, x, y):
        """查询map目标在costmap中的代价；未知、TF失败或越界返回-1。"""
        if self.costmap is None:
            return -1

        point = self._map_point_in_costmap_frame(x, y)
        if point is None:
            return -1
        cost_x, cost_y = point
        info = self.costmap.info
        cost = costmap_value_at(
            self.costmap.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, cost_x, cost_y)
        if cost < 0:
            return -1
        rospy.loginfo_throttle(
            5.0,
            "[vision_triggered_navigator] 查询costmap map=(%.3f, %.3f) %s=(%.3f, %.3f) -> cost=%d",
            x, y, self.costmap.header.frame_id or self.map_frame,
            cost_x, cost_y, cost)
        return cost

    def _map_point_in_costmap_frame(self, x, y):
        """Transform a map-frame point into the latest costmap frame."""
        if self.costmap is None:
            return None
        frame = self.costmap.header.frame_id or self.map_frame
        if frame == self.map_frame:
            return float(x), float(y)
        point = PoseStamped()
        point.header.frame_id = self.map_frame
        point.header.stamp = rospy.Time(0)
        point.pose.position.x = float(x)
        point.pose.position.y = float(y)
        point.pose.orientation.w = 1.0
        try:
            self.tf_listener.waitForTransform(
                frame, self.map_frame, rospy.Time(0), rospy.Duration(0.3))
            transformed = self.tf_listener.transformPose(frame, point)
            return transformed.pose.position.x, transformed.pose.position.y
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "[vision_triggered_navigator] map目标(%.4f, %.4f)无法转换到costmap坐标系%s: %s；按未知代价处理",
                x, y, frame, str(exc))
            return None

    def _coverage_pose_cost(self, x, y):
        """Evaluate the whole robot footprint; unknown means 'let move_base decide'."""
        if self.costmap is None:
            return False, -1, False
        point = self._map_point_in_costmap_frame(x, y)
        if point is None:
            return False, -1, False
        info = self.costmap.info
        return footprint_max_cost(
            self.costmap.data,
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            point[0],
            point[1],
            self.robot_footprint_radius,
            self.lethal_cost,
        )

    def _coverage_point_inside_room(self, x, y):
        """Keep fallback observation poses inside the measured room envelope."""
        margin = max(0.08, min(self.robot_footprint_radius, 0.20))
        return (
            self.rect_x_min + margin <= float(x) <= self.rect_x_max - margin and
            self.rect_y_min + margin <= float(y) <= self.rect_y_max - margin
        )

    def _coverage_plan_length(self, x, y, yaw):
        """Return a global-plan length to a candidate, or None when unreachable."""
        pose = self._get_robot_pose(self.base_frame)
        if pose is None:
            return None
        request = GetPlanRequest()
        request.start.header.frame_id = self.map_frame
        request.start.header.stamp = rospy.Time.now()
        request.start.pose.position.x = pose[0]
        request.start.pose.position.y = pose[1]
        request.start.pose.orientation = euler_to_quaternion(pose[2])
        request.goal.header.frame_id = self.map_frame
        request.goal.header.stamp = request.start.header.stamp
        request.goal.pose.position.x = float(x)
        request.goal.pose.position.y = float(y)
        request.goal.pose.orientation = euler_to_quaternion(yaw)
        request.tolerance = self.coverage_fallback_make_plan_tolerance
        try:
            rospy.wait_for_service(
                self.coverage_fallback_make_plan_service, timeout=0.35)
            response = rospy.ServiceProxy(
                self.coverage_fallback_make_plan_service, GetPlan)(request)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(
                2.0, "[vision_triggered_navigator] nearby make_plan unavailable: %s",
                str(exc))
            return None
        poses = response.plan.poses
        if not poses:
            return None
        length = 0.0
        previous = poses[0].pose.position
        for stamped in poses[1:]:
            current = stamped.pose.position
            length += math.hypot(current.x - previous.x, current.y - previous.y)
            previous = current
        return length

    def _select_coverage_fallback(self, x, y, yaw):
        """Choose one nearby footprint-safe point that move_base can actually plan to."""
        if not self.coverage_fallback_enabled:
            return None
        pose = self._get_robot_pose(self.base_frame)
        if pose is None:
            return None
        candidates = coverage_fallback_candidates(
            x, y, pose[0], pose[1], self.coverage_fallback_radii)
        for candidate_x, candidate_y in candidates:
            if not self._coverage_point_inside_room(candidate_x, candidate_y):
                continue
            known, max_cost, _blocked = self._coverage_pose_cost(
                candidate_x, candidate_y)
            if known and max_cost >= self.lethal_cost:
                continue
            plan_length = self._coverage_plan_length(candidate_x, candidate_y, yaw)
            if plan_length is None:
                continue
            rospy.logwarn(
                "[vision_triggered_navigator] nearby scan fallback: "
                "requested=(%.2f,%.2f) selected=(%.2f,%.2f) "
                "offset=%.2fm plan=%.2fm cost=%d",
                x, y, candidate_x, candidate_y,
                math.hypot(candidate_x - x, candidate_y - y),
                plan_length, max_cost)
            return candidate_x, candidate_y, yaw
        return None

    def is_goal_feasible(self, x, y):
        """判断目标点是否可行：cost 已知且小于阈值"""
        cost = self._get_cost_at(x, y)
        if cost < 0:
            rospy.logwarn_throttle(2.0,
                "[vision_triggered_navigator] 目标 (%.3f, %.3f) 代价未知，按可行处理", x, y)
            return True
        if cost >= self.cost_threshold:
            rospy.logwarn("[vision_triggered_navigator] 目标 (%.3f, %.3f) 代价 %d >= 阈值 %d，不可行",
                          x, y, cost, self.cost_threshold)
            return False
        return True

    def cancel_goal(self):
        """取消当前 move_base 目标"""
        if self.move_base_client.get_state() in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
            rospy.logwarn("[vision_triggered_navigator] 取消当前 move_base 目标.")
            self.move_base_client.cancel_goal()

    def _wait_navigation_idle(self, timeout=2.0):
        """Do not publish direct cmd_vel until move_base has relinquished control."""
        deadline = rospy.get_time() + max(0.0, float(timeout))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            state = self.move_base_client.get_state()
            if state not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                self.cmd_vel_pub.publish(Twist())
                return True
            self.move_base_client.cancel_goal()
            self.cmd_vel_pub.publish(Twist())
            rate.sleep()
        state = self.move_base_client.get_state()
        return state not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]

    def _clear_costmaps_and_wait(self, timeout=2.0):
        """Clear stale obstacle history, then require fresh scan and costmap data."""
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=timeout)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("[vision_triggered_navigator] 清理costmap失败: %s", str(exc))
            return False
        called_at = rospy.get_time()
        deadline = rospy.get_time() + max(0.1, float(timeout))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if (self.costmap_received_at > called_at and
                    self.scan_received_at > called_at):
                rospy.loginfo("[vision_triggered_navigator] costmap清理后已收到新雷达和完整局部代价地图.")
                return True
            rate.sleep()
        rospy.logerr("[vision_triggered_navigator] costmap清理后未在%.1fs内收到新快照.", timeout)
        return False

    def _align_coverage_anchor_yaw(self, map_pose):
        """Finish only the calibrated anchor heading with odometry, not TEB."""
        odom_frame = self.odom_frame_from_msg or self.odom_frame
        target = self._transform_map_pose(odom_frame, map_pose)
        if target is None:
            return False
        self._publish_status("coverage_anchor_yaw_aligning")
        deadline = rospy.get_time() + self.coverage_anchor_yaw_timeout
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if self.triggered:
                self.cmd_vel_pub.publish(Twist())
                rospy.loginfo(
                    "[vision_triggered_navigator] OCR触发优先，中断锚点航向补偿.")
                return False
            if not self._odom_is_fresh():
                self.cmd_vel_pub.publish(Twist())
                return False
            error = normalize_angle(target[2] - self.odom_yaw)
            if abs(error) <= self.coverage_anchor_yaw_tolerance:
                self._hold_stopped(self.coverage_anchor_yaw_hold)
                self._publish_status("coverage_anchor_yaw_aligned")
                rospy.loginfo(
                    "[vision_triggered_navigator] 精确锚点航向由odom闭环完成: error=%.3frad.",
                    error)
                return True
            step = min(abs(error), math.radians(10.0))
            if not self._rotate_center_step(
                    1.0 if error > 0.0 else -1.0, step,
                    abort_on_trigger=True):
                return False
        self.cmd_vel_pub.publish(Twist())
        rospy.logerr("[vision_triggered_navigator] 精确锚点航向闭环超过%.1fs.",
                     self.coverage_anchor_yaw_timeout)
        return False

    def _check_current_goal_cb(self, event):
        """定时器回调：检查当前导航目标是否仍然可行"""
        if self.current_goal_infeasible:
            return
        if self.current_goal_x is None or self.current_goal_y is None:
            return
        if not self.is_goal_feasible(self.current_goal_x, self.current_goal_y):
            rospy.logwarn("[vision_triggered_navigator] 当前导航目标中途变得不可行，取消并跳点.")
            self.current_goal_infeasible = True
            self.cancel_goal()

    # ------------------------------------------------------------------
    # 动作执行
    # ------------------------------------------------------------------
    def send_goal(self, x, y, yaw):
        """
        发送导航目标并等待结果。
        等待期间启动定时器实时检查目标可行性。
        返回 move_base 终态（SUCCEEDED / PREEMPTED / ABORTED / ...）
        """
        self.current_goal_x = x
        self.current_goal_y = y
        self.current_goal_infeasible = False
        self.current_goal_timed_out = False
        self.current_goal_rotation_stall = False
        self.current_goal_needs_yaw_alignment = False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = euler_to_quaternion(yaw)

        rospy.loginfo("[vision_triggered_navigator] 发送导航目标: x=%.4f y=%.4f yaw=%.4f", x, y, yaw)
        self.move_base_client.send_goal(goal)

        timer = None
        if not self.coverage_search_mode:
            # 仅保留给原有独立导航模式；任务2覆盖模式不再按单栅格cost取消目标。
            period = rospy.Duration(1.0 / max(self.feasibility_check_rate, 0.1))
            timer = rospy.Timer(period, self._check_current_goal_cb)

        started = rospy.get_time()
        progress_samples = []
        last_progress_check = 0.0
        last_progress_log = 0.0
        latest_distance = float("nan")
        latest_yaw_error = float("nan")
        window_progress = 0.0
        coverage_extended = False
        rotation_window_started = started
        rotation_window_pose = None
        rotation_window_yaw = None
        rotation_accumulated = 0.0
        anchor_close_since = None
        no_progress_started = None
        no_progress_distance = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            state = self.move_base_client.get_state()
            if state not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                break
            if self.coverage_search_mode and not self.triggered:
                now = rospy.get_time()
                elapsed = now - started
                if now - last_progress_check >= 0.25:
                    last_progress_check = now
                    pose = self._get_robot_pose(self.base_frame)
                    if pose is not None:
                        latest_distance = math.hypot(x - pose[0], y - pose[1])
                        latest_yaw_error = abs(normalize_angle(yaw - pose[2]))
                        if rotation_window_pose is None:
                            rotation_window_pose = pose
                            rotation_window_yaw = pose[2]
                            rotation_window_started = now
                        else:
                            rotation_accumulated += abs(normalize_angle(
                                pose[2] - rotation_window_yaw))
                            rotation_window_yaw = pose[2]
                        progress_samples.append((now, latest_distance))
                        cutoff = now - self.coverage_goal_progress_window
                        progress_samples = [
                            item for item in progress_samples if item[0] >= cutoff]
                        if progress_samples:
                            window_progress = max(
                                0.0, progress_samples[0][1] - latest_distance)
                        if no_progress_started is None:
                            no_progress_started = now
                            no_progress_distance = latest_distance
                        elif (no_progress_distance - latest_distance >=
                              self.coverage_goal_min_progress):
                            no_progress_started = now
                            no_progress_distance = latest_distance
                        elif (now - no_progress_started >=
                              self.coverage_no_progress_timeout and
                              latest_distance > self.coverage_anchor_position_tolerance):
                            self.current_goal_no_progress = True
                            self._publish_status("coverage_goal_no_progress")
                            rospy.logwarn(
                                "[vision_triggered_navigator] scan goal made no progress "
                                "for %.1fs (distance=%.3fm); cancel before move_base oscillates.",
                                now - no_progress_started, latest_distance)
                            self.cancel_goal()
                            break
                        if latest_distance <= self.coverage_anchor_position_tolerance:
                            if anchor_close_since is None:
                                anchor_close_since = now
                            elif now - anchor_close_since >= self.coverage_anchor_yaw_hold:
                                if latest_yaw_error <= self.coverage_anchor_yaw_tolerance:
                                    self.current_goal_pose_accepted = True
                                    rospy.logwarn(
                                        "[vision_triggered_navigator] scan pose accepted "
                                        "within relaxed tolerance: distance=%.3fm yaw_error=%.1fdeg.",
                                        latest_distance, math.degrees(latest_yaw_error))
                                else:
                                    self.current_goal_needs_yaw_alignment = True
                                    rospy.logwarn(
                                        "[vision_triggered_navigator] scan position accepted "
                                        "(distance=%.3fm); switch from TEB to odom yaw alignment "
                                        "(error=%.1fdeg).",
                                        latest_distance, math.degrees(latest_yaw_error))
                                self.cancel_goal()
                                break
                        else:
                            anchor_close_since = None
                        if (rotation_window_pose is not None and
                                now - rotation_window_started >=
                                self.coverage_rotation_watchdog_window):
                            moved = math.hypot(
                                pose[0] - rotation_window_pose[0],
                                pose[1] - rotation_window_pose[1])
                            if coverage_motion_is_rotation_stall(
                                    moved, rotation_accumulated,
                                    self.coverage_rotation_min_progress,
                                    self.coverage_rotation_max_yaw):
                                self.current_goal_rotation_stall = True
                                self._publish_status(
                                    "coverage_goal_recovery_preempted")
                                rospy.logwarn(
                                    "[vision_triggered_navigator] 覆盖目标转圈预警: %.1fs位移%.3fm累计转角%.1fdeg；抢在move_base恢复前取消.",
                                    self.coverage_rotation_watchdog_window,
                                    moved, math.degrees(rotation_accumulated))
                                self.cancel_goal()
                                break
                            rotation_window_started = now
                            rotation_window_pose = pose
                            rotation_window_yaw = pose[2]
                            rotation_accumulated = 0.0
                    if now - last_progress_log >= 2.0:
                        last_progress_log = now
                        rospy.loginfo(
                            "[vision_triggered_navigator] 覆盖目标进度 elapsed=%.1fs distance=%.3fm yaw_error=%.3frad window_progress=%.3fm extended=%s",
                            elapsed, latest_distance, latest_yaw_error,
                            window_progress, coverage_extended)

                if coverage_extended:
                    decision = coverage_timeout_decision(
                        elapsed, self.coverage_goal_min_progress,
                        self.coverage_goal_soft_timeout,
                        self.coverage_goal_hard_timeout,
                        self.coverage_goal_min_progress)
                    if decision != "hard_timeout":
                        rate.sleep()
                        continue
                else:
                    decision = coverage_timeout_decision(
                        elapsed, window_progress,
                        self.coverage_goal_soft_timeout,
                        self.coverage_goal_hard_timeout,
                        self.coverage_goal_min_progress)
                    if decision == "extend":
                        coverage_extended = True
                        self._publish_status("coverage_goal_extended")
                        rospy.logwarn(
                            "[vision_triggered_navigator] 覆盖目标达到软时限%.1fs，但最近%.1fs仍前进%.3fm，延长至硬时限%.1fs.",
                            self.coverage_goal_soft_timeout,
                            self.coverage_goal_progress_window,
                            window_progress,
                            self.coverage_goal_hard_timeout)
                        rate.sleep()
                        continue
                if decision in ("soft_timeout", "hard_timeout"):
                    self.current_goal_timed_out = True
                    rospy.logwarn(
                        "[vision_triggered_navigator] 精确观察点(%.4f, %.4f, %.4f)%s: elapsed=%.1fs distance=%.3fm window_progress=%.3fm，取消并进入下一原始锚点.",
                        x, y, yaw,
                        "达到硬时限" if decision == "hard_timeout" else "软时限无有效进展",
                        elapsed, latest_distance, window_progress)
                    self.cancel_goal()
                    break
            rate.sleep()

        if timer is not None:
            timer.shutdown()
        if (self.current_goal_timed_out or self.current_goal_rotation_stall or
                self.current_goal_needs_yaw_alignment or
                self.current_goal_pose_accepted or self.current_goal_no_progress):
            self.move_base_client.wait_for_result(rospy.Duration(1.0))
        final_state = self.move_base_client.get_state()

        if final_state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[vision_triggered_navigator] 导航目标到达.")
        elif final_state == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn("[vision_triggered_navigator] 导航目标被抢占/取消.")
        elif final_state == actionlib.GoalStatus.ABORTED:
            rospy.logerr("[vision_triggered_navigator] 导航目标被终止.")
        else:
            rospy.logwarn("[vision_triggered_navigator] 导航结束，状态: %s", str(final_state))

        return final_state

    def _hold_scan_step(self, step_label, candidate_since):
        """Publish zero velocity while OCR consumes stable frames at one heading."""
        started = rospy.get_time()
        deadline = scan_dwell_deadline(
            started,
            self.coverage_scan_dwell,
            self.target_payload_at if self.target_payload_at >= candidate_since else 0.0,
            self.coverage_candidate_hold,
            self.coverage_scan_max_dwell,
        )
        extension_logged = False
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.triggered:
            self.cmd_vel_pub.publish(Twist())
            candidate_at = self.target_payload_at
            new_deadline = scan_dwell_deadline(
                started,
                self.coverage_scan_dwell,
                candidate_at if candidate_at >= candidate_since else 0.0,
                self.coverage_candidate_hold,
                self.coverage_scan_max_dwell,
            )
            if new_deadline > deadline:
                deadline = new_deadline
            if (not extension_logged and candidate_at >= candidate_since and
                    deadline > started + self.coverage_scan_dwell + 1e-3):
                extension_logged = True
                rospy.loginfo(
                    "[vision_triggered_navigator] %s 收到目标候选，停车延长确认至%.2fs.",
                    step_label, deadline - started)
            if rospy.get_time() >= deadline:
                break
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def _step_scan(self, direction, duration):
        """Run a TF-closed-loop stop-and-look sweep for task2 coverage mode."""
        if duration <= 0:
            return True
        if self.coverage_scan_angular_speed <= 0.0:
            rospy.logerr("[vision_triggered_navigator] 步进扫描角速度必须大于0.")
            return False

        direction_sign = 1.0 if direction == "left" else -1.0
        total_angle = abs(self.rotation_speed * float(duration))
        steps = split_scan_angle(total_angle, self.coverage_scan_step_angle)
        rospy.loginfo(
            "[vision_triggered_navigator] 步进扫描 %s: total=%.1fdeg steps=%d "
            "speed=%.2frad/s dwell=%.2fs",
            direction, math.degrees(total_angle), len(steps),
            self.coverage_scan_angular_speed, self.coverage_scan_dwell)

        for step_index, step_angle in enumerate(steps, 1):
            if self.triggered:
                self.cmd_vel_pub.publish(Twist())
                return True
            start_pose = self._get_robot_pose(self.base_frame)
            if start_pose is None:
                self.cmd_vel_pub.publish(Twist())
                rospy.logerr(
                    "[vision_triggered_navigator] 步进扫描%d/%d起始TF不可用，延期当前锚点.",
                    step_index, len(steps))
                return False

            step_started = rospy.get_time()
            last_pose_at = step_started
            handled_candidate_at = self.target_payload_at
            deadline = (step_started + step_angle / self.coverage_scan_angular_speed +
                        self.coverage_scan_step_timeout_margin)
            max_progress = 0.0
            twist = Twist()
            twist.angular.z = self.coverage_scan_angular_speed * direction_sign
            rate = rospy.Rate(20)

            while not rospy.is_shutdown() and not self.triggered:
                now = rospy.get_time()
                if self.target_payload_at > handled_candidate_at:
                    handled_candidate_at = self.target_payload_at
                    self.cmd_vel_pub.publish(Twist())
                    rospy.loginfo(
                        "[vision_triggered_navigator] 步进%d/%d转动中捕获目标候选，立即停车确认.",
                        step_index, len(steps))
                    pause_started = rospy.get_time()
                    self._hold_scan_step(
                        "步进{}/{}".format(step_index, len(steps)), step_started)
                    deadline += max(0.0, rospy.get_time() - pause_started)
                    if self.triggered:
                        return True
                    handled_candidate_at = self.target_payload_at

                pose = self._get_robot_pose(self.base_frame)
                if pose is not None:
                    last_pose_at = now
                    progress = normalize_angle(pose[2] - start_pose[2]) * direction_sign
                    max_progress = max(max_progress, progress)
                    if max_progress >= step_angle:
                        break
                elif now - last_pose_at >= self.coverage_scan_pose_timeout:
                    self.cmd_vel_pub.publish(Twist())
                    rospy.logerr(
                        "[vision_triggered_navigator] 步进扫描TF超过%.2fs未更新，"
                        "立即停车并延期当前锚点.", self.coverage_scan_pose_timeout)
                    return False

                if now >= deadline:
                    self.cmd_vel_pub.publish(Twist())
                    rospy.logerr(
                        "[vision_triggered_navigator] 步进扫描%d/%d超时，"
                        "progress=%.1f/%.1fdeg，延期当前锚点.",
                        step_index, len(steps), math.degrees(max_progress),
                        math.degrees(step_angle))
                    return False
                self.cmd_vel_pub.publish(twist)
                rate.sleep()

            self.cmd_vel_pub.publish(Twist())
            if self.triggered:
                return True
            pose = self._get_robot_pose(self.base_frame)
            heading = math.degrees(pose[2]) if pose is not None else float("nan")
            rospy.loginfo(
                "[vision_triggered_navigator] 步进%d/%d到位 heading=%.1fdeg，停车识别.",
                step_index, len(steps), heading)
            self._hold_scan_step(
                "步进{}/{}".format(step_index, len(steps)), step_started)
        return True

    def rotate(self, direction, duration):
        """
        发布角速度使机器人自转。
        direction: "left" 左转，"right" 右转
        duration: 保持时间（秒）
        """
        if duration <= 0:
            return True

        if self.coverage_search_mode:
            return self._step_scan(direction, duration)

        twist = Twist()
        direction_sign = 1.0 if direction == "left" else -1.0
        twist.angular.z = self.rotation_speed * direction_sign

        rospy.loginfo("[vision_triggered_navigator] 自转 %s, 速度 %.2f rad/s, 保持 %.2f s",
                      direction, twist.angular.z, duration)

        start = rospy.Time.now()
        time_limit = duration
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            # 轮询时检查是否被视觉/键盘触发打断，打断立即停止
            if elapsed >= time_limit or self.triggered:
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # 发送零速停止
        self.cmd_vel_pub.publish(Twist())
        return True

    def perform_rotations(self, rotations):
        """顺序执行一组自转动作"""
        for rot in rotations:
            if self.triggered:
                break
            direction = rot.get("direction", "left")
            duration = rot.get("duration", 0.0)
            if not self.rotate(direction, duration):
                return False
        return True

    def _visit_coverage_point(self, point, patrol_idx):
        """Visit one anchor once, using one nearby reachable pose when needed."""
        self.current_goal_timed_out = False
        self.current_goal_rotation_stall = False
        self.current_goal_needs_yaw_alignment = False
        self.current_goal_pose_accepted = False
        self.current_goal_no_progress = False
        x, y, yaw = exact_observation_target(point)
        self._publish_coverage_observation("navigating", patrol_idx, point)
        if self.triggered:
            return "triggered"

        known, max_cost, _blocked = self._coverage_pose_cost(x, y)
        exact_blocked = should_skip_coverage_anchor(
            known, max_cost, self.lethal_cost)
        if exact_blocked:
            rospy.logwarn(
                "[vision_triggered_navigator] scan point %d footprint is occupied "
                "(cost=%d); looking for a nearby reachable observation pose.",
                patrol_idx + 1, max_cost)

        result = None
        navigation_reached = False
        active_point = dict(point)
        if not exact_blocked:
            rospy.loginfo(
                "[vision_triggered_navigator] scan point %d: (%.4f, %.4f, %.4f)",
                patrol_idx + 1, x, y, yaw)
            for attempt in range(self.coverage_goal_retry_count + 1):
                result = self.send_goal(x, y, yaw)
                if self.triggered:
                    return "triggered"
                if self.current_goal_pose_accepted:
                    navigation_reached = True
                    break
                if self.current_goal_needs_yaw_alignment:
                    if (self._wait_navigation_idle() and
                            self._align_coverage_anchor_yaw((x, y, yaw))):
                        navigation_reached = True
                        break
                    if self.triggered:
                        self.cmd_vel_pub.publish(Twist())
                        return "triggered"
                    rospy.logwarn(
                        "[vision_triggered_navigator] scan point %d yaw alignment failed; "
                        "trying nearby observation pose.", patrol_idx + 1)
                    break
                if result == actionlib.GoalStatus.SUCCEEDED:
                    navigation_reached = True
                    break
                if (self.current_goal_rotation_stall and
                        attempt < self.coverage_goal_retry_count):
                    self.cmd_vel_pub.publish(Twist())
                    if not self._wait_navigation_idle():
                        break
                    self._publish_status("coverage_goal_retry")
                    rospy.logwarn(
                        "[vision_triggered_navigator] scan point %d rotation stall; "
                        "clear costmap and retry the calibrated pose once.",
                        patrol_idx + 1)
                    if not self._clear_costmaps_and_wait():
                        break
                    continue
                break

        if not navigation_reached and not self.triggered:
            if not self._wait_navigation_idle():
                self.cmd_vel_pub.publish(Twist())
            fallback = self._select_coverage_fallback(x, y, yaw)
            if fallback is not None:
                x, y, yaw = fallback
                active_point.update({"x": x, "y": y, "yaw": yaw})
                self._publish_coverage_observation(
                    "fallback_selected", patrol_idx, active_point)
                self._publish_status("coverage_fallback_selected")
                result = self.send_goal(x, y, yaw)
                if self.triggered:
                    return "triggered"
                if self.current_goal_pose_accepted:
                    navigation_reached = True
                elif self.current_goal_needs_yaw_alignment:
                    if (self._wait_navigation_idle() and
                            self._align_coverage_anchor_yaw((x, y, yaw))):
                        navigation_reached = True
                elif result == actionlib.GoalStatus.SUCCEEDED:
                    navigation_reached = True

        if not navigation_reached:
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn(
                "[vision_triggered_navigator] scan point %d could not be reached "
                "without prolonged oscillation (state=%s timeout=%s no_progress=%s "
                "rotation_stall=%s); skip this point and keep the mission moving.",
                patrol_idx + 1, str(result), self.current_goal_timed_out,
                self.current_goal_no_progress, self.current_goal_rotation_stall)
            return "skipped_failed"
        if not self._wait_navigation_idle():
            self.cmd_vel_pub.publish(Twist())
            rospy.logerr(
                "[vision_triggered_navigator] 精确锚点%d到达后move_base未释放控制权，禁止观察自转并进入下一原始点.",
                patrol_idx + 1)
            return "skipped_failed"

        self.cmd_vel_pub.publish(Twist())
        self._publish_coverage_observation("scanning", patrol_idx, active_point)
        initial_hold_at = rospy.get_time()
        self._hold_scan_step(
            "锚点{}初始朝向".format(patrol_idx + 1),
            initial_hold_at - max(self.target_bbox_stale,
                                  self.coverage_scan_dwell))
        if self.triggered:
            return "triggered"
        if not self.perform_rotations(active_point.get("rotations", [])):
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn(
                "[vision_triggered_navigator] 精确锚点%d步进扫描未完成，不重访，进入下一原始点.",
                patrol_idx + 1)
            return "skipped_scan_failed"
        if self.triggered:
            return "triggered"
        rospy.loginfo(
            "[vision_triggered_navigator] coverage anchor=%d state=covered exact=true",
            patrol_idx + 1)
        self._publish_coverage_observation("covered", patrol_idx, active_point)
        return "covered"

    def _odom_is_fresh(self):
        return (self.odom_yaw is not None and sensor_is_fresh(
            self.odom_received_at, rospy.get_time(), self.odom_stale))

    def _rotate_center_step(self, direction, target_angle,
                            abort_on_trigger=False):
        """Rotate one small odometry-closed-loop step, ramping through deadband."""
        if not self._odom_is_fresh():
            rospy.logerr("[vision_triggered_navigator] /odom超过%.2fs未更新，拒绝居中转动.",
                         self.odom_stale)
            return False

        direction = 1.0 if direction >= 0.0 else -1.0
        start_yaw = self.odom_yaw
        speed = self.target_center_start_speed
        window_started = rospy.get_time()
        window_yaw = start_yaw
        ramp_steps = int(math.ceil(
            max(0.0, self.target_center_step_max_speed - self.target_center_start_speed) /
            max(self.target_center_speed_increment, 1e-3)))
        deadline = rospy.get_time() + max(
            2.0,
            target_angle / max(self.target_center_start_speed, 0.01) +
            (ramp_steps + 2) * self.target_center_motion_window + 0.5)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if abort_on_trigger and self.triggered:
                self.cmd_vel_pub.publish(Twist())
                rospy.loginfo(
                    "[vision_triggered_navigator] OCR触发优先，中断当前航向步进.")
                return False
            if not self._odom_is_fresh():
                self.cmd_vel_pub.publish(Twist())
                rospy.logerr("[vision_triggered_navigator] 居中步进期间/odom失效，立即停车.")
                return False

            progress = abs(normalize_angle(self.odom_yaw - start_yaw))
            if progress >= target_angle:
                self.cmd_vel_pub.publish(Twist())
                rospy.loginfo(
                    "[vision_triggered_navigator] 居中步进完成 angle=%.2fdeg speed=%.2f",
                    math.degrees(progress), speed)
                return True

            now = rospy.get_time()
            if now - window_started >= self.target_center_motion_window:
                window_progress = abs(normalize_angle(self.odom_yaw - window_yaw))
                if window_progress < self.target_center_min_progress:
                    if speed + 1e-6 < self.target_center_step_max_speed:
                        speed = min(
                            self.target_center_step_max_speed,
                            speed + self.target_center_speed_increment)
                        rospy.logwarn(
                            "[vision_triggered_navigator] 角速度未越过底盘死区，提升至%.2frad/s.",
                            speed)
                    else:
                        self.cmd_vel_pub.publish(Twist())
                        rospy.logerr(
                            "[vision_triggered_navigator] 已到%.2frad/s仍无里程计转角，居中失败.",
                            speed)
                        return False
                window_started = now
                window_yaw = self.odom_yaw

            twist = Twist()
            twist.angular.z = direction * speed
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.logerr("[vision_triggered_navigator] 居中单步转动超时.")
        return False

    def _wait_fresh_target(self, previous_stamp, deadline):
        """Wait stopped for an OCR box newer than the one used for the last step."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            self.cmd_vel_pub.publish(Twist())
            if self.target_payload_at > previous_stamp:
                return True
            rate.sleep()
        return False

    def _centering_failure(self, message, status="centering_failed"):
        self.cmd_vel_pub.publish(Twist())
        self._publish_status(status)
        rospy.logerr("[vision_triggered_navigator] %s", message)
        return False

    def _wait_for_initial_recenter_target(self):
        """Use a fresh OCR box immediately, otherwise wait for one new sample."""
        if target_sample_is_fresh(
                self.target_error, self.target_payload_at,
                rospy.get_time(), self.target_bbox_stale):
            return True
        previous_stamp = self.target_payload_at
        deadline = rospy.get_time() + self.parking_recenter_initial_wait
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            self.cmd_vel_pub.publish(Twist())
            if (self.target_payload_at > previous_stamp and
                    target_sample_is_fresh(
                        self.target_error, self.target_payload_at,
                        rospy.get_time(), self.target_bbox_stale)):
                return True
            rate.sleep()
        return False

    def _center_visual_target(self, tolerance=None, timeout=None,
                              state="target_centering",
                              failure_state="centering_failed"):
        """Center the OCR box with stop-look odometry-closed-loop angular steps."""
        if not self.coverage_search_mode:
            return True
        tolerance = (self.target_center_tolerance if tolerance is None
                     else abs(float(tolerance)))
        timeout = (self.target_center_timeout if timeout is None
                   else max(0.1, float(timeout)))
        self._publish_status(state)
        if not self._wait_navigation_idle():
            return self._centering_failure(
                "move_base未释放控制权，拒绝视觉居中.", failure_state)
        if not self._odom_is_fresh():
            return self._centering_failure(
                "视觉居中开始前/odom不可用.", failure_state)

        deadline = rospy.get_time() + timeout
        centered_hits = 0
        last_centered_stamp = 0.0
        steering_sign = self.target_center_steering_sign
        reversed_once = False
        must_improve_after_reverse = False
        step_scale = 1.0

        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            age = rospy.get_time() - self.target_payload_at
            if self.target_error is None or age > self.target_bbox_stale:
                return self._centering_failure(
                    "目标框丢失或超过时效，停止而不恢复巡航.", failure_state)
            if not self._odom_is_fresh():
                return self._centering_failure(
                    "视觉居中期间/odom超过时效.", failure_state)

            if abs(self.target_error) <= tolerance:
                self.cmd_vel_pub.publish(Twist())
                if self.target_payload_at > last_centered_stamp:
                    last_centered_stamp = self.target_payload_at
                    centered_hits += 1
                    rospy.loginfo(
                        "[vision_triggered_navigator] target centered error=%.3f hits=%d/%d",
                        self.target_error, centered_hits, self.target_center_required_hits)
                if centered_hits >= self.target_center_required_hits:
                    self._hold_stopped(self.target_center_settle)
                    return True
                if not self._wait_fresh_target(last_centered_stamp, min(
                        deadline, rospy.get_time() + self.target_bbox_stale)):
                    return self._centering_failure(
                        "居中后未收到第二帧新目标框.", failure_state)
                continue

            centered_hits = 0
            before_error = float(self.target_error)
            before_stamp = self.target_payload_at
            step_angle = center_step_angle(
                before_error,
                tolerance,
                self.target_center_fine_threshold,
                self.target_center_coarse_step,
                self.target_center_fine_step,
            )
            step_angle *= step_scale
            direction = (1.0 if steering_sign >= 0.0 else -1.0) * math.copysign(
                1.0, before_error)
            rospy.loginfo(
                "[vision_triggered_navigator] 居中步进 error=%.3f step=%.1fdeg direction=%+.0f",
                before_error, math.degrees(step_angle), direction)
            if not self._rotate_center_step(direction, step_angle):
                return self._centering_failure(
                    "底盘未完成视觉居中步进.", failure_state)
            self._hold_stopped(self.target_center_settle)
            settled_at = rospy.get_time()
            if not self._wait_fresh_target(
                    max(before_stamp, settled_at),
                    min(deadline, rospy.get_time() + self.target_bbox_stale)):
                return self._centering_failure(
                    "步进后未收到新的目标框.", failure_state)

            after_error = float(self.target_error)
            improvement = abs(before_error) - abs(after_error)
            crossed_center = before_error * after_error < 0.0
            rospy.loginfo(
                "[vision_triggered_navigator] 居中反馈 before=%.3f after=%.3f improvement=%.3f",
                before_error, after_error, improvement)
            if crossed_center:
                # The control direction was correct: the target crossed the image
                # center. Reversing the steering calibration here would command a
                # second step in the same physical direction. Keep the mapping and
                # reduce the next stop-look step so noisy multi-scale OCR boxes can
                # converge instead of oscillating around the center.
                must_improve_after_reverse = False
                step_scale = max(0.25, step_scale * 0.5)
                rospy.logwarn(
                    "[vision_triggered_navigator] 目标已跨过画面中心，保持转向映射并将下一步缩短为%.2f倍.",
                    step_scale)
            elif must_improve_after_reverse:
                if improvement <= 0.0:
                    return self._centering_failure(
                        "自动反向后误差仍未减小，停止居中.", failure_state)
                must_improve_after_reverse = False
            elif improvement < -self.target_center_reverse_threshold:
                if reversed_once:
                    return self._centering_failure(
                        "目标误差再次增大，停止居中.", failure_state)
                steering_sign *= -1.0
                reversed_once = True
                must_improve_after_reverse = True
                rospy.logwarn(
                    "[vision_triggered_navigator] 首次步进使误差增大，自动反转居中方向为%+.0f.",
                    steering_sign)

        return self._centering_failure(
            "目标居中超过%.1fs，车辆保持停车." % timeout, failure_state)

    def _center_parking_target_continuous(self):
        """Translate under the sign while keeping the chassis normal to the wall."""
        self._publish_status("parking_recenter")
        if (not self._wait_navigation_idle() or not self._odom_is_fresh() or
                self.odom_pose is None):
            return False

        odom_frame = self.odom_frame_from_msg or self.odom_frame
        wall_geometry = self._transform_wall_geometry(odom_frame)
        if wall_geometry is None:
            rospy.logwarn(
                "[vision_triggered_navigator] parking recenter has no wall geometry; "
                "preserving the locked tangent goal.")
            return False
        _wall_point, inward_normal = wall_geometry

        deadline = rospy.get_time() + self.parking_recenter_timeout
        stable_since = None
        centered_hits = 0
        last_centered_stamp = 0.0
        filtered_error = None
        start_pose = tuple(self.odom_pose)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            now = rospy.get_time()
            if not self._odom_is_fresh() or self.odom_pose is None:
                self.cmd_vel_pub.publish(Twist())
                return False
            pose = self.odom_pose
            travelled = math.hypot(
                pose[0] - start_pose[0], pose[1] - start_pose[1])
            if travelled > self.parking_recenter_max_travel:
                self.cmd_vel_pub.publish(Twist())
                rospy.logwarn(
                    "[vision_triggered_navigator] parking lateral recenter reached "
                    "travel limit %.3fm; preserving the best centered pose.",
                    travelled)
                return False
            if not target_sample_is_fresh(
                    self.target_error, self.target_payload_at,
                    now, self.target_bbox_stale):
                self.cmd_vel_pub.publish(Twist())
                stable_since = None
                rate.sleep()
                continue

            raw_error = float(self.target_error)
            filtered_error = (raw_error if filtered_error is None else
                               0.55 * filtered_error + 0.45 * raw_error)

            fit = self._wall_fit_for_pose(inward_normal, pose)
            if fit is None:
                fit = self._predicted_wall_fit(pose)
            if fit is None:
                self.cmd_vel_pub.publish(Twist())
                stable_since = None
                centered_hits = 0
                rate.sleep()
                continue
            yaw_error = normalize_angle(float(fit["normal_angle"]))

            centered = abs(filtered_error) <= self.parking_recenter_tolerance
            aligned = abs(yaw_error) <= self.parking_recenter_yaw_tolerance
            if centered and aligned:
                self.cmd_vel_pub.publish(Twist())
                if self.target_payload_at > last_centered_stamp:
                    last_centered_stamp = self.target_payload_at
                    centered_hits += 1
                if stable_since is None:
                    stable_since = now
                elif (centered_hits >= self.parking_recenter_required_hits and
                      now - stable_since >= self.parking_recenter_stable_sec):
                    rospy.loginfo(
                        "[vision_triggered_navigator] parking lateral center stable "
                        "error=%+.3f yaw=%+.2fdeg hits=%d travel=%.3fm",
                        filtered_error, math.degrees(yaw_error), centered_hits,
                        travelled)
                    return True
                rate.sleep()
                continue

            stable_since = None
            centered_hits = 0
            twist = Twist()
            if not centered:
                magnitude = min(
                    self.parking_recenter_max_lateral,
                    self.parking_recenter_lateral_kp * abs(filtered_error))
                magnitude = max(
                    min(self.parking_recenter_min_lateral,
                        self.parking_recenter_max_lateral), magnitude)
                twist.linear.y = (
                    self.parking_recenter_lateral_sign *
                    math.copysign(magnitude, filtered_error))
                twist.linear.y, side_clearance, side_guard = (
                    self._guard_parking_lateral(twist.linear.y))
            else:
                side_clearance = float("inf")
                side_guard = "centered"
            if not aligned:
                twist.angular.z = max(
                    -self.parking_recenter_max_yaw,
                    min(self.parking_recenter_max_yaw,
                        self.parking_recenter_yaw_kp * yaw_error))
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo_throttle(
                0.4,
                "[vision_triggered_navigator] parking lateral center "
                "error=%+.3f filtered=%+.3f yaw=%+.2fdeg "
                "cmd=(vy=%+.3f,wz=%+.3f) side=%.3f guard=%s",
                raw_error, filtered_error, math.degrees(yaw_error),
                twist.linear.y, twist.angular.z, side_clearance, side_guard)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.logwarn(
            "[vision_triggered_navigator] parking lateral recenter reached %.1fs "
            "limit; preserving the best centered wall goal.",
            self.parking_recenter_timeout)
        return False

    def _guard_parking_lateral(self, requested):
        """Scale one lateral parking command using the obstacle-side laser sector."""
        requested = float(requested)
        if abs(requested) <= 1e-6:
            return 0.0, float("inf"), "idle"
        clearance = self.scan_left_min if requested > 0.0 else self.scan_right_min
        if clearance is None or not math.isfinite(clearance):
            return requested, float("inf"), "no_return"
        clearance = float(clearance)
        if clearance <= self.parking_recenter_side_stop:
            return 0.0, clearance, "blocked"
        if clearance >= self.parking_recenter_side_slow:
            return requested, clearance, "clear"
        scale = ((clearance - self.parking_recenter_side_stop) /
                 (self.parking_recenter_side_slow -
                  self.parking_recenter_side_stop))
        return requested * max(0.0, min(1.0, scale)), clearance, "slowed"

    def _hold_stopped(self, duration):
        deadline = rospy.get_time() + max(0.0, float(duration))
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            self.cmd_vel_pub.publish(Twist())
            rate.sleep()

    def _transform_map_pose(self, target_frame, pose):
        """Transform one map pose into target_frame, returning an xyz tuple."""
        stamped = PoseStamped()
        stamped.header.frame_id = self.map_frame
        stamped.header.stamp = rospy.Time(0)
        stamped.pose.position.x = float(pose[0])
        stamped.pose.position.y = float(pose[1])
        stamped.pose.orientation = euler_to_quaternion(float(pose[2]))
        try:
            self.tf_listener.waitForTransform(
                target_frame, self.map_frame, rospy.Time(0), rospy.Duration(0.5))
            transformed = self.tf_listener.transformPose(target_frame, stamped)
            return (
                transformed.pose.position.x,
                transformed.pose.position.y,
                quaternion_to_yaw(transformed.pose.orientation),
            )
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as exc:
            rospy.logerr(
                "[vision_triggered_navigator] 无法将map停泊位姿转换到%s: %s",
                target_frame, str(exc))
            return None

    def _transform_wall_geometry(self, target_frame):
        """Transform wall centre and inward normal from map into target_frame."""
        if self.parking_wall_point is None or self.parking_inward_normal is None:
            return None
        wx, wy = self.parking_wall_point
        nx, ny = self.parking_inward_normal
        wall = self._transform_map_pose(target_frame, (wx, wy, 0.0))
        inward = self._transform_map_pose(
            target_frame, (wx + nx, wy + ny, 0.0))
        if wall is None or inward is None:
            return None
        normal_x = inward[0] - wall[0]
        normal_y = inward[1] - wall[1]
        length = math.hypot(normal_x, normal_y)
        if length <= 1e-6:
            rospy.logerr("[vision_triggered_navigator] odom墙面法向量长度为0.")
            return None
        return (wall[0], wall[1]), (normal_x / length, normal_y / length)

    def _make_move_base_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation = euler_to_quaternion(float(yaw))
        return goal

    def _navigate_to_parking_staging(self, goal):
        """Use move_base only to reach a safe staging pose, with spin watchdog."""
        x, y, yaw = [float(value) for value in goal]
        pose = self._get_robot_pose(self.base_frame)
        if pose is None:
            rospy.logerr("[vision_triggered_navigator] 无法获取预停点起始位姿.")
            return False
        if staging_pose_reached(
                pose, (x, y, yaw), self.parking_staging_acceptance,
                self.parking_staging_yaw_tolerance):
            rospy.loginfo(
                "[vision_triggered_navigator] 已满足预停点位置%.2fm/航向%.3frad，跳过move_base.",
                self.parking_staging_acceptance,
                self.parking_staging_yaw_tolerance)
            return self._wait_navigation_idle()

        rospy.loginfo(
            "[vision_triggered_navigator] 发送预停点: x=%.4f y=%.4f yaw=%.4f timeout=%.1fs",
            x, y, yaw, self.parking_staging_timeout)
        self.move_base_client.send_goal(self._make_move_base_goal(x, y, yaw))
        started = rospy.get_time()
        window_started = started
        window_pose = pose
        progress_started = started
        progress_pose = pose
        last_yaw = pose[2]
        yaw_accumulated = 0.0
        reached = False
        failure_reason = ""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pose = self._get_robot_pose(self.base_frame)
            if pose is not None:
                distance = math.hypot(x - pose[0], y - pose[1])
                yaw_accumulated += abs(normalize_angle(pose[2] - last_yaw))
                last_yaw = pose[2]
                yaw_error = abs(normalize_angle(yaw - pose[2]))
                if staging_pose_reached(
                        pose, (x, y, yaw), self.parking_staging_acceptance,
                        self.parking_staging_yaw_tolerance):
                    reached = True
                    break
                progress_moved = math.hypot(
                    pose[0] - progress_pose[0], pose[1] - progress_pose[1])
                if progress_moved >= self.parking_staging_min_progress:
                    progress_started = rospy.get_time()
                    progress_pose = pose
                elif (rospy.get_time() - progress_started >=
                      self.parking_staging_no_progress_timeout):
                    failure_reason = (
                        "预停点连续%.1fs无有效位移(distance=%.3f)" %
                        (self.parking_staging_no_progress_timeout, distance))
                    break
                if rospy.get_time() - window_started >= self.parking_staging_watchdog_window:
                    moved = math.hypot(pose[0] - window_pose[0],
                                       pose[1] - window_pose[1])
                    if staging_motion_is_rotation_stall(
                            moved, yaw_accumulated,
                            self.parking_staging_min_progress,
                            self.parking_staging_max_rotation):
                        failure_reason = (
                            "预停点出现原地旋转: %.1fs位移%.3fm累计转角%.1fdeg" %
                            (self.parking_staging_watchdog_window, moved,
                             math.degrees(yaw_accumulated)))
                        break
                    window_started = rospy.get_time()
                    window_pose = pose
                    yaw_accumulated = 0.0

            state = self.move_base_client.get_state()
            if state not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                failure_reason = (
                    "move_base预停点提前结束(state=%s distance=%.3f yaw_error=%.3f)" %
                    (str(state), distance if pose is not None else float("nan"),
                     yaw_error if pose is not None else float("nan")))
                break
            if rospy.get_time() - started >= self.parking_staging_timeout:
                failure_reason = "预停点导航超过%.1fs" % self.parking_staging_timeout
                break
            rate.sleep()

        self.move_base_client.cancel_goal()
        idle = self._wait_navigation_idle(timeout=2.0)
        self.cmd_vel_pub.publish(Twist())
        if reached and idle:
            rospy.loginfo("[vision_triggered_navigator] 预停点交接完成，move_base已释放控制权.")
            return True
        if not idle:
            failure_reason = "move_base未释放/cmd_vel控制权"
        rospy.logerr("[vision_triggered_navigator] parking_staging_failed: %s",
                     failure_reason or "未知原因")
        return False

    def _remember_wall_fit(self, fit, pose):
        """Low-pass one continuous wall fit and remember its odometry pose."""
        remembered = dict(fit)
        predicted_previous = self._predicted_wall_fit(pose)
        if (predicted_previous is not None and wall_fit_is_continuous(
                remembered, predicted_previous,
                max(0.10, self.parking_wall_fit_max_distance_jump),
                max(math.radians(15.0),
                    self.parking_wall_fit_max_normal_jump))):
            alpha = self.parking_wall_fit_filter_alpha
            remembered["distance"] = (
                alpha * float(remembered["distance"]) +
                (1.0 - alpha) * float(predicted_previous["distance"]))
            angle_delta = normalize_angle(
                float(remembered["normal_angle"]) -
                float(predicted_previous["normal_angle"]))
            remembered["normal_angle"] = normalize_angle(
                float(predicted_previous["normal_angle"]) +
                alpha * angle_delta)
        remembered["predicted"] = False
        self.parking_last_wall_fit = remembered
        self.parking_last_wall_fit_at = rospy.get_time()
        self.parking_last_wall_fit_pose = tuple(pose)
        return remembered

    def _predicted_wall_fit(self, pose):
        """Propagate a briefly missing wall fit with odometry, never with time alone."""
        previous = self.parking_last_wall_fit
        previous_pose = self.parking_last_wall_fit_pose
        age = rospy.get_time() - self.parking_last_wall_fit_at
        if (previous is None or previous_pose is None or
                age < 0.0 or age > self.parking_wall_fit_grace):
            return None
        px, py, pyaw = previous_pose
        normal_odom_angle = pyaw + float(previous["normal_angle"])
        nx = math.cos(normal_odom_angle)
        ny = math.sin(normal_odom_angle)
        wall_x = px + nx * float(previous["distance"])
        wall_y = py + ny * float(previous["distance"])
        predicted_distance = ((wall_x - pose[0]) * nx +
                              (wall_y - pose[1]) * ny)
        if predicted_distance <= 0.0:
            return None
        predicted = dict(previous)
        predicted["distance"] = predicted_distance
        predicted["normal_angle"] = normalize_angle(
            normal_odom_angle - pose[2])
        predicted["predicted"] = True
        predicted["age"] = age
        return predicted

    def _wall_fit_for_pose(self, inward_normal_odom, pose):
        """Fit the physical wall and reject lines inconsistent with the map side."""
        fit = fit_wall_line(
            self.scan_wall_points,
            self.parking_wall_fit_min_points,
            self.parking_wall_fit_min_span,
            self.parking_wall_fit_max_residual,
        )
        outward_angle_odom = math.atan2(
            -inward_normal_odom[1], -inward_normal_odom[0])
        expected_in_base = normalize_angle(outward_angle_odom - pose[2])
        if (fit and wall_fit_matches_expected(
                fit, expected_in_base,
                self.parking_wall_fit_max_normal_error)):
            return self._remember_wall_fit(fit, pose)
        # Once a long wall has been acquired, close range may crop its visible
        # span below 25 cm.  Re-fit with the near threshold, but accept only a
        # geometrically continuous line so a cone cluster cannot take over.
        now = rospy.get_time()
        if (self.parking_last_wall_fit is not None and sensor_is_fresh(
                self.parking_last_wall_fit_at, now,
                self.parking_wall_fit_grace)):
            near_fit = fit_wall_line(
                self.scan_wall_points,
                self.parking_wall_fit_min_points,
                self.parking_wall_fit_near_min_span,
                self.parking_wall_fit_max_residual,
            )
            if (near_fit and wall_fit_matches_expected(
                    near_fit, expected_in_base,
                    self.parking_wall_fit_max_normal_error) and
                    wall_fit_is_continuous(
                        near_fit, self.parking_last_wall_fit,
                        self.parking_wall_fit_max_distance_jump,
                        self.parking_wall_fit_max_normal_jump)):
                rospy.loginfo_throttle(
                    1.0,
                    "[vision_triggered_navigator] 近墙连续拟合启用: span=%.3fm distance=%.3fm.",
                    near_fit["span"], near_fit["distance"])
                return self._remember_wall_fit(near_fit, pose)
        return None

    def _run_parking_docking(self, map_goal):
        """Finish against the measured wall, locking only tangent position in odom."""
        if not self._wait_navigation_idle(timeout=2.0):
            rospy.logerr("[vision_triggered_navigator] move_base仍占用控制权，拒绝直接停泊.")
            return False
        if not self._odom_is_fresh() or self.odom_pose is None:
            rospy.logerr("[vision_triggered_navigator] /odom不新鲜，拒绝直接停泊.")
            return False
        if (not sensor_is_fresh(self.scan_received_at, rospy.get_time(),
                                self.scan_stale) or
                self.scan_front_min is None):
            rospy.logerr("[vision_triggered_navigator] /scan不新鲜或前向无有效量程，拒绝直接停泊.")
            return False

        odom_frame = self.odom_frame_from_msg or self.odom_frame
        target = self._transform_map_pose(odom_frame, map_goal)
        wall_geometry = self._transform_wall_geometry(odom_frame)
        if target is None or wall_geometry is None:
            return False
        wall_point, inward_normal = wall_geometry
        outward_normal = (-inward_normal[0], -inward_normal[1])
        tangent = (-outward_normal[1], outward_normal[0])
        desired_wall_distance = max(
            self.parking_min_wall_distance,
            self.parking_goal_offset + self.parking_normal_offset)
        rospy.loginfo(
            "[vision_triggered_navigator] 锁定墙面停泊 frame=%s tangent_target=(%.4f,%.4f) wall_distance=%.3f",
            odom_frame, target[0], target[1], desired_wall_distance)

        deadline = rospy.get_time() + self.parking_docking_timeout
        stable_since = None
        rotation_window_started = 0.0
        rotation_window_yaw = None
        self.parking_final_wall_fit = None
        self.parking_final_tangent_error = None
        self.parking_last_wall_fit = None
        self.parking_last_wall_fit_at = 0.0
        self.parking_last_wall_fit_pose = None
        self.parking_failure_status = "parking_docking_failed"
        docking_status_sent = False
        fit_wait_started = rospy.get_time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if not self._odom_is_fresh() or self.odom_pose is None:
                self.cmd_vel_pub.publish(Twist())
                rospy.logerr("[vision_triggered_navigator] 停泊期间/odom超过%.2fs未更新.",
                             self.odom_stale)
                return False
            if (not sensor_is_fresh(self.scan_received_at, rospy.get_time(),
                                    self.scan_stale) or
                    self.scan_front_min is None):
                self.cmd_vel_pub.publish(Twist())
                rospy.logerr("[vision_triggered_navigator] 停泊期间/scan超过%.2fs未更新.",
                             self.scan_stale)
                return False

            pose = self.odom_pose
            fit = self._wall_fit_for_pose(inward_normal, pose)
            if fit is None:
                fit = self._predicted_wall_fit(pose)
                if fit is None:
                    if (rospy.get_time() - fit_wait_started <=
                            self.parking_wall_fit_grace):
                        self.cmd_vel_pub.publish(Twist())
                        rate.sleep()
                        continue
                    self.cmd_vel_pub.publish(Twist())
                    rospy.logerr(
                        "[vision_triggered_navigator] parking_wall_fit_failed: "
                        "wall was unavailable for %.2fs.",
                        self.parking_wall_fit_grace)
                    self.parking_failure_status = "parking_wall_fit_failed"
                    return False
                rospy.logwarn_throttle(
                    0.5,
                    "[vision_triggered_navigator] parking wall fit bridged "
                    "by odometry age=%.2fs distance=%.3fm yaw=%+.3f.",
                    fit.get("age", 0.0), fit["distance"],
                    fit["normal_angle"])
            else:
                fit_wait_started = rospy.get_time()
            if not docking_status_sent:
                self._publish_status("parking_docking")
                docking_status_sent = True

            tangent_error = ((target[0] - pose[0]) * tangent[0] +
                             (target[1] - pose[1]) * tangent[1])
            normal_error = fit["distance"] - desired_wall_distance
            yaw_error = normalize_angle(fit["normal_angle"])
            errors = (normal_error, tangent_error, yaw_error)
            if docking_within_tolerance(
                    errors,
                    self.parking_dock_normal_tolerance,
                    self.parking_dock_tangent_tolerance,
                    self.parking_dock_yaw_tolerance):
                self.cmd_vel_pub.publish(Twist())
                if fit.get("predicted", False):
                    stable_since = None
                    rate.sleep()
                    continue
                if stable_since is None:
                    stable_since = rospy.get_time()
                elif rospy.get_time() - stable_since >= self.parking_dock_stable_sec:
                    rospy.loginfo(
                        "[vision_triggered_navigator] 实墙停泊收敛 stable=%.2fs errors=(normal=%.3f tangent=%.3f yaw=%.3f)",
                        self.parking_dock_stable_sec,
                        errors[0], errors[1], errors[2])
                    self.parking_final_wall_fit = dict(fit)
                    self.parking_final_tangent_error = tangent_error
                    return True
                rate.sleep()
                continue
            stable_since = None

            if fit["distance"] < self.parking_min_wall_distance:
                self.cmd_vel_pub.publish(Twist())
                rospy.logerr(
                    "[vision_triggered_navigator] 实测墙距%.3fm小于硬限%.3fm，立即停车.",
                    fit["distance"], self.parking_min_wall_distance)
                return False
            # A return much closer than the fitted wall is an obstacle, not wall data.
            lidar_base_distance = lidar_base_wall_distance(
                self.scan_front_min, self.parking_lidar_forward_offset)
            if (lidar_base_distance < self.parking_lidar_stop_distance or
                    lidar_base_distance < fit["distance"] - 0.08):
                self.cmd_vel_pub.publish(Twist())
                rospy.logerr(
                    "[vision_triggered_navigator] 雷达近障碍触发停车: base等效=%.3fm wall_fit=%.3f raw=%.3f.",
                    lidar_base_distance, fit["distance"],
                    self.scan_front_min)
                return False

            command = wall_frame_docking_command(
                normal_error, tangent_error, yaw_error,
                self.parking_dock_normal_tolerance,
                self.parking_dock_tangent_tolerance,
                self.parking_dock_yaw_tolerance,
                self.parking_dock_max_x,
                self.parking_dock_max_y,
                self.parking_dock_max_yaw,
                self.parking_dock_min_yaw,
                self.parking_dock_translation_yaw_gate,
                self.parking_dock_forward_yaw_gate,
                self.parking_dock_forward_tangent_gate,
            )
            if abs(command[2]) > 0.0:
                if rotation_window_yaw is None:
                    rotation_window_yaw = pose[2]
                    rotation_window_started = rospy.get_time()
                elif rospy.get_time() - rotation_window_started >= 0.6:
                    progress = abs(normalize_angle(pose[2] - rotation_window_yaw))
                    if progress < math.radians(0.5):
                        self.cmd_vel_pub.publish(Twist())
                        rospy.logerr(
                            "[vision_triggered_navigator] parking_docking_failed: angular.z=%.3f持续0.6s但转角仅%.2fdeg.",
                            command[2], math.degrees(progress))
                        return False
                    rotation_window_yaw = pose[2]
                    rotation_window_started = rospy.get_time()
            else:
                rotation_window_yaw = None
                rotation_window_started = 0.0
            guarded_y, side_clearance, side_guard = self._guard_parking_lateral(
                command[1])
            if side_guard in ("blocked", "slowed"):
                rospy.logwarn_throttle(
                    0.5,
                    "[vision_triggered_navigator] docking lateral guard=%s "
                    "requested=%+.3f applied=%+.3f clearance=%.3fm",
                    side_guard, command[1], guarded_y, side_clearance)
            twist = Twist()
            twist.linear.x = command[0]
            twist.linear.y = guarded_y
            twist.angular.z = command[2]
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo_throttle(
                0.5,
                "[vision_triggered_navigator] docking errors=(normal=%+.3f tangent=%+.3f yaw=%+.3f) cmd=(%+.3f,%+.3f,%+.3f) wall_fit=%.3f span=%.3f residual=%.4f inliers=%d",
                errors[0], errors[1], errors[2], command[0], command[1], command[2],
                fit["distance"], fit["span"], fit["residual"], fit["inliers"])
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.logerr("[vision_triggered_navigator] 停泊闭环超过%.1fs仍未收敛.",
                     self.parking_docking_timeout)
        return False

    def _update_dynamic_parameters(self, namespace, bools=None, doubles=None):
        """Update named parameters without caching the full config description."""
        service_name = namespace.rstrip("/") + "/set_parameters"
        rospy.wait_for_service(service_name, timeout=3.0)
        request = ReconfigureRequest()
        request.config.bools = [
            BoolParameter(name=name, value=bool(value))
            for name, value in (bools or {}).items()
        ]
        request.config.doubles = [
            DoubleParameter(name=name, value=float(value))
            for name, value in (doubles or {}).items()
        ]
        return rospy.ServiceProxy(service_name, Reconfigure)(request).config

    def _disable_move_base_recovery_for_coverage(self):
        """Deterministically prevent move_base from executing recovery spins."""
        if not self.coverage_search_mode:
            return True
        try:
            self._saved_move_base_recovery = {
                "recovery_behavior_enabled": bool(rospy.get_param(
                    self.move_base_reconfigure_ns +
                    "/recovery_behavior_enabled", True)),
                "clearing_rotation_allowed": bool(rospy.get_param(
                    self.move_base_reconfigure_ns +
                    "/clearing_rotation_allowed", True)),
            }
            self._saved_teb_oscillation_recovery = bool(rospy.get_param(
                self.local_planner_reconfigure_ns +
                "/oscillation_recovery", True))
            updated = self._update_dynamic_parameters(
                self.move_base_reconfigure_ns,
                bools={
                    "recovery_behavior_enabled": False,
                    "clearing_rotation_allowed": False,
                })
            planner_updated = self._update_dynamic_parameters(
                self.local_planner_reconfigure_ns,
                bools={"oscillation_recovery": False})
            updated_bools = {item.name: item.value for item in updated.bools}
            planner_bools = {
                item.name: item.value for item in planner_updated.bools
            }
            if (bool(updated_bools.get("recovery_behavior_enabled", False)) or
                    bool(updated_bools.get("clearing_rotation_allowed", False)) or
                    bool(planner_bools.get("oscillation_recovery", False))):
                raise RuntimeError("move_base rejected recovery disable request")
            self._publish_status("coverage_recovery_disabled")
            rospy.logwarn(
                "[vision_triggered_navigator] 任务2期间已临时关闭move_base恢复行为和清障旋转；退出时自动恢复.")
            return True
        except Exception as exc:
            rospy.logerr(
                "[vision_triggered_navigator] 无法关闭move_base旋转恢复，拒绝启动任务2运动: %s",
                str(exc))
            self._restore_move_base_recovery()
            return False

    def _restore_move_base_recovery(self):
        saved = self._saved_move_base_recovery
        saved_teb = self._saved_teb_oscillation_recovery
        self._saved_move_base_recovery = None
        self._saved_teb_oscillation_recovery = None
        try:
            if saved:
                self._update_dynamic_parameters(
                    self.move_base_reconfigure_ns, bools=saved)
                rospy.loginfo(
                    "[vision_triggered_navigator] 已恢复move_base恢复配置 recovery=%s clearing_rotation=%s.",
                    saved["recovery_behavior_enabled"],
                    saved["clearing_rotation_allowed"])
            if saved_teb is not None:
                self._update_dynamic_parameters(
                    self.local_planner_reconfigure_ns,
                    bools={"oscillation_recovery": saved_teb})
                rospy.loginfo(
                    "[vision_triggered_navigator] 已恢复TEB oscillation_recovery=%s.",
                    saved_teb)
        except Exception as exc:
            rospy.logerr(
                "[vision_triggered_navigator] 恢复move_base恢复配置失败: %s",
                str(exc))

    def _tighten_final_tolerances(self):
        """Temporarily tighten TEB only for the 50cm task2 parking goal."""
        if not self.validate_parking_box:
            return True
        try:
            self._saved_planner_tolerances = {
                "xy_goal_tolerance": float(rospy.get_param(
                    self.local_planner_reconfigure_ns +
                    "/xy_goal_tolerance", 0.15)),
                "yaw_goal_tolerance": float(rospy.get_param(
                    self.local_planner_reconfigure_ns +
                    "/yaw_goal_tolerance", 0.1)),
                "free_goal_vel": bool(rospy.get_param(
                    self.local_planner_reconfigure_ns +
                    "/free_goal_vel", False)),
            }
            updated = self._update_dynamic_parameters(
                self.local_planner_reconfigure_ns,
                bools={"free_goal_vel": False},
                doubles={
                    "xy_goal_tolerance": self.parking_xy_tolerance,
                    "yaw_goal_tolerance": self.parking_yaw_tolerance,
                })
            updated_doubles = {
                item.name: item.value for item in updated.doubles
            }
            rospy.loginfo(
                "[vision_triggered_navigator] 最终停泊临时收紧TEB容差 xy=%.3f yaw=%.3f",
                float(updated_doubles.get(
                    "xy_goal_tolerance", self.parking_xy_tolerance)),
                float(updated_doubles.get(
                    "yaw_goal_tolerance", self.parking_yaw_tolerance)))
            return True
        except Exception as exc:
            self._saved_planner_tolerances = None
            rospy.logerr("[vision_triggered_navigator] 无法收紧最终停泊TEB容差: %s", str(exc))
            return False

    def _restore_final_tolerances(self):
        """Restore navigation-team TEB values after task2 parking."""
        saved = self._saved_planner_tolerances
        self._saved_planner_tolerances = None
        if not saved:
            return
        try:
            self._update_dynamic_parameters(
                self.local_planner_reconfigure_ns,
                bools={"free_goal_vel": saved["free_goal_vel"]},
                doubles={
                    "xy_goal_tolerance": saved["xy_goal_tolerance"],
                    "yaw_goal_tolerance": saved["yaw_goal_tolerance"],
                })
            rospy.loginfo(
                "[vision_triggered_navigator] 已恢复TEB容差 xy=%.3f yaw=%.3f",
                float(saved["xy_goal_tolerance"]),
                float(saved["yaw_goal_tolerance"]))
        except Exception as exc:
            rospy.logerr("[vision_triggered_navigator] 恢复TEB容差失败: %s", str(exc))

    def _validate_parking_pose(self):
        """Require the full configured footprint to be inside the 50cm box."""
        if not self.validate_parking_box:
            return True
        if (self.parking_final_wall_fit is not None and
                self.parking_final_tangent_error is not None):
            fit = self.parking_final_wall_fit
            # Local wall frame: +x points inward, +y follows the wall.  When
            # aligned the base faces the wall, hence yaw=pi.
            local_pose = (
                float(fit["distance"]),
                -float(self.parking_final_tangent_error),
                math.pi - float(fit["normal_angle"]),
            )
            diagnostics = parking_footprint_margins(
                local_pose, (0.0, 0.0), (1.0, 0.0),
                self.parking_box_width, self.parking_box_depth,
                self.footprint_half_length, self.footprint_half_width, 0.0)
            minimum_margin = min(
                float(diagnostics.get("near_margin", float("-inf"))),
                float(diagnostics.get("far_margin", float("-inf"))),
                float(diagnostics.get("side_margin", float("-inf"))))
            valid = (bool(diagnostics.get("inside")) and
                     minimum_margin >= self.parking_required_margin)
            rospy.loginfo(
                "[vision_triggered_navigator] 实墙停泊验证 distance=%.3f tangent_error=%+.3f yaw_error=%+.3f margins(near=%.3f far=%.3f side=%.3f min=%.3f required=%.3f) valid=%s",
                fit["distance"], self.parking_final_tangent_error,
                fit["normal_angle"], diagnostics.get("near_margin", float("nan")),
                diagnostics.get("far_margin", float("nan")),
                diagnostics.get("side_margin", float("nan")), minimum_margin,
                self.parking_required_margin, valid)
            return valid
        if self.parking_wall_point is None or self.parking_inward_normal is None:
            rospy.logerr("[vision_triggered_navigator] 缺少停泊框几何，无法验证.")
            return False
        pose = self._get_robot_pose(self.base_frame)
        if pose is None:
            rospy.logerr("[vision_triggered_navigator] 无法获取最终位姿，停泊验证失败.")
            return False
        diagnostics = parking_footprint_margins(
            pose,
            self.parking_wall_point,
            self.parking_inward_normal,
            self.parking_box_width,
            self.parking_box_depth,
            self.footprint_half_length,
            self.footprint_half_width,
            self.parking_validation_margin,
        )
        # diagnostics margins already exclude the legacy validation margin;
        # add it back so parking_required_margin is the physical box margin.
        minimum_margin = self.parking_validation_margin + min(
            float(diagnostics.get("near_margin", float("-inf"))),
            float(diagnostics.get("far_margin", float("-inf"))),
            float(diagnostics.get("side_margin", float("-inf"))),
        )
        valid = (bool(diagnostics.get("inside")) and
                 minimum_margin >= self.parking_required_margin)
        rospy.loginfo(
            "[vision_triggered_navigator] 停泊框验证 wall=%s pose=(%.4f, %.4f, %.4f) "
            "box=%.2fx%.2f normal=[%.3f,%.3f] tangent_abs=%.3f "
            "error(normal=%+.3f tangent=%+.3f) "
            "margins(near=%.3f far=%.3f side=%.3f min=%.3f required=%.3f) full_footprint_inside=%s",
            self.parking_wall_name or "unknown",
            pose[0], pose[1], pose[2], self.parking_box_width,
            self.parking_box_depth,
            float(diagnostics.get("normal_min", float("nan"))),
            float(diagnostics.get("normal_max", float("nan"))),
            float(diagnostics.get("tangent_abs_max", float("nan"))),
            float(diagnostics.get("normal_error", float("nan"))),
            float(diagnostics.get("tangent_error", float("nan"))),
            float(diagnostics.get("near_margin", float("nan"))),
            float(diagnostics.get("far_margin", float("nan"))),
            float(diagnostics.get("side_margin", float("nan"))),
            minimum_margin, self.parking_required_margin,
            valid)
        for index, corner in enumerate(diagnostics.get("corners", []), 1):
            rospy.loginfo(
                "[vision_triggered_navigator] footprint corner%d map=(%.3f,%.3f) "
                "normal=%.3f tangent=%.3f margins(near=%.3f far=%.3f side=%.3f)",
                index, corner["x"], corner["y"],
                corner["normal"], corner["tangent"],
                corner["near_margin"], corner["far_margin"],
                corner["side_margin"])
        return valid

    # ------------------------------------------------------------------
    # 视觉触发目标计算
    # ------------------------------------------------------------------
    def compute_vision_goal(self):
        """
        根据 base_link 车头正方向射线与实测四边形围墙求交，
        沿墙法向和切向连续计算停车目标，返回 (x, y, yaw)。
        """
        pose = self._get_robot_pose(self.base_frame)
        if pose is None:
            rospy.logerr("[vision_triggered_navigator] 无法获取机器人位置，无法计算视觉目标.")
            return None
        px, py, robot_yaw = pose
        use_target_bearing = (
            self.coverage_search_mode and
            target_sample_is_fresh(
                self.target_error, self.target_payload_at,
                rospy.get_time(), self.target_bbox_stale)
        )
        if use_target_bearing:
            yaw = target_bearing_yaw(
                robot_yaw, self.target_error, self.camera_horizontal_fov,
                self.camera_bearing_sign, self.camera_boresight_yaw_offset)
            rospy.loginfo(
                "[vision_triggered_navigator] 锁定厂牌方位 robot_yaw=%.1fdeg "
                "image_error=%+.3f bearing_offset=%+.1fdeg target_yaw=%.1fdeg",
                math.degrees(robot_yaw), self.target_error,
                math.degrees(normalize_angle(yaw - robot_yaw)),
                math.degrees(yaw))
        else:
            yaw = normalize_angle(robot_yaw + self.camera_boresight_yaw_offset)
            rospy.logwarn(
                "[vision_triggered_navigator] 厂牌框已超时，暂以相机光轴锁定墙段.")
        dx = math.cos(yaw)
        dy = math.sin(yaw)

        rospy.loginfo("[vision_triggered_navigator] 射线起点 (%.4f, %.4f), 方向 yaw=%.4f",
                      px, py, yaw)

        selected = select_corner_aware_wall(
            (px, py), (dx, dy), self.walls,
            self.parking_corner_tie_distance)
        if selected is None:
            rospy.logerr("[vision_triggered_navigator] 射线与围墙无交点，无法计算视觉目标.")
            return None

        raw_point = selected["point"]
        endpoint_margin = max(
            self.parking_wall_endpoint_margin,
            0.5 * self.parking_box_width + self.parking_required_margin,
        )
        ix, iy, clamped = clamp_point_to_wall_segment(
            raw_point, selected["start"], selected["end"], endpoint_margin)
        best_point = (ix, iy)
        nx, ny = selected["normal"]
        best_wall_name = selected["name"]
        gx, gy, gyaw = parking_goal_from_wall(
            best_point,
            selected["normal"],
            self.parking_goal_offset,
            self.parking_normal_offset,
            self.parking_tangent_offset,
        )

        rospy.loginfo(
            "[vision_triggered_navigator] 墙段=%s 原交点=(%.4f,%.4f) "
            "安全交点=(%.4f,%.4f) facing=%.3f corner_clamped=%s "
            "内法向=(%.4f,%.4f) normal_offset=%+.3f tangent_offset=%+.3f "
            "目标点=(%.4f,%.4f,yaw=%.4f)",
            best_wall_name, raw_point[0], raw_point[1], ix, iy,
            selected["facing"], clamped, nx, ny,
            self.parking_normal_offset, self.parking_tangent_offset,
            gx, gy, gyaw)
        self.parking_wall_point = (ix, iy)
        self.parking_inward_normal = (nx, ny)
        self.parking_wall_name = best_wall_name
        self._publish_status("parking_wall_{}".format(best_wall_name))
        return gx, gy, gyaw

    def compute_staging_goals(self):
        """Build globally plannable staging alternatives for one locked wall."""
        if self.parking_wall_point is None or self.parking_inward_normal is None:
            return []
        candidates = []
        seen = set()
        for normal_offset in self.parking_staging_normal_offsets:
            for tangent_delta in self.parking_staging_tangent_offsets:
                goal = parking_goal_from_wall(
                    self.parking_wall_point,
                    self.parking_inward_normal,
                    normal_offset,
                    self.parking_normal_offset,
                    self.parking_tangent_offset + tangent_delta,
                )
                key = (round(goal[0], 3), round(goal[1], 3), round(goal[2], 3))
                if key in seen or not self._coverage_point_inside_room(goal[0], goal[1]):
                    continue
                seen.add(key)
                known, max_cost, _blocked = self._coverage_pose_cost(goal[0], goal[1])
                if known and max_cost >= self.lethal_cost:
                    continue
                plan_length = self._coverage_plan_length(*goal)
                if plan_length is None:
                    continue
                candidates.append((goal, normal_offset, tangent_delta, plan_length))
                if len(candidates) >= self.parking_staging_max_attempts:
                    break
            if len(candidates) >= self.parking_staging_max_attempts:
                break
        if not candidates:
            rospy.logerr(
                "[vision_triggered_navigator] selected wall has no plannable "
                "staging candidate; refusing to send a known-unreachable goal.")
            return []
        for index, (goal, normal_offset, tangent_delta, plan_length) in enumerate(
                candidates, 1):
            rospy.loginfo(
                "[vision_triggered_navigator] staging candidate %d "
                "pose=(%.3f,%.3f,%.3f) normal=%.2f tangent_delta=%+.2f plan=%.2fm",
                index, goal[0], goal[1], goal[2], normal_offset,
                tangent_delta, plan_length)
        return [item[0] for item in candidates]

    def compute_staging_goal(self):
        """Compatibility wrapper returning the first plannable staging pose."""
        goals = self.compute_staging_goals()
        return goals[0] if goals else None

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def run(self):
        if not self.navigation_start_event.is_set():
            self._publish_status("prewarmed_waiting_start")
            rospy.loginfo(
                "[vision_triggered_navigator] 初始化完成，等待总控放行导航。"
            )
        while (not rospy.is_shutdown() and
               not self.navigation_start_event.wait(0.1)):
            pass
        if rospy.is_shutdown():
            return
        rospy.loginfo("[vision_triggered_navigator] 节点启动，开始三阶段导航.")
        if (self.coverage_search_mode and
                not self._disable_move_base_recovery_for_coverage()):
            self.cmd_vel_pub.publish(Twist())
            self._publish_status("coverage_recovery_disable_failed")
            return
        self._publish_status("patrolling")

        # 步骤 0：给 AMCL 发送初始位姿
        self.publish_initial_pose_to_amcl()

        state = "PATROL"
        patrol_idx = 0
        coverage_order = coverage_anchor_order(
            len(self.patrol_points),
            self.preferred_coverage_anchor,
            self.max_coverage_anchors,
        )
        if (self.preferred_coverage_anchor > 0 and coverage_order and
                coverage_order[0] == self.preferred_coverage_anchor - 1):
            rospy.loginfo(
                "[vision_triggered_navigator] preferred anchor %d is first; "
                "remaining anchors continue in cyclic scan order.",
                self.preferred_coverage_anchor)
        coverage_count = len(coverage_order)
        coverage_position = 0

        while not rospy.is_shutdown():
            # 一旦被触发，立即切换到视觉阶段
            if self.triggered and state == "PATROL":
                rospy.loginfo("[vision_triggered_navigator] 巡航被打断，进入视觉触发阶段.")
                state = "VISION"
                continue

            if state == "PATROL":
                if self.coverage_search_mode:
                    if coverage_position >= coverage_count:
                        rospy.logerr(
                            "[vision_triggered_navigator] %d个精确观察点已按原顺序处理完成，但未锁定目标.",
                            coverage_count)
                        self._publish_status("failed")
                        break

                    point_idx = coverage_order[coverage_position]
                    point = self.patrol_points[point_idx]
                    rospy.loginfo(
                        "[vision_triggered_navigator] === 覆盖锚点 %d / %d，逻辑编号%d ===",
                        coverage_position + 1, coverage_count, point_idx + 1)
                    outcome = self._visit_coverage_point(point, point_idx)
                    if outcome == "triggered":
                        state = "VISION"
                        continue
                    coverage_position += 1
                    continue

                if patrol_idx >= len(self.patrol_points):
                    if not self.navigate_to_end_after_trigger:
                        rospy.logerr("[vision_triggered_navigator] 巡航点全部完成但未识别到目标厂牌.")
                        self._publish_status("failed")
                        break
                    rospy.loginfo("[vision_triggered_navigator] 巡航点全部完成，进入结束点阶段.")
                    state = "END"
                    continue

                point = self.patrol_points[patrol_idx]
                x = point["x"]
                y = point["y"]
                yaw = point["yaw"]
                rotations = point.get("rotations", [])

                rospy.loginfo("[vision_triggered_navigator] === 巡航点 %d / %d ===",
                              patrol_idx + 1, len(self.patrol_points))

                # 发送前检查可行性
                if not self.is_goal_feasible(x, y):
                    rospy.logwarn("[vision_triggered_navigator] 巡航点 %d 初始不可行，跳过.", patrol_idx + 1)
                    patrol_idx += 1
                    continue

                result = self.send_goal(x, y, yaw)

                # 若触发被打断
                if self.triggered:
                    rospy.loginfo("[vision_triggered_navigator] 巡航点 %d 导航中被触发，进入视觉阶段.",
                                  patrol_idx + 1)
                    state = "VISION"
                    continue

                # 若因中途代价变高被取消
                if self.current_goal_infeasible:
                    rospy.logwarn("[vision_triggered_navigator] 巡航点 %d 中途不可行，跳到下一目标.",
                                  patrol_idx + 1)
                    patrol_idx += 1
                    continue

                # 成功到达则执行自转
                if result == actionlib.GoalStatus.SUCCEEDED:
                    self.perform_rotations(rotations)
                    patrol_idx += 1
                else:
                    rospy.logwarn("[vision_triggered_navigator] 巡航点 %d 导航未成功，跳过.", patrol_idx + 1)
                    patrol_idx += 1

            elif state == "VISION":
                rospy.loginfo("[vision_triggered_navigator] === 视觉触发阶段 ===")
                self.cancel_goal()
                self._hold_stopped(self.coverage_scan_settle)
                self._publish_status("target_locked")
                if self.center_only:
                    if not self._center_visual_target():
                        rospy.logerr(
                            "[vision_triggered_navigator] center_only模式目标居中失败.")
                        break
                    self._hold_stopped(self.arrival_hold_sec)
                    self._publish_status("centered")
                    rospy.logwarn(
                        "[vision_triggered_navigator] center_only=true：仅完成居中，不执行50cm框停泊.")
                    break
                self._publish_status("target_geometry_locking")
                goal = self.compute_vision_goal()
                if goal is not None:
                    gx, gy, gyaw = goal
                    self._publish_status("target_geometry_locked")
                else:
                    rospy.logerr("[vision_triggered_navigator] 视觉目标计算失败.")
                    self._publish_status("failed")
                    break
                staging_goals = self.compute_staging_goals()
                if not staging_goals:
                    self._publish_status("parking_staging_failed")
                    self._hold_stopped(self.arrival_hold_sec)
                    break
                staging_reached = False
                for staging_attempt, staging_goal in enumerate(staging_goals, 1):
                    self._publish_status(
                        "parking_staging" if staging_attempt == 1 else
                        "parking_staging_retry_{}".format(staging_attempt))
                    if self._navigate_to_parking_staging(staging_goal):
                        staging_reached = True
                        break
                    rospy.logwarn(
                        "[vision_triggered_navigator] staging candidate %d/%d failed; "
                        "trying the next plannable pose without pausing the mission.",
                        staging_attempt, len(staging_goals))
                if not staging_reached:
                    self._publish_status("parking_staging_failed")
                    self._hold_stopped(self.arrival_hold_sec)
                    break
                self._publish_status("parking_recenter")
                if self._wait_for_initial_recenter_target():
                    recentered = self._center_parking_target_continuous()
                    if recentered:
                        # A completed close-range recenter may refine the tangent.
                        refined_goal = self.compute_vision_goal()
                        if refined_goal is not None:
                            gx, gy, gyaw = refined_goal
                        else:
                            rospy.logwarn(
                                "[vision_triggered_navigator] 近墙目标重算失败；"
                                "保留首次锁定的墙段继续停泊.")
                    else:
                        self._publish_status("parking_recenter_skipped")
                        rospy.logwarn(
                            "[vision_triggered_navigator] 近墙OCR精调未完成；"
                            "保留已锁定墙段，转入雷达停泊，不中止任务.")
                else:
                    self._publish_status("parking_recenter_skipped")
                    rospy.logwarn(
                        "[vision_triggered_navigator] 预停后%.1fs内无新OCR目标框；保留首次锁定的墙段/切向目标，继续实墙停泊.",
                        self.parking_recenter_initial_wait)
                self._publish_status("parking_wall_aligning")
                if not self._run_parking_docking((gx, gy, gyaw)):
                    self._publish_status(self.parking_failure_status)
                    self._hold_stopped(self.arrival_hold_sec)
                    break
                self._hold_stopped(self.arrival_hold_sec)
                self._publish_status("parking_verifying")
                parking_valid = self._validate_parking_pose()
                if not parking_valid:
                    self._publish_status("parking_validation_failed")
                    self._hold_stopped(self.arrival_hold_sec)
                    rospy.logerr(
                        "[vision_triggered_navigator] 低速闭环已结束，但完整footprint未达到50cm框2cm余量要求.")
                    break
                if self.navigate_to_end_after_trigger:
                    state = "END"
                else:
                    self._hold_stopped(self.arrival_hold_sec)
                    self._publish_status("arrived")
                    rospy.loginfo("[vision_triggered_navigator] 已抵达厂牌，按配置不前往结束点.")
                    break

            elif state == "END":
                rospy.loginfo("[vision_triggered_navigator] === 结束点阶段 ===")
                x = self.end_goal["x"]
                y = self.end_goal["y"]
                yaw = self.end_goal["yaw"]
                result = self.send_goal(x, y, yaw)
                if result == actionlib.GoalStatus.SUCCEEDED:
                    self._publish_status("completed")
                    rospy.loginfo("[vision_triggered_navigator] 全部流程结束.")
                else:
                    self._publish_status("failed")
                break

            else:
                break

        self.cmd_vel_pub.publish(Twist())
        self._restore_move_base_recovery()


def main():
    node = VisionTriggeredNavigator()
    node.run()


if __name__ == "__main__":
    main()
