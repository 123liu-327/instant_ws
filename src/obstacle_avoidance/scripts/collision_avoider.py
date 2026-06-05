#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态避障安全控制器 (健壮版)
===========================
move_base → /cmd_vel_nominal → 本节点 → /cmd_vel → 底盘

算法：楔形间隙法 (Gap Finding)
1. 激光360°扫描 → 标记"阻塞/通畅"
2. 合并相邻阻塞点 → 障碍扇区
3. 扇区之间 → 安全间隙
4. 选最宽 + 最接近目标的间隙 → 走
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarProcessor:
    """激光雷达数据预处理"""

    def __init__(self):
        self.angles = []
        self.ranges = []
        self.num_points = 0

    def update(self, scan):
        if len(scan.ranges) == 0:
            return
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        range_min = scan.range_min
        self.num_points = len(scan.ranges)
        self.angles = []
        self.ranges = []
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r) or r < range_min:
                self.ranges.append(float('inf'))
            else:
                self.ranges.append(r)
            self.angles.append(angle_min + i * angle_inc)


class GapNavigator:
    """楔形间隙法：找安全间隙 + 决策行驶方向"""

    def __init__(self):
        self.robot_radius = 0.20
        self.safe_distance = 0.80
        self.min_gap_width = 0.40
        self.max_linear_speed = 0.6
        self.max_angular_speed = 0.8

    def find_blocked_sectors(self, angles, ranges):
        """找出所有被障碍物阻塞的角度扇区"""
        sectors = []
        in_sector = False
        sector_start = 0.0
        sector_points = []

        for angle, r in zip(angles, ranges):
            blocked = (r < self.safe_distance)

            if blocked and not in_sector:
                in_sector = True
                sector_start = angle
                sector_points = [angle]

            elif blocked and in_sector:
                sector_points.append(angle)

            elif not blocked and in_sector:
                in_sector = False
                sector_end = sector_points[-1]
                if abs(sector_end - sector_start) > math.radians(1.0):
                    sectors.append((sector_start, sector_end))

        if in_sector:
            sector_end = sector_points[-1]
            if abs(sector_end - sector_start) > math.radians(1.0):
                sectors.append((sector_start, sector_end))

        return sectors

    def merge_adjacent_sectors(self, sectors):
        """合并间隙太窄的相邻扇区（机器人钻不过去）"""
        if len(sectors) < 2:
            return sectors
        sectors = sorted(sectors, key=lambda s: s[0])
        merged = [sectors[0]]
        for next_s in sectors[1:]:
            cur = merged[-1]
            gap_arc = (next_s[0] - cur[1]) * self.safe_distance
            if gap_arc < self.min_gap_width:
                merged[-1] = (cur[0], next_s[1])
            else:
                merged.append(next_s)
        return merged

    def find_gaps(self, sectors):
        """从阻塞扇区中提取所有安全间隙"""
        if len(sectors) == 0:
            return [(0.0, 2 * math.pi)]

        sectors = sorted(sectors, key=lambda s: s[0])
        gaps = []

        for i in range(len(sectors)):
            next_i = (i + 1) % len(sectors)
            if next_i == 0:
                gap_start = sectors[i][1]
                gap_end = sectors[next_i][0] + 2 * math.pi
            else:
                gap_start = sectors[i][1]
                gap_end = sectors[next_i][0]

            gap_width = gap_end - gap_start
            gap_arc = gap_width * self.safe_distance
            if gap_arc > self.min_gap_width:
                gap_center = gap_start + gap_width / 2.0
                if gap_center > math.pi:
                    gap_center -= 2 * math.pi
                elif gap_center < -math.pi:
                    gap_center += 2 * math.pi
                gaps.append((gap_center, gap_width))

        return gaps

    def compute_best_direction(self, gaps, desired_angle):
        """评分选择最佳间隙"""
        if len(gaps) == 0:
            return None

        best_gap = gaps[0]
        best_score = -float('inf')

        for gap_center, gap_width in gaps:
            width_score = min(gap_width, 2.0) / 2.0
            angle_diff = abs(self._angle_diff(gap_center, desired_angle))
            direction_score = max(0.0, 1.0 - angle_diff / math.pi)
            score = width_score * 3.0 + direction_score * 2.0

            if score > best_score:
                best_score = score
                best_gap = (gap_center, gap_width)

        return best_gap

    def compute_cmd(self, angles, ranges, nominal):
        """主决策：根据障碍物和目标方向计算速度指令"""
        cmd = Twist()

        blocked = self.find_blocked_sectors(angles, ranges)
        blocked = self.merge_adjacent_sectors(blocked)
        gaps = self.find_gaps(blocked)
        min_dist = min(ranges) if ranges else float('inf')
        desired_angle = math.atan2(nominal.linear.y, nominal.linear.x)

        if len(blocked) == 0 or min_dist > self.safe_distance:
            cmd = nominal
        else:
            best_gap = self.compute_best_direction(gaps, desired_angle)

            if best_gap is None:
                rospy.logwarn_throttle(1.0, "所有方向被堵！原地等待...")
            else:
                gap_center, gap_width = best_gap

                speed_factor = min(1.0, gap_width / 1.5)
                speed_factor *= min(1.0, (min_dist - self.robot_radius) / self.safe_distance)
                target_speed = self.max_linear_speed * speed_factor

                if abs(nominal.linear.x) > 0.01 or abs(nominal.linear.y) > 0.01:
                    cmd.linear.x = target_speed * math.cos(gap_center)
                    cmd.linear.y = target_speed * math.sin(gap_center)
                else:
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0

                cmd.angular.z = gap_center * 2.0
                cmd.angular.z = max(-self.max_angular_speed,
                                    min(self.max_angular_speed, cmd.angular.z))

                if gap_width < math.radians(20):
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0

                rospy.loginfo_throttle(0.5,
                    "避障 | 间隙%.0f° 中心%.0f° | 最近%.2fm | 速度%.2f",
                    math.degrees(gap_width), math.degrees(gap_center),
                    min_dist, target_speed)

        cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.x))
        cmd.linear.y = max(-self.max_linear_speed, min(self.max_linear_speed, cmd.linear.y))
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))
        return cmd

    @staticmethod
    def _angle_diff(a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


class CollisionAvoider:
    def __init__(self):
        rospy.init_node('collision_avoider')
        self.lidar = LidarProcessor()
        self.navigator = GapNavigator()
        self.latest_scan = None
        self.nominal_cmd = Twist()
        self.last_nominal_time = rospy.Time.now()

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cmd_vel_nominal', Twist, self.nominal_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.loginfo("Collision Avoider v2 已启动 (楔形间隙法)")

    def scan_callback(self, scan):
        self.lidar.update(scan)
        self.latest_scan = scan

    def nominal_callback(self, cmd):
        self.nominal_cmd = cmd
        self.last_nominal_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.latest_scan is None:
                rate.sleep()
                continue

            elapsed = (rospy.Time.now() - self.last_nominal_time).to_sec()
            nominal = self.nominal_cmd if elapsed < 0.5 else Twist()

            cmd = self.navigator.compute_cmd(
                self.lidar.angles, self.lidar.ranges, nominal)
            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        CollisionAvoider().run()
    except rospy.ROSInterruptException:
        pass
