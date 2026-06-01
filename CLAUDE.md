# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a **ROS Noetic** catkin workspace. The workspace is at `/home/iden/car_ws`.

```bash
# Build the workspace
catkin_make

# Build a single package
catkin_make --pkg <package_name>
```

When adding new C++ nodes or libraries, update the package's `CMakeLists.txt` and `package.xml`. The top-level `src/CMakeLists.txt` is a symlink to catkin's toplevel.cmake and should not be edited.

## Runtime

Source the workspace before running anything:

```bash
source /home/iden/car_ws/devel/setup.bash
```

### Main launch sequence

```bash
# Full system launch (navigation stack + base driver + LiDAR + AMCL + move_base)
roslaunch ucar_controller all_process.launch

# Cruise mission (waypoint patrol using move_base)
rosrun ucar_controller process_5.5.1.py

# Visual line-following (the follow_test variant is the currently active one)
roslaunch flow_end follow_test.launch

# Trigger follow_test to start with a path choice
rostopic pub /follow_begin std_msgs/String "data: 'Left'"
```

### Saving maps

After SLAM mapping, save the map with:

```bash
rosrun map_server map_saver -f <map_name>
```

Map files live in `src/ucar_nav/maps/`. The active map is configured in `ucar_navigation_test.launch`.

## Architecture

The system is an autonomous ground vehicle (differential drive) with three main operational modes:

### 1. Visual line-following (primary mode)

**Package:** `flow_end` (C++, OpenCV) — the most actively developed package.

- **`follow_test`** (`src/follow_test.cpp`) — The current active line-following node. Reads camera images via ROS, extracts line features using adaptive thresholding, and publishes `/cmd_vel` to steer. Supports Left/Middle/Right path selection via the `/follow_begin` topic. Includes:
  - Initial turn phase: uses IMU angular velocity integration (`curent_wz * dt * 57.3`) to pre-rotate toward the chosen branch before line tracking
  - L-shaped corner parking detection
  - `process_image.cpp` — Core image processing, perspective transform via lookup table (`point_map`), line extraction with `Findline_Adaptive`
  - `follow_line.cpp` — PID-based line following control, also contains legacy roundabout/corner states
  - `PID.cpp` — PID controller
  - `Signal.cpp` — Signal handling (SIGINT)
  - `Callback_test.cpp` — ROS topic subscriptions/advertisements for follow_test
- **`follow_end`** — Legacy full-featured version with roundabout and LiDAR-based flow control. Not the primary node anymore.
- Key launch files: `follow_test.launch` (active), `follow_end.launch` (compatibility wrapper that delegates to follow_test.launch)
- Configuration: all via ROS param (launch file), no separate YAML config files
- Video recording: can save debug output as AVI files instead of publishing ROS topics

### 2. Waypoint navigation

**Package:** `ucar_controller` (Python)

- `scripts/process.py` and `scripts/process_5.5.1.py` — Cruise mission scripts that send sequential waypoints to `move_base` via the `MoveBaseAction` actionlib interface. Waypoints are hardcoded as `(x, y, z, qx, qy, qz, qw)` lists in a `nav_point` dictionary.
- `scripts/process_qr.py` — Variant with QR-code triggered behaviors.
- `sensor_tf_server.py` — Publishes static transforms for sensor frames.

**Package:** `my_planner` (C++) — Custom local planner plugin for `move_base`, replaces the standard DWA/TEB planner. Registered as `my_planner/MyPlanner` via pluginlib.

### 3. Navigation stack

**Package:** `ucar_nav`

- `launch/ucar_navigation_test.launch` — The main navigation launch file. Starts:
  - `base_driver` (from `ucar_controller`) — Serial communication with the motor controller
  - `ydlidar` — LiDAR driver
  - `map_server` — Serves static map (currently `5.4.1.yaml`)
  - `amcl_omni` — AMCL localization (omni-directional model)
  - `move_base` — Global planner (`global_planner/GlobalPlanner`) + custom local planner (`my_planner/MyPlanner`)
- `launch/config/move_base/costmap_common_params.yaml` — Costmap configuration (footprint, obstacle layer, inflation)
- `scripts/ucar_followline.py` — Pure pursuit-based line following

### 4. Hardware drivers

- **`iden_controller`** (C++) — Custom base driver (`my_base_driver.cpp`, `base_driver.cpp`) with serial communication, CRC handling (`crc_table.cpp`), and TF services. Config files in `config/` for different vehicle variants (mini, ucarV2, xiao).
- **`ydlidar`** — YDLIDAR LiDAR driver
- **`fdilink_ahrs`** — IMU/AHRS driver (publishes `/imu`)
- **`ucar_camera`** — USB camera driver (publishes `/ucar_camera/image_raw`)

### 5. Perception and interaction

- **`yolo_ros`** + **`yolov5`** — YOLOv5-based object detection (imported from external repos)
- **`speech_command`** — iFLYTEK voice command recognition with audio files for wakeup/feedback
- **`line_follower`** — Standalone line-following with dynamic reconfigure (PID + image params)
- **`test`** — Miscellaneous test nodes (QR code detection, LiDAR angle test, IMU test)
- **`dec_msgs`** — Custom ROS messages
- **`waterplus_map_tools`** — Map manipulation utilities
- **`playsound`** — Audio playback utility
- **`startup_scripts`** — Device initialization scripts (`initdev_mini.sh`, `initdev_xiao.sh`)

### Key ROS topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | out | Motor velocity commands |
| `/ucar_camera/image_raw` | `sensor_msgs/Image` | in | Camera feed |
| `/imu` | `sensor_msgs/Imu` | in | IMU data (angular_velocity.z used for turn integration) |
| `/scan` | `sensor_msgs/LaserScan` | in | LiDAR scan data |
| `/follow_begin` | `std_msgs/String` | in | Line-following trigger ("Left"/"Middle"/"Right"/"Stop") |
| `/follow_end` | `std_msgs/String` | out | Line-following completion signal |
| `/odom` | `nav_msgs/Odometry` | in | Wheel odometry |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | in/out | Initial pose for AMCL |

## Code conventions

- C++ standard: C++17 for `flow_end`, C++11 for `iden_controller`
- Python scripts use Python 3 with `#!/usr/bin/env python3`
- Chinese comments and log messages are common throughout the codebase
- The workspace path appears in config files as both `/home/iden/car_ws` and `/home/ucar/ucar_ws` (the robot's onboard path)
- `flow_end` has its own internal `.git` repo (independent version control within the catkin workspace)

# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.