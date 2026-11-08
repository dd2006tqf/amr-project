# Chuyun lightweight navigation

This package contains a dependency-light ROS 2 navigation stack for a static
2-D occupancy map. It needs fused odometry, a `sensor_msgs/LaserScan`, the
robot TF tree, and no Nav2 runtime components.

## Modules

- `GridMap`: PGM/YAML loading, obstacle distance field, and inflated map.
- `CorrelativeScanMatcher`: odometry-seeded, coarse-to-fine likelihood-field
  localization with score/coverage rejection.
- `AStarPlanner`: clearance-aware global planning and line-of-sight path
  simplification, including temporary obstacles projected from the live scan.
- `LocalTrajectoryPlanner`: short-horizon velocity sampling with collision,
  clearance, progress, and command-continuity scoring. It remembers the chosen
  passing side to suppress left/right hesitation.
- `PathFollower`: look-ahead tracking, acceleration limits, scan freshness
  failsafe, and local-planner integration.
- `LiteNavigationNode`: ROS parameters, topics, TF, timestamp alignment, and
  orchestration only.

The scan matcher transforms laser endpoints into `base_frame` using TF before
matching. It also interpolates odometry to the scan timestamp, which avoids the
systematic error caused by matching a delayed scan against the newest odometry.

## Build only this package

```bash
cd /root/chuyun_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chuyun_navigation2 --symlink-install
source install/setup.bash
```

## Run

Start the robot driver and fused odometry first so that `/scan` and
`/odometry/filtered` exist. By default the launch file publishes Chuyun's
fixed `base_footprint -> laser_frame` transform (z = 0.151 m, derived from the
current URDF), so the lightweight stack can run without `robot_state_publisher`.

```bash
ros2 launch chuyun_navigation2 lite_navigation.launch.py \
  map:=/root/chuyun_ws/src/chuyun_navigation2/maps/room.yaml
```

If a complete `robot_state_publisher` is already running, avoid duplicate TF
publishers by adding `publish_laser_tf:=false`. The `base_frame` and
`laser_frame` launch arguments can be changed for another robot.

Set the initial pose with RViz (`/initialpose`) and send a Nav2 goal
(`/goal_pose`). The public compatibility topics remain `/map`, `/plan`,
`/cmd_vel`, and `/amcl_pose`. Localization quality is published as a value from
0 to 1 on `/lite_localization/score`; navigation state is published on
`/lite_navigation/status`.

The main localization tuning controls are in `config/lite_navigation.yaml`:

- Increase `localization_search_xy` / `localization_search_yaw` if odometry can
  drift farther between matches (with a proportional CPU cost).
- Lower `localization_min_score` slightly only when valid scans are being
  rejected; a low threshold can accept a false wall alignment.
- `localization_correction_gain` controls smoothing. Higher values correct
  faster but make `map -> odom` less smooth.

Dynamic obstacles are handled in three layers. The local trajectory planner
updates from every scan and normally steers around a moving object. If the
robot is already inside the configured safety clearance, a recovery sampler
selects a direction whose clearance increases monotonically instead of
rejecting every command and remaining stopped. Sustained avoidance projects
current scan endpoints into a wide temporary obstacle/clearance layer and A*
publishes a replacement `/plan`; the clearance cost intentionally favors a
wide, stable detour over the shortest path. The relevant states are
`LOCAL_AVOIDING`, `LOCAL_RECOVERY`, `LOCAL_BLOCKED`, and `DYNAMIC_REPLAN`.
