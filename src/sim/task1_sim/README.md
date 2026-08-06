# task1_sim

NJORD 2026 Task 1 maneuvering/path-finding simulation harness.

Task 1 requires the ASV to follow GPS checkpoints, detect cardinal markers,
pass each marker on the side its cardinal orientation indicates, avoid buoy
contact, and compare the actual route with the ideal route. This package
models that flow in a simulation-local `map` frame.

## Course geometry (default: `task_type:=task1`)

The default route is a Cartesian sim approximation of the full NJORD 2026
Task1 course: the task1-1 maneuvering section followed by the cardinal-marker
slalom:

```
(0,0) -> (10,5) -> ... -> (40,0) -> (50,0) -> (50,-25) ->
    (24,-35) -> (15,-15) -> (8,-35) -> (0,-25)
```

Three cardinal marks sit at `y=-25` along that route and force an
alternating-side passage:

| marker (x, y)   | orientation | boat passes |
|-----------------|-------------|-------------|
| (28.0, -25.0)   | S           | south       |
| (18.0, -25.0)   | N           | north       |
| (11.0, -25.0)   | S           | south       |

Marker positions and orientations are configured in
`config/task1_params.yaml` (`buoy_position_xy` / `buoy_marks`, index-aligned)
and mirrored in `waypoint_publisher`'s
`task1_waypoints.yaml`'s `task1_config` so the orchestrator's own
goal-tracking and the actual Nav2 route agree. `forced_mark` in
`task1_params.yaml` is a testing override that forces every marker to one
mark, ignoring `buoy_marks`.

A second scenario, `task_type:=task1_follow`, runs the original two-period
sine-wave follow-test route (`task1_follow_config` in
`task1_waypoints.yaml`) with no cardinal-marker detection. It exists for
regression-testing plain waypoint following, not the competition scenario.

## Cardinal-marker perception pipeline (sim)

The sim has no camera/LiDAR, so `cardinal_perception_sim` stands in for the
real ZED2i + Mid-360 fusion pipeline (`zed2i_driver`'s `sdk_node_zed`):

1. `task1_orchestrator` publishes `buoy_marks` as a JSON array on
   `/sim/cardinal_mark`, index-aligned with `buoy_position_xy` (e.g.
   `["S", "N", "S"]`), so each marker keeps its own orientation.
2. `cardinal_perception_sim` gates detections by the boat's distance and
   bearing to each marker (ZED2i range/FOV, Mid-360 range/FOV) and, once a
   marker is within range, auto-triggers `/yolo/start_inference`.
3. Once inference is confirmed, `cardinal_perception_sim` publishes
   `njord_interfaces/BuoyDetectionArray` on `/buoy_detections_3d` with each
   detection's `class_id` taken from that marker's own mark.
4. `cardinal_wall_publisher` (package `buoy_obstacle_publisher`) turns
   confirmed detections into a one-sided Nav2 obstacle wall on
   `/virtual_obstacles`, running from the marker to the course's far edge
   (`course_bounds`) on the side the boat must *not* take -- e.g. a north
   (`N`) marker walls off the south side, leaving the north side free.

## Responsibilities

- launch the Task1 sim stack: dynamics, Nav2, waypoint publisher, validator, and orchestrator
- retain a cardinal-marker stub through `/yolo/start_inference` for task validation
- publish fixed simulation geometry on `/sim_obstacles`; fused cardinal detections
  produce `/virtual_obstacles` through `cardinal_wall_publisher`
- publish route and buoy status on `/sim/task1_status`
- publish boundary/buoy RViz markers on `/sim/boundary_markers`
- publish cardinal-direction RViz arrows on `/sim/cardinal_mark_markers`
- publish `/sim/goal_reached` only after waypoint, avoidance, and final goal constraints pass

Waypoint goals are sent by `waypoint_publisher` through Nav2's
`/navigate_through_poses` action and visualized on `/waypoint_markers`.
`task1_sim` does not publish `/goal_pose`.

For the Task1 simulation, the full numbered goal sequence and the 1.0 m Nav2
reach circles are published as `visualization_msgs/MarkerArray` on
`/sim/task1_waypoint_markers`. The repository's `foxglove_setting.json`
layout enables this topic by default.
The connecting waypoint line is off by default; enable it with
`show_waypoint_route_line:=true` when launching `waypoint_publisher` directly.

## Usage

```bash
ros2 launch task1_sim task1_sim.launch.py

# Follow-test route instead of the competition slalom
ros2 launch task1_sim task1_sim.launch.py task_type:=task1_follow

# Fast control/Nav2 loop using SimNode truth odometry only
ros2 launch task1_sim task1_sim_truth.launch.py

# Simulated IMU/GNSS plus local/global EKF and navsat processing
ros2 launch task1_sim task1_sim_sensor_parity.launch.py
```

Useful launch switches:

- `use_dynamics:=false`
- `use_nav2:=false`
- `use_waypoints:=false`
- `use_validator:=false`
- `task_type:=task1` (default, competition slalom) or `task_type:=task1_follow`
- `use_cardinal_perception_sim:=false`

The default route and simulated obstacle geometry are configured in
`config/task1_params.yaml`. The global costmap window
(`config/task1_nav2_params.yaml`, `global_costmap.width`/`height`/`origin_x`/
`origin_y`) covers both `task1` and `task1_follow` routes.
