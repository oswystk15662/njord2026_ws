# task1_sim

NJORD 2026 Task 1 maneuvering/path-finding simulation harness.

Task 1 requires the ASV to follow four GPS checkpoints, pass intermediate GPS
waypoints, detect cardinal markers, pass the buoy on the side indicated by the
marker, avoid buoy contact, and compare the actual route with the ideal route.
This package models that flow in a simulation-local `map` frame.

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

## Usage

```bash
ros2 launch task1_sim task1_sim.launch.py

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

The default route and simulated obstacle geometry are configured in
`config/task1_params.yaml`.
