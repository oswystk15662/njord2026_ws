# task1_sim

NJORD 2026 Task 1 maneuvering/path-finding simulation harness.

Task 1 requires the ASV to follow four GPS checkpoints, pass intermediate GPS
waypoints, detect cardinal markers, pass the buoy on the side indicated by the
marker, avoid buoy contact, and compare the actual route with the ideal route.
This package models that flow in a simulation-local `map` frame.

## Responsibilities

- launch the Task1 sim stack: dynamics, Nav2, waypoint publisher, validator, and orchestrator
- provide a YOLO/cardinal-marker stub through `/yolo/start_inference`
- publish dynamic virtual obstacles on `/virtual_obstacles`
- publish route and buoy status on `/sim/task1_status`
- publish boundary/buoy RViz markers on `/sim/boundary_markers`
- publish `/sim/goal_reached` only after waypoint, avoidance, and final goal constraints pass

Waypoint goals are sent by `waypoint_publisher` through Nav2's
`/navigate_through_poses` action. `task1_sim` does not publish `/goal_pose`.

## Usage

```bash
ros2 launch task1_sim task1_sim.launch.py
```

Useful launch switches:

- `use_dynamics:=false`
- `use_nav2:=false`
- `use_waypoints:=false`
- `use_validator:=false`

The default route and simulated obstacle geometry are configured in
`config/task1_params.yaml`.
