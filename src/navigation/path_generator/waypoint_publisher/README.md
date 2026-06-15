# waypoint_publisher Package

Implements task-specific waypoint publishing for NJORD 2026 competition using Nav2's NavigateThroughPoses action.

## Implementation Status

### COMPLETED ✓
- [x] Package structure (ament_python)
- [x] Config files for task1, task2, task3
- [x] waypoint_publisher_node.py (Nav2 NavigateThroughPoses action client)
- [x] Launch file (waypoint_publisher.launch.py)
- [x] task1_sim refactoring: removed pub_goal_pose

### IN PROGRESS / TODO
- [ ] task2_sim: remove pub_goal_pose (lines 39, 62)
- [ ] task3_sim: remove pub_goal_pose (lines 41, 64)
- [ ] Runtime testing with Nav2
- [ ] Integrate with robot launch files

---

## File Structure

```
waypoint_publisher/
├── README.md (this file)
├── package.xml (Python, depends on rclpy, nav2_msgs, geometry_msgs)
├── CMakeLists.txt (ament_python, symlink install for configs & launch)
├── setup.py (entry_points: waypoint_publisher_node)
├── setup.cfg
├── resource/waypoint_publisher
├── waypoint_publisher/
│   ├── __init__.py
│   └── waypoint_publisher_node.py (main implementation)
├── config/
│   ├── task1_waypoints.yaml (4 waypoints)
│   ├── task2_waypoints.yaml (4 waypoints)
│   └── task3_waypoints.yaml (task3.1 & task3.2 configs)
└── launch/
    └── waypoint_publisher.launch.py
```

---

## waypoint_publisher_node.py Overview

**Functionality:**
- Reads task-specific waypoints from YAML config
- Creates Nav2 NavigateThroughPoses action client
- Publishes waypoint sequence on startup (or on demand)
- For task3: implements multi-stage state machine (approach → dock → wait → undock)

**Parameters:**
- `task_type` (str, default='task1'): One of ['task1', 'task2', 'task3_1', 'task3_2']
- `frame_id` (str, default='map'): TF frame for waypoints
- `publish_rate_hz` (float, default=2.0): Publish frequency

**Action Client:**
- Subscribes to `/navigate_through_poses` action server (Nav2)
- Sends NavigateThroughPoses.Goal with poses list

**Task3 State Machine:**
```
IDLE → APPROACHING (stage1: approach + dock waypoints)
     → DOCKED_WAIT (wait timer: 10s for task3.1, 5s for task3.2)
     → UNDOCKING (stage2: exit waypoint)
     → EXITING
     → COMPLETE
```

---

## Waypoint Configurations

### task1_waypoints.yaml
```
task1_config:
  frame_id: "map"
  waypoints:
    - id: 1, x: 0.0, y: 0.0, type: "start", name: "Start"
    - id: 2, x: 10.0, y: 0.0, type: "intermediate", name: "Buoy Detection"
    - id: 3, x: 20.0, y: 0.0, type: "intermediate", name: "Cardinal Mark"
    - id: 4, x: 25.0, y: 0.0, type: "goal", name: "Goal"
```

### task2_waypoints.yaml
```
task2_config:
  waypoints:
    - id: 5, x: 0.0, y: 0.0, type: "start"
    - id: "gate_1", x: 20.0, y: 0.0, type: "gate"
    - id: "gate_2", x: 40.0, y: 0.0, type: "gate"
    - id: 6, x: 60.0, y: 0.0, type: "goal"
```

### task3_waypoints.yaml
**task3.1 (Normal Docking):**
```
task3_1_config:
  waypoints:
    - id: 7, x: 0.0, y: 0.0, type: "start", name: "Approach (10m away)"
    - id: 8, x: 0.0, y: 0.0, type: "dock", name: "Dock"
    - id: 9, x: 0.0, y: 0.0, type: "goal", name: "Exit"
  constraints:
    wait_time_s: 10
    reversal_requirement: true
```

**task3.2 (Parallel Docking):**
```
task3_2_config:
  waypoints:
    - id: 9, x: 0.0, y: 0.0, type: "start", name: "Approach (10m away)"
    - id: "dock_berth", x: 0.0, y: 0.0, type: "dock", name: "Parallel Dock"
    - id: 10, x: 0.0, y: 0.0, type: "goal", name: "Exit"
  constraints:
    wait_time_s: 5
    exit_movement: "forward"
```

---

## Usage

### Launch
```bash
ros2 launch waypoint_publisher waypoint_publisher.launch.py task_type:=task1
```

### Parameters
```bash
ros2 launch waypoint_publisher waypoint_publisher.launch.py \
  task_type:=task3_1 \
  frame_id:=map \
  publish_rate_hz:=2.0
```

---

## Integration Status

### task1_sim (src/sim/task1_sim/)
**Status: ✓ DONE**
- Removed `pub_goal_pose` publisher (line 100)
- Removed `pub_goal_pose.publish()` call in `publish_start_and_goal()` (line 177)
- Retains:
  - `/sim/start` (Bool)
  - `/sim/goal_reached` (Bool)
  - `/sim/cardinal_mark` (String)
  - `/virtual_obstacles` (OccupancyGrid / `/virtual_costmap`)
  - `/sim/task1_status` (String)
  - `/sim/boundary_markers` (MarkerArray)

### task2_sim (src/sim/task2_sim/)
**Status: ✗ TODO**
- Lines to remove:
  - Line 39: `self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", transient_qos)`
  - Line 62: `self.pub_goal_pose.publish(goal)` in `publish_goal()`
- File: `src/sim/task2_sim/task2_sim/orchestrator.py`

### task3_sim (src/sim/task3_sim/)
**Status: ✗ TODO**
- Lines to remove:
  - Line 41: `self.pub_goal_pose = self.create_publisher(PoseStamped, "/goal_pose", transient_qos)`
  - Line 64: `self.pub_goal_pose.publish(goal)` in `publish_goal()`
- File: `src/sim/task3_sim/task3_sim/orchestrator.py`

---

## Build Status

```
✓ waypoint_publisher: Clean build successful
✓ task1_sim: Clean build successful (after refactoring)
✓ YAML configs: All parse successfully
✓ Python syntax: All files validated
```

---

## Next Steps

1. **Remove goal_pose from task2_sim and task3_sim**
   - Edit `src/sim/task2_sim/task2_sim/orchestrator.py` (lines 39, 62)
   - Edit `src/sim/task3_sim/task3_sim/orchestrator.py` (lines 41, 64)
   - Rebuild both packages

2. **Update task launch files**
   - Ensure task1_all.launch.py, task2_all.launch.py, task3_all.launch.py include waypoint_publisher node
   - Example: `waypoint_publisher_node_wrapper = Node(package='waypoint_publisher', executable='waypoint_publisher_node', parameters=[{'task_type': 'task1'}])`

3. **Integration testing**
   - Verify Nav2's /navigate_through_poses action server is running
   - Test waypoint publishing with `ros2 topic echo /goal_pose` (if using simple topic publishing) or check action feedback
   - Monitor navigation stack to verify cmd_vel generation

4. **task3 special handling**
   - Verify state machine transitions (APPROACHING → DOCKED_WAIT → UNDOCKING)
   - Test timer-based wait period (10s vs 5s)
   - Confirm exit waypoint is published after wait completes

5. **Coordinate system validation**
   - GPS coordinates in task*_waypoints.yaml are placeholders (0.0, 0.0)
   - Replace with actual competition course coordinates when available
   - Verify frame_id consistency (map vs utm vs gps)

---

## Known Issues / Design Notes

1. **Nav2 Action vs Topic Publishing**
   - Current implementation: NavigateThroughPoses action (asynchronous, multiple poses)
   - Alternative: Simple PoseStamped topic (legacy, single goal)
   - Task3's multi-stage flow requires async handling → action-based approach is appropriate

2. **Waypoint Coordinates**
   - All coordinates in YAML configs are placeholders (x:0.0, y:0.0)
   - These must be replaced with actual GPS/map coordinates from competition organizers
   - Consider adding separate "real" vs "test" config variants if needed

3. **task3 Dock Detection**
   - Current waypoint_publisher publishes approach + dock waypoints
   - Actual dock identification (AR-tag, visual detection) should be handled by separate perception stack
   - Waypoint_publisher waits for goal_reached callback, then publishes exit waypoint

4. **Error Handling**
   - No explicit error recovery if NavigateThroughPoses server is unavailable
   - Consider adding retry logic or fallback mechanism for production use

---

## Author Notes (Internal)

- Implemented using Nav2 NavigateThroughPoses action for consistency with ROS2 Nav2 stack
- task3 state machine handles asynchronous multi-stage flow (approach-dock-wait-undock)
- Config-driven design allows easy task switching via YAML and launch parameters
- Removed tight coupling between taskN_sim and navigation goal publishing
- Branch: 4-task1, created on 2026-05-28

---

## Build Commands

```bash
# Clean build
colcon build --packages-select waypoint_publisher

# Clean rebuild after config changes
rm -rf build/waypoint_publisher install/waypoint_publisher
colcon build --packages-select waypoint_publisher

# Build with task1_sim
colcon build --packages-select waypoint_publisher task1_sim
```

---

## Testing Commands

```bash
# Check if node launches without errors
ros2 run waypoint_publisher waypoint_publisher_node --ros-args -p task_type:=task1

# Check config loading
python3 -c "import yaml; print(yaml.safe_load(open('src/navigation/path_generator/waypoint_publisher/config/task1_waypoints.yaml')))"

# Monitor topic (if using topic-based publishing)
ros2 topic echo /goal_pose
```

---

Last updated: 2026-05-28
Implemented by: waypoint_publisher implementation task
