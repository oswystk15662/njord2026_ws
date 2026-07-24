# Task 2 MPPI other-vessel avoidance

This integration selectively ports the collision-avoidance pipeline from
commit `8cdd9126222d7dea0e53e01ab39e2b735e2e87c0` onto the current `test07089`
hardware stack. YOLO and quay perception are intentionally not part of the
control loop.

## Real-vessel data flow

```text
/livox/lidar
  -> task2_cloud_filter_node
  -> /task2/points_filtered
  -> pcl_segmentation classical_pipeline
  -> /tracked_objects
  -> opponent_selector_node
       -> /other_ship/twist (map-frame absolute ground velocity)
       -> TF map -> opponent_vessel
  -> planner_node (MPPI)
  -> /planned_path_pruned
  -> Nav2 FollowPath
  -> /cmd_vel_nav_raw
  -> velocity_smoother
  -> /cmd_vel
  -> thruster_driver
  -> /thruster_command
  -> serial_writer
```

The real Nav2 costmaps use `/task2/points_filtered`. The simulation keeps
using its ideal `/pointcloud` source.

## Launch

The existing `robot/task2.launch.py` is unchanged. Start the dedicated MPPI
stack with:

```bash
ros2 launch robot task2_real.launch.py
```

Important launch arguments:

- `enable_lidar`, `enable_ship_tracking`, `enable_mppi`, `enable_nav2`
- `enable_camera` (default `false`; camera inference is not needed here)
- `enable_thrusters` (default `true`, matching current `test07089`)
- `enable_debug_topics`

For dockside validation, always override the operational default:

```bash
ros2 launch robot task2_real.launch.py enable_thrusters:=false
```

## Freshness and fallback behavior

`opponent_selector_node` publishes only while a confirmed, fresh track exists.
`planner_node` independently expires received opponent updates after 2 seconds.
With `require_other_ship=false`, it then generates a reconnect/straight path
without an opponent.

This fallback preserves planner availability; it is not a collision-safety
stop. Thrusters must remain disabled until track-loss behavior has been
validated with recorded and live sensor data.

## Required vessel-side validation

1. Confirm the Mid-360 cloud is upright after the URDF roll correction.
2. Measure and tune `waterline_z_m` and the self-crop bounds.
3. Verify `/tracked_objects` velocity is relative and expressed in the tracked
   object's body frame, as assumed by the selector.
4. Confirm `/other_ship/twist` direction and `map -> opponent_vessel` position
   against a stationary and then moving target.
5. Measure MPPI solve time on the Jetson with the configured 225-step horizon
   and 5000 samples.
6. Validate head-on and crossing scenarios with `enable_thrusters:=false`
   before enabling propulsion.

The package declares the standard `python3-torch` rosdep dependency. On Jetson,
install NVIDIA's compatible PyTorch build in the same Python environment used
by ROS 2 and skip that rosdep key if the distribution package would replace
the vendor build.
