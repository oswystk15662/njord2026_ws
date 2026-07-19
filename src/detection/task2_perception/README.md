# task2_perception

Task 2 (collision avoidance) LiDAR perception glue for the Njord ASV.
Bridges the raw Livox Mid-360 cloud into the reused `pcl_segmentation`
submodule pipeline, and bridges the submodule tracker output into the MPPI
planner. Also detects the quay wall and feeds it to Nav2.

## Full chain

```
/livox/lidar (PointCloud2, livox_frame, ~10 Hz, xyz+intensity)
    |
    v
task2_cloud_filter                              [this package]
    TF livox_frame -> base_link (URDF owns the upside-down correction)
    nonfinite -> range (0.5-60 m) -> self crop-box -> object height band
    -> water removal (z band + guarded RANSAC horizontal plane)
    |
    v
/task2/points_filtered (base_link)
    |
    v
pcl_segmentation submodule (reused AS-IS, zero modification):
    ros2 launch ship_perception_bringup classical_pipeline.launch.py \
        lidar_topic:=/task2/points_filtered \
        ego_odom_topic:=/odometry/filtered/local
    preprocessing_node (voxel 0.1 m, ROI, accumulate 3 frames)
        -> /pcl/preprocessed
    ground_remover_node -> /pcl/nonground ----------------+
    cluster_node -> /pcl/cluster_centroids (PoseArray)    |
    ship_tracker_node (EKF, Hungarian association)        |
        -> /tracked_objects                               |
           (ship_perception_msgs/TrackedObjectArray,      |
            pose base_link-relative, twist RELATIVE and   |
            expressed in the object body frame — the      |
            tracker's ego compensation is a no-op stub)   |
        -> TF base_link -> tracked_obj_{id}               |
    |                                                     v
    v                                          quay_wall_detector [this pkg]
opponent_selector                              [this package]  vertical-cell
    gate (confirmed / distance / size /        candidates -> 2D RANSAC lines
    points / staleness) -> rank ("nearest"     -> temporal confirmation
    | "min_tcpa" | "track_id") -> top-1        -> /quay_wall/points (base_link,
    body twist -> base axes -> + ego twist        Nav2 obstacle-layer source)
    -> map frame -> low-pass + spike gate      -> /quay_wall/markers
    |                                          -> /quay_wall/costmap (map,
    v                                             inflated quay_safety_margin_m)
/other_ship/twist (TwistStamped, map frame, ABSOLUTE ground velocity)
+ TF map -> opponent_vessel
    |
    v
MPPI planner (asv_trajectory_planner/planner_node.py,
    other_twist_is_relative=False, require_other_ship=False:
    no opponent published -> plans a straight path — safe degradation)
```

## Nodes

| Node (executable) | In | Out |
|---|---|---|
| `task2_cloud_filter` (`task2_cloud_filter_node`) | `/livox/lidar`, TF | `/task2/points_filtered`, `/task2/debug/*` (opt.) |
| `opponent_selector` (`opponent_selector_node`) | `/tracked_objects`, `/odometry/filtered/local`, TF map->base_link | `/other_ship/twist`, TF map->opponent_vessel |
| `quay_wall_detector` (`quay_wall_detector_node`) | `/pcl/nonground` | `/quay_wall/points`, `/quay_wall/markers`, `/quay_wall/costmap` |

Launch: `ros2 launch task2_perception task2_perception.launch.py`
(args `enable_cloud_filter`, `enable_opponent_selector`,
`enable_quay_detection`, all default `true`). The submodule
`classical_pipeline.launch.py` must be launched separately with the arguments
shown above.

## Design notes

- **Inverted LiDAR**: the Mid-360 is mounted upside-down. The nominal
  roll = pi correction lives ONLY in the URDF `lidar_joint`
  (`src/robot/urdf/robot.urdf.xacro`); all nodes consume TF. The
  `lidar_inverted` / `lidar_*_deg` parameters of the cloud filter are an
  emergency manual pre-rotation for URDF-less setups and warn loudly when
  active. **HUMAN: measure the real mount angles before trusting
  perception/GLIM.**
- **Water removal** cannot eat walls or decks: the RANSAC plane is accepted
  only when its normal is near-vertical (`water_plane_normal_z_min`) AND its
  height is within `water_plane_max_height_error_m` of `waterline_z_m`.
- **Velocities**: the tracker EKF already estimates velocity; nothing here
  numerically differentiates positions. The selector rotates the body-frame
  twist into base_link axes, adds the ego twist (`/odometry/filtered/local`,
  child frame base_link), rotates into map, then applies the same low-pass +
  spike rejection style as `opponent_twist_from_tf_node.py`.
- **Safe degradation**: no valid confirmed track -> `/other_ship/twist` and
  the opponent TF simply stop (MPPI plans straight); the quay topics publish
  nothing a costmap would mark, so the sim without walls is unaffected.
- **Nav2**: `nav2_params_task2.yaml` adds a second obstacle-layer observation
  source `quay` on `/quay_wall/points` (marking, non-clearing) in both
  costmaps.

## Pure modules / tests

`cloud_ops.py`, `wall_fit.py`, `tracking_glue.py`, `smoothing.py` are
ROS-free (numpy only) and covered by pytest:

```
python3 -m pytest src/detection/task2_perception/test/ -q
```

## Parameters that MUST be measured on the vessel

See `config/task2_perception_params.yaml`, marked `HUMAN: measure on vessel`:
`waterline_z_m`, the `self_crop_*` box, `wall_min_points`, and the URDF
LiDAR mount angles/position.
