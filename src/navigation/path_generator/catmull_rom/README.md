# catmull_rom_path_smoother

Standalone Catmull-Rom smoother for `nav_msgs/msg/Path`.

The node subscribes to a configured input path topic and republishes a smoothed
path on `/plan_smoothed` by default. It is intentionally independent from the
Nav2 smoother plugin path, so it can be used for visualization, experiments, or
legacy path-generator flows without replacing the active Nav2 planning stack.

## Topics

Default parameters:

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/plan` | `nav_msgs/msg/Path` |
| Publish | `/plan_smoothed` | `nav_msgs/msg/Path` |

## Parameters

See `config/catmull_rom_params.yaml`.

- `input_topic`: input `nav_msgs/msg/Path` topic
- `output_topic`: output smoothed path topic
- `frame_id`: output frame override. Empty string preserves the input frame.
- `sample_interval`: approximate distance between generated samples in meters
- `publish_transient_local`: publish output with transient local durability
- `preserve_input_header_stamp`: copy the input stamp instead of stamping with now
- `minimum_input_poses`: minimum input path size before smoothing

## Launch

```bash
ros2 launch catmull_rom_path_smoother catmull_rom_path_smoother.launch.py
```
