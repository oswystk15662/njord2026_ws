---
name: nav2-debug
description: Debug ROS 2 Nav2 launch, lifecycle, parameter, plugin, TF, costmap, controller_server, planner_server, and behavior tree problems. Use for Nav2 errors and navigation stack startup issues. Avoid hardware launch unless explicitly approved.
---

# nav2-debug

Use this skill for Nav2-related debugging.

## Common checks

- lifecycle node startup
- parameter file path
- plugin names
- controller_plugins
- planner_plugins
- behavior tree path
- global_frame
- robot_base_frame
- odom_frame
- map -> odom TF
- odom -> base_link TF
- costmap topics

## Preferred order

1. Check launch file.
2. Check Nav2 params.
3. Check TF assumptions.
4. Check plugin names.
5. Check lifecycle manager.
6. Run package-level build.
7. Run minimal safe launch only if approved or clearly simulation-only.

## Avoid

- hardware launch
- full workspace test
- changing many Nav2 parameters at once