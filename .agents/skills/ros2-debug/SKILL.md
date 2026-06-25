---
name: ros2-debug
description: Debug ROS 2 package build, launch, parameter, dependency, CMakeLists.txt, package.xml, node, namespace, and log issues. Use for ROS 2 workspace/package problems. Prefer package-level build and targeted searches.
---

# ros2-debug

Use this skill for ROS 2 package debugging.

## Debug order

1. Identify the package.
2. Check `package.xml`.
3. Check `CMakeLists.txt` or setup files.
4. Check launch file paths.
5. Check parameter file paths.
6. Check node names and namespaces.
7. Check logs.
8. Run package-level build.

## Preferred commands

- `git status --short`
- `rg <symbol> src/<pkg>`
- `colcon build --packages-select <pkg>`
- `colcon test --packages-select <pkg>`

## Avoid

- full workspace build unless needed
- hardware launch
- unrelated refactors
- dependency installs without approval

## Output

Always report:
- likely cause
- relevant files
- smallest fix
- validation command