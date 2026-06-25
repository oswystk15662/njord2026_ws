# CODEX_NOTES.md

## Project notes

- This workspace is used for robotics development.
- Hardware behavior must be treated carefully.
- Prefer simulation or package-level validation before hardware assumptions.
- Do not make broad architecture changes unless requested.

## Common debugging order

1. Check `git status`.
2. Identify active package.
3. Inspect package-level files only.
4. Run package-level build.
5. Run minimal launch/test if safe.
6. Update `HANDOFF.md`.

## Common ROS 2 checks

- package.xml dependencies
- CMakeLists.txt install rules
- launch file paths
- parameter file paths
- node names and namespaces
- TF frame names
- lifecycle node states