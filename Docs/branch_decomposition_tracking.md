# Branch Decomposition Tracking

This document tracks the decomposition of the historical `4-task1` and
`test0506` branches into reviewable changes for `master`.

GitHub tracking issue: [#11](https://github.com/IBO-ASV/njord2026_ws/issues/11)

## Source preservation

| Source branch | Archived tip | Archive tag |
|---|---:|---|
| `4-task1` | `c020411` | `archive/4-task1-before-decomposition-20260615` |
| `test0506` | `6f8b986` | `archive/test0506-before-decomposition-20260615` |

The source branches must not be merged directly. Each accepted change is
reconstructed from the latest `master`, scoped to one topic, and reviewed
independently.

## Integration order

| Order | Topic | Planned branch | Source commits | Status |
|---:|---|---|---|---|
| 1 | CI baseline | `chore/ci-baseline`, `fix/ci-ros-container` | new | Merged in [#12](https://github.com/IBO-ASV/njord2026_ws/pull/12) and [#25](https://github.com/IBO-ASV/njord2026_ws/pull/25) |
| 2 | Dependency and submodule cleanup ([#14](https://github.com/IBO-ASV/njord2026_ws/issues/14)) | `chore/dependency-submodules` | `791ddf0`, `82e42c7`, `6f8b986` | Pending review |
| 3 | GLIM, EKF, and localization ([#13](https://github.com/IBO-ASV/njord2026_ws/issues/13)) | `feature/glim-ekf-localization` | `791ddf0`, `4bde855`, `e31390c`, `c020411` | Merged in [#24](https://github.com/IBO-ASV/njord2026_ws/pull/24) |
| 4 | Manual control and Micon agent ([#15](https://github.com/IBO-ASV/njord2026_ws/issues/15)) | `feature/manual-control-micon-agent` | `791ddf0`, `bf12ca9`, `82e42c7`, `919d5cf`, `8dc48fa` | Pending |
| 5 | X4 omni thruster control ([#16](https://github.com/IBO-ASV/njord2026_ws/issues/16)) | `feature/x4-omni-thruster-control` | `8dc48fa`, `18ab245`, `e4464f1`, `7d8634f`, `c020411` | Pending |
| 6 | Buoy perception pipeline ([#17](https://github.com/IBO-ASV/njord2026_ws/issues/17)) | `feature/buoy-perception-pipeline` | `09bc35e`, `528f308`, `82e42c7` | Merged directly in `26c6076` |
| 7 | Nav2 waypoint and smoothing ([#18](https://github.com/IBO-ASV/njord2026_ws/issues/18)) | `feature/nav2-waypoint-smoothing` | `58c545b`, `9f8d770`, `4bde855`, `f481f46`, `c020411` | Merged directly in `51a543a` |
| 8 | Task1 simulation ([#19](https://github.com/IBO-ASV/njord2026_ws/issues/19)) | `feature/task1-simulation` | `31ba23e`, `28eb2ad`, `82e42c7`, `09bc35e` | Pending |
| 9 | Task3 navigation and simulation ([#20](https://github.com/IBO-ASV/njord2026_ws/issues/20)) | `feature/task3-navigation-simulation` | Task3 commits through `c020411` | Merged directly in `4b25e34` |
| 10 | Visualization and documentation ([#21](https://github.com/IBO-ASV/njord2026_ws/issues/21)) | `docs/architecture-simulation` | `a3d180c`, `18ab245`, `8e6d589`, `c020411` | Pending |
| 11 | `test0506`-only changes ([#22](https://github.com/IBO-ASV/njord2026_ws/issues/22)) | Investigation issues only | `6f8b986` | Hold |

## Explicitly held or excluded

- `.antigravity/rules`
- empty `Docs/about_*.md` files
- empty `src/robot/launch/task2_all.launch.py` and `task3_all.launch.py`
- incomplete `manual_with_odom` package
- empty `cubic_spline` package
- empty `src/ros2.repos`
- `test0506` emergency-stop inversion and manual thrust gain increase
- `test0506` GNSS status semantic change
- unrelated formatting-only changes

## Required checks

Every integration branch must:

1. be based on the latest accepted `master`;
2. contain only its documented topic;
3. pass `git diff --check`;
4. pass changed XML, YAML, and Python syntax checks;
5. build and test affected ROS packages;
6. record unavailable hardware or simulation validation explicitly.

Hardware-dependent changes remain on hold until their hardware behavior is
confirmed.

## Known validation debt

- `simple_sim` was an obsolete lightweight experiment for `/cmd_vel`, `/odom`,
  and path-following checks. It was removed instead of restoring its old lint
  test coverage because the maintained simulation work now lives under
  `src/sim`.
