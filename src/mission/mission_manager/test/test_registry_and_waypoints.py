from pathlib import Path

import pytest

from mission_manager.task_registry import RegistryError, TaskRegistry, required_runtime_readiness
from mission_manager.waypoint_config import WaypointConfigLoader


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WAYPOINT_ROOT = PACKAGE_ROOT.parents[1] / "navigation" / "path_generator" / "waypoint_publisher"


def test_registry_exposes_supported_and_unimplemented_tasks():
    registry = TaskRegistry.from_file(
        PACKAGE_ROOT / "config" / "task_registry.yaml",
        package_shares={"waypoint_publisher": WAYPOINT_ROOT},
    )
    assert registry.get("task1").runnable
    assert registry.get("task2").executor == "task2_mppi"
    assert required_runtime_readiness(registry.get("task2")) == {"buoy_perception"}
    assert registry.get("task1_2").availability == "not_implemented"
    assert "Marker-driven" in registry.get("task1_2").reason


def test_existing_task_routes_load_without_changing_waypoint_files():
    loader = WaypointConfigLoader()
    task1 = loader.load(WAYPOINT_ROOT / "config/task1_waypoints.yaml", "task1_config")
    task2 = loader.load(WAYPOINT_ROOT / "config/task2_waypoints.yaml", "task2_config")
    task3 = loader.load(WAYPOINT_ROOT / "config/task3_waypoints.yaml", "task3_1_config")
    assert len(task1.waypoints) == 17
    assert task1.coordinate_mode == "origin_relative_xy"
    assert task1.origin is not None
    assert task1.waypoints[12].competition_id == "3"
    assert task1.waypoints[-1].competition_id == "4"
    assert [waypoint.waypoint_id for waypoint in task2.waypoints] == ["5", "gate_1", "gate_2", "6"]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_1_gate")] == ["7", "b31_corridor_gate", "8"]


def test_route_supports_origin_relative_and_all_geodetic_coordinates(tmp_path):
    config = tmp_path / "coordinates.yaml"
    config.write_text(
        """route:\n  frame_id: map\n  coordinate_mode: origin_relative_xy\n  origin: {latitude: 35.0, longitude: 139.0}\n  waypoints:\n    - {id: a, x: 2.0, y: -3.0, yaw: 0.0}\n""",
        encoding="utf-8",
    )
    local = WaypointConfigLoader().load(config, "route")
    assert local.projection_points()[0].latitude == 35.0
    assert [(w.x, w.y) for w in local.with_projected_points(((10.0, 20.0),)).waypoints] == [(12.0, 17.0)]
    config.write_text(
        """route:\n  frame_id: map\n  coordinate_mode: geodetic\n  waypoints:\n    - {id: a, latitude: 35.0, longitude: 139.0, yaw: 0.0}\n""",
        encoding="utf-8",
    )
    geodetic = WaypointConfigLoader().load(config, "route")
    assert len(geodetic.projection_points()) == 1
    assert [(w.x, w.y) for w in geodetic.with_projected_points(((1.5, 2.5),)).waypoints] == [(1.5, 2.5)]


def test_duplicate_yaml_keys_are_rejected(tmp_path):
    bad = tmp_path / "duplicate.yaml"
    bad.write_text("tasks: {}\ntasks: {}\n", encoding="utf-8")
    with pytest.raises(RegistryError, match="duplicate YAML key"):
        TaskRegistry.from_file(bad)
