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
    assert registry.get("task1").frame_id == "odom"
    assert registry.get("task2").executor == "task2_mppi"
    assert registry.get("task2").frame_id == "odom"
    assert registry.get("task3_1").runnable
    assert registry.get("task3_1").frame_id == "odom"
    assert registry.get("task3_1").executor == "staged_docking"
    assert registry.get("task3_2").runnable
    assert registry.get("task3_2").nav2_profile == "task3"
    assert registry.get("task4").runnable
    assert registry.get("return_home").runnable
    assert registry.get("move_to_exam_field").runnable
    assert registry.get("move_to_exam_field").executor == "waypoint_sequence"
    assert registry.get("move_to_exam_field").frame_id == "odom"
    assert required_runtime_readiness(registry.get("task2")) == set()
    assert registry.get("task1_2").availability == "not_implemented"
    assert "Marker-driven" in registry.get("task1_2").reason


def test_existing_task_routes_load_without_changing_waypoint_files():
    loader = WaypointConfigLoader()
    task1 = loader.load(WAYPOINT_ROOT / "config/task1_waypoints.yaml", "task1_config")
    task2 = loader.load(WAYPOINT_ROOT / "config/task2_waypoints.yaml", "task2_config")
    task3 = loader.load(WAYPOINT_ROOT / "config/task3_waypoints.yaml", "task3_1_config")
    task3_2 = loader.load(WAYPOINT_ROOT / "config/task3_waypoints.yaml", "task3_2_config")
    task4 = loader.load(WAYPOINT_ROOT / "config/task4_waypoints.yaml", "task4_config")
    assert len(task1.waypoints) == 11
    assert task1.frame_id == task3.frame_id == task3_2.frame_id == "odom"
    assert [waypoint.waypoint_id for waypoint in task4.waypoints] == [
        waypoint.waypoint_id for waypoint in task1.waypoints
    ]
    assert len(task1.projection_points()) == len(task1.waypoints)
    assert task1.waypoints[6].competition_id == "3"
    assert task1.waypoints[-1].competition_id == "4"
    assert [waypoint.waypoint_id for waypoint in task2.waypoints] == ["5", "6"]
    assert [(waypoint.latitude, waypoint.longitude) for waypoint in task2.waypoints] == [
        (63.4408000000, 10.4233194444), (63.4404230000, 10.4229704000),
    ]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_1_gate")] == ["7"]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_1")] == ["8"]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_2")] == ["berth1"]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_3")] == ["8"]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_4")] == ["9"]
    assert [(waypoint.waypoint_id, waypoint.latitude, waypoint.longitude) for waypoint in task3.waypoints] == [
        ("7", 63.4409750000, 10.4237833333),
        ("8", 63.4409333333, 10.4240611111),
        ("berth1", 63.4409400000, 10.4240929000),
        ("9", 63.4408472222, 10.4240444444),
    ]
    assert [waypoint.waypoint_id for waypoint in task3_2.waypoints] == ["10", "11", "berth2", "12"]
    exam_field = loader.load(PACKAGE_ROOT / "config" / "move_to_exam_field_waypoints.yaml", "move_to_exam_field_config")
    assert exam_field.frame_id == "odom"
    assert [waypoint.waypoint_id for waypoint in exam_field.waypoints] == [
        "gps_0_1", "waypoint_1", "waypoint_2", "gps_0_2_1",
    ]


def test_task1_uses_gps1_as_start_not_navigation_goal():
    source = (PACKAGE_ROOT / "mission_manager" / "mission_manager_node.py").read_text()
    assert 'waypoint.competition_id != "1"' in source
    assert "Task1 route has no navigation waypoints after GPS1" in source


def test_action_waits_for_ros_timer_not_asyncio_loop():
    source = (PACKAGE_ROOT / "mission_manager" / "mission_manager_node.py").read_text()
    assert "from rclpy.task import Future" in source
    assert "await self._action_tick" in source
    assert "asyncio" not in source


def test_runtime_profile_switch_projects_geographic_route():
    source = (PACKAGE_ROOT / "mission_manager" / "mission_manager_node.py").read_text()
    runtime_result = source[source.index("def _on_runtime_result"):source.index("def _on_runtime_status")]
    assert "self._start_coordinate_projection(decision.execution_id, task, route, request)" in runtime_result
    assert "self._machine.transition(" not in runtime_result


def test_route_supports_latitude_longitude_coordinates(tmp_path):
    config = tmp_path / "coordinates.yaml"
    config.write_text(
        """route:\n  frame_id: map\n  waypoints:\n    - {id: a, latitude: 35.0, longitude: 139.0, yaw: 0.0}\n""",
        encoding="utf-8",
    )
    route = WaypointConfigLoader().load(config, "route")
    assert len(route.projection_points()) == 1
    assert [(w.x, w.y) for w in route.with_projected_points(((1.5, 2.5),)).waypoints] == [(1.5, 2.5)]


def test_route_supports_competition_dms_coordinates(tmp_path):
    config = tmp_path / "coordinates.yaml"
    config.write_text(
        """route:
  frame_id: map
  waypoints:
    - {id: a, latitude: "63°26′26.89″ N", longitude: "10°25′24.22″ E", yaw: 0.0}
""",
        encoding="utf-8",
    )
    waypoint = WaypointConfigLoader().load(config, "route").waypoints[0]
    assert waypoint.latitude == pytest.approx(63.4408027778)
    assert waypoint.longitude == pytest.approx(10.4233944444)


def test_route_rejects_collapsed_projection_for_separated_geographic_points(tmp_path):
    config = tmp_path / "coordinates.yaml"
    config.write_text(
        """route:\n  frame_id: map\n  waypoints:\n    - {id: a, latitude: 35.0, longitude: 139.0, yaw: 0.0}\n    - {id: b, latitude: 35.001, longitude: 139.001, yaw: 0.0}\n""",
        encoding="utf-8",
    )
    route = WaypointConfigLoader().load(config, "route")
    assert route.projection_is_degenerate(((0.0, 0.0), (0.0, 0.0)))
    assert not route.projection_is_degenerate(((0.0, 0.0), (100.0, 100.0)))


def test_route_rejects_one_collapsed_pair_in_otherwise_spread_projection(tmp_path):
    config = tmp_path / "coordinates.yaml"
    config.write_text(
        """route:\n  frame_id: map\n  waypoints:\n    - {id: a, latitude: 35.0, longitude: 139.0, yaw: 0.0}\n    - {id: b, latitude: 35.001, longitude: 139.001, yaw: 0.0}\n    - {id: c, latitude: 35.002, longitude: 139.002, yaw: 0.0}\n""",
        encoding="utf-8",
    )
    route = WaypointConfigLoader().load(config, "route")
    assert route.projection_is_degenerate(((0.0, 0.0), (100.0, 100.0), (100.0, 100.0)))


def test_duplicate_yaml_keys_are_rejected(tmp_path):
    bad = tmp_path / "duplicate.yaml"
    bad.write_text("tasks: {}\ntasks: {}\n", encoding="utf-8")
    with pytest.raises(RegistryError, match="duplicate YAML key"):
        TaskRegistry.from_file(bad)
