"""Regression contracts for the real-vessel Task2 waypoint source."""

from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "asv_trajectory_planner" /
          "task2_waypoint_pose_publisher.py").read_text(encoding="utf-8")


def test_task2_projects_gps_waypoints_one_at_a_time():
    assert "exactly one request in flight" in SOURCE
    assert "self.projection_future = self.from_ll_client.call_async(" in SOURCE
    assert "self.projection_index += 1" in SOURCE
    assert "self.projection_futures" not in SOURCE


def test_task2_projection_order_is_start_then_goal():
    assert "points = [self.start_wp, self.goal_wp]" in SOURCE
    assert "Resolved Task2 GPS5 then GPS6 coordinates" in SOURCE


def test_task2_final_completion_requires_a_path_ending_at_gps6():
    source = (Path(__file__).parents[1] / "asv_trajectory_planner" /
              "follow_path_client_node.py").read_text(encoding="utf-8")
    assert 'self.declare_parameter("final_goal_pose_topic", "/waypoint2_pose")' in source
    assert "if not self._path_ends_at_final_goal():" in source
    assert "waiting for a path terminating at Task2 GPS6" in source


def test_task2_autonomy_readiness_requires_path_and_follow_path_only():
    source = (Path(__file__).parents[1] / "asv_trajectory_planner" /
              "task2_autonomy_ready_node.py").read_text(encoding="utf-8")
    assert "path_fresh and self.follow_path_client.server_is_ready()" in source
    assert "collision_monitor_active" not in source


def test_crm_visualization_includes_the_detected_buoy_outside_cost():
    source = (Path(__file__).parents[1] / "asv_trajectory_planner" /
              "planner_node.py").read_text(encoding="utf-8")
    assert "def _buoy_outside_risk_map(" in source
    assert "risk = np.maximum(risk, buoy_risk)" in source
    assert "outside_amount = d - (d_b - margin_m)" in source
    assert "outside_amount = (d_b + margin_m) - d" in source
