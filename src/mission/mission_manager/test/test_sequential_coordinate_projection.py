from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "mission_manager"
    / "mission_manager_node.py"
).read_text(encoding="utf-8")


def test_mission_manager_projects_waypoints_one_at_a_time():
    assert "project in route order one point at a time" in SOURCE
    assert "self._request_next_coordinate_projection()" in SOURCE
    assert "if len(resolved) < len(points):" in SOURCE
    assert "for point in route.projection_points()" not in SOURCE
