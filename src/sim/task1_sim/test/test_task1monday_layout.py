import json
import math
from pathlib import Path

import yaml


PARAMS_PATH = Path(__file__).resolve().parents[1] / "config" / "task1_params.yaml"


def test_task1monday_layout_uses_gps1_enu_origin_and_surveyed_buoys():
    params = yaml.safe_load(PARAMS_PATH.read_text(encoding="utf-8"))[
        "task1_orchestrator"]["ros__parameters"]
    checkpoints = json.loads(params["gps_checkpoint_xy"])
    buoys = json.loads(params["buoy_position_xy"])

    assert checkpoints[0] == [0.0, 0.0]
    assert len(checkpoints) == 4
    assert len(buoys) == 4
    assert json.loads(params["buoy_marks"]) == ["RED", "E", "W", "GREEN"]

    gps3, gps4 = checkpoints[2], checkpoints[3]
    assert math.isclose(
        params["course_heading_rad"],
        math.atan2(gps4[1] - gps3[1], gps4[0] - gps3[0]),
        abs_tol=1.0e-3,
    )


def test_task1_launch_starts_navsat_projection_for_gps_waypoints():
    launch = (PARAMS_PATH.parents[1] / "launch" / "task1_sim.launch.py").read_text(
        encoding="utf-8")
    assert 'executable="navsat_transform_node"' in launch
    assert '"use_navsat"' in launch
    assert '("odometry/gps", "/odometry/gps/navsat")' in launch
    assert '"wait_for_datum": True' in launch
    assert '"datum": [63.4408027778, 10.4233944444, 0.0]' in launch


def test_task1_launch_starts_at_gps1_and_heads_to_first_surveyed_waypoint():
    launch = (PARAMS_PATH.parents[1] / "launch" / "task1_sim.launch.py").read_text(
        encoding="utf-8")
    assert 'Create the dynamics node at GPS1' in launch
    assert 'start_x, start_y = checkpoints[0]' in launch
    assert 'next_x, next_y = waypoint1[0]' in launch


def test_task1_navigation_allows_navsat_and_nav2_to_start_together():
    launch = (PARAMS_PATH.parents[1] / "launch" / "task1_navigation.launch.py").read_text(
        encoding="utf-8")
    assert '"bond_timeout": 10.0' in launch
