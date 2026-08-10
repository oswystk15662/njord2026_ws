import json
import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "task1_waypoints.yaml"
NODE_PATH = PACKAGE_ROOT / "waypoint_publisher" / "waypoint_publisher_node.py"
SIM_PARAMS_PATH = PACKAGE_ROOT.parents[2] / "sim" / "task1_sim" / "config" / "task1_params.yaml"


def test_task1_navigates_from_wp_1_1_through_gps4_in_survey_order():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["task1_config"]
    ids = [waypoint["competition_id"] for waypoint in config["waypoints"]]
    assert ids == ["1", "1.1", "1.2", "1.3", "1.4", "1.5", "2", "3", "3.1", "3.2", "3.3", "3.4", "4"]
    assert ids[1:] == ["1.1", "1.2", "1.3", "1.4", "1.5", "2", "3", "3.1", "3.2", "3.3", "3.4", "4"]


def test_task1_treats_gps1_as_start_pose_not_navigation_goal():
    source = NODE_PATH.read_text(encoding="utf-8")
    assert "GPS1 denotes the vessel's start pose" in source
    assert "!= '1'" in source


def test_task1_sim_geometry_and_yaws_match_the_surveyed_geodetic_route():
    route = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["task1_config"]["waypoints"]
    sim = yaml.safe_load(SIM_PARAMS_PATH.read_text(encoding="utf-8"))["task1_orchestrator"]["ros__parameters"]

    # The Task1 map frame is local ENU metres with GPS1 as origin.
    earth_radius_m = 6378137.0
    latitude0 = math.radians(route[0]["latitude"])
    longitude0 = math.radians(route[0]["longitude"])

    def local_xy(waypoint):
        return (
            earth_radius_m * math.cos(latitude0) * (math.radians(waypoint["longitude"]) - longitude0),
            earth_radius_m * (math.radians(waypoint["latitude"]) - latitude0),
        )

    projected = [local_xy(waypoint) for waypoint in route]
    expected_sim_points = (
        json.loads(sim["gps_checkpoint_xy"]),
        json.loads(sim["waypoint1_xy"]),
        json.loads(sim["waypoint2_xy"]),
    )
    source_groups = (
        [projected[index] for index in (0, 6, 7, 12)],
        projected[1:6],
        projected[8:12],
    )
    for actual_group, expected_group in zip(expected_sim_points, source_groups):
        for actual, expected in zip(actual_group, expected_group):
            assert math.isclose(actual[0], expected[0], abs_tol=0.02)
            assert math.isclose(actual[1], expected[1], abs_tol=0.02)

    # A waypoint arrow faces the following surveyed point; GPS4 retains the
    # incoming direction because it has no following route point.
    for index, waypoint in enumerate(route):
        here = projected[index]
        following = projected[index + 1] if index + 1 < len(projected) else here
        previous = projected[index - 1] if index else here
        reference = following if index + 1 < len(projected) else previous
        if index + 1 < len(projected):
            expected_yaw = math.atan2(reference[1] - here[1], reference[0] - here[0])
        else:
            expected_yaw = math.atan2(here[1] - reference[1], here[0] - reference[0])
        assert math.isclose(waypoint["yaw"], expected_yaw, abs_tol=1.0e-3)
