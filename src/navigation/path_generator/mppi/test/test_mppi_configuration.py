"""Regression checks for MPPI planner configuration without ROS or PyTorch."""

import ast
import os

import yaml


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLANNER_FILE = os.path.join(
    PACKAGE_DIR, "asv_trajectory_planner", "planner_node.py"
)
PARAMS_FILE = os.path.join(PACKAGE_DIR, "config", "mppi_params.yaml")
REAL_LAUNCH_FILE = os.path.join(PACKAGE_DIR, "launch", "planner_real.launch.py")
SIM_LAUNCH_FILE = os.path.join(
    PACKAGE_DIR, "launch", "planner_with_follow_path.launch.py"
)


def _literal_parameter_defaults():
    with open(PLANNER_FILE, "r", encoding="utf-8") as stream:
        tree = ast.parse(stream.read())

    defaults = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if not (
            isinstance(function, ast.Attribute)
            and function.attr == "declare_parameter"
            and len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
        ):
            continue
        try:
            defaults[node.args[0].value] = ast.literal_eval(node.args[1])
        except (ValueError, TypeError):
            pass
    return defaults


def test_yaml_mppi_values_match_planner_defaults():
    with open(PARAMS_FILE, "r", encoding="utf-8") as stream:
        yaml_params = yaml.safe_load(stream)["planner_node"]["ros__parameters"]

    defaults = _literal_parameter_defaults()
    assert yaml_params["planning_frequency"] == defaults["planning_frequency"]

    mppi = yaml_params["mppi"]
    assert len(mppi) == 21
    for key, value in mppi.items():
        if key == "target_speed":
            assert value == 2.0 * 1852.0 / 3600.0
        else:
            assert value == defaults[f"mppi.{key}"]


def test_opponent_timeout_is_declared_and_wired_in_both_launches():
    defaults = _literal_parameter_defaults()
    assert defaults["other_ship_timeout_sec"] == 2.0

    for launch_file in (REAL_LAUNCH_FILE, SIM_LAUNCH_FILE):
        with open(launch_file, "r", encoding="utf-8") as stream:
            source = stream.read()
        assert '"other_ship_timeout_sec": 2.0' in source


def test_real_launch_loads_shared_mppi_yaml():
    with open(REAL_LAUNCH_FILE, "r", encoding="utf-8") as stream:
        source = stream.read()
    assert '"mppi_params.yaml"' in source
    assert "mppi_params_file" in source
