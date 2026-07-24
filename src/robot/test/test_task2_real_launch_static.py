"""Static checks for the dedicated real-vessel Task 2 MPPI launch."""

import ast
import io
import os
import tokenize

import yaml


ROBOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(ROBOT_DIR))
LAUNCH_FILE = os.path.join(ROBOT_DIR, "launch", "task2_real.launch.py")
NAV_LAUNCH_FILE = os.path.join(
    ROBOT_DIR, "launch", "navigation_launch_task2.py"
)
NAV2_PARAMS_FILE = os.path.join(
    ROBOT_DIR, "config", "nav2_params_task2_real.yaml"
)
PLANNER_NODE_FILE = os.path.join(
    REPO_ROOT,
    "src",
    "navigation",
    "path_generator",
    "mppi",
    "asv_trajectory_planner",
    "planner_node.py",
)
MPPI_PARAMS_FILE = os.path.join(
    REPO_ROOT,
    "src",
    "navigation",
    "path_generator",
    "mppi",
    "config",
    "mppi_params.yaml",
)

SIM_ONLY_EXECUTABLES = (
    "opponent_twist_from_tf_node",
    "task2_gps_waypoint_publisher",
    "opponent_vessel_node",
    "ideal_lidar_pointcloud_node",
    "dutyed_tf_pub_with_disturbance",
    "sim_thruster_command_adapter",
)


def _read(path):
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def _code_without_comments_and_docstring(source):
    tree = ast.parse(source)
    docstring_pos = None
    if (
        tree.body
        and isinstance(tree.body[0], ast.Expr)
        and isinstance(tree.body[0].value, ast.Constant)
        and isinstance(tree.body[0].value.value, str)
    ):
        node = tree.body[0].value
        docstring_pos = (node.lineno, node.col_offset)

    kept = []
    for token in tokenize.generate_tokens(io.StringIO(source).readline):
        if token.type == tokenize.COMMENT:
            continue
        if (
            token.type == tokenize.STRING
            and docstring_pos is not None
            and token.start == docstring_pos
        ):
            continue
        kept.append(token.string)
    return " ".join(kept)


def _declared_arguments(source):
    arguments = {}
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        name = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
        if name != "DeclareLaunchArgument" or not node.args:
            continue
        if not isinstance(node.args[0], ast.Constant):
            continue
        default = None
        for keyword in node.keywords:
            if (
                keyword.arg == "default_value"
                and isinstance(keyword.value, ast.Constant)
            ):
                default = keyword.value.value
        arguments[node.args[0].value] = default
    return arguments


def _contains_key_fragment(value, fragment):
    if isinstance(value, dict):
        return any(
            fragment in str(key).lower()
            or _contains_key_fragment(child, fragment)
            for key, child in value.items()
        )
    if isinstance(value, list):
        return any(_contains_key_fragment(child, fragment) for child in value)
    return False


def test_launch_files_parse():
    ast.parse(_read(LAUNCH_FILE))
    ast.parse(_read(NAV_LAUNCH_FILE))


def test_real_launch_excludes_sim_yolo_and_quay_code():
    code = _code_without_comments_and_docstring(_read(LAUNCH_FILE))
    for executable in SIM_ONLY_EXECUTABLES:
        assert executable not in code
    for forbidden in ("yolo11s.launch.py", "enable_yolo", "quay_wall"):
        assert forbidden not in code


def test_real_launch_defaults_match_test07089_operation():
    arguments = _declared_arguments(_read(LAUNCH_FILE))
    assert arguments["enable_camera"] == "false"
    for name in (
        "enable_lidar",
        "enable_ship_tracking",
        "enable_mppi",
        "enable_nav2",
        "enable_thrusters",
    ):
        assert arguments[name] == "true"


def test_thruster_passthrough_uses_enable_thrusters():
    tree = ast.parse(_read(LAUNCH_FILE))
    found = False
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if (
                isinstance(key, ast.Constant)
                and key.value == "enable_thruster"
                and isinstance(value, ast.Name)
                and value.id == "enable_thrusters"
            ):
                found = True
    assert found


def test_tracking_is_gated_by_lidar_and_tracking_flags():
    source = _read(LAUNCH_FILE)
    tree = ast.parse(source)
    expression = None
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if any(getattr(target, "id", None) == "tracking_enabled" for target in node.targets):
            expression = ast.get_source_segment(source, node.value)
    assert expression is not None
    assert "enable_lidar" in expression
    assert "enable_ship_tracking" in expression


def test_nav2_commands_pass_through_velocity_smoother():
    source = _read(NAV_LAUNCH_FILE)
    assert source.count("('cmd_vel', '/cmd_vel_nav_raw')") == 6
    assert source.count("('cmd_vel_smoothed', '/cmd_vel')") == 2
    assert "/cmd_vel_thruster" not in source


def test_real_costmaps_use_filtered_lidar_cloud():
    with open(NAV2_PARAMS_FILE, "r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    for costmap in ("local_costmap", "global_costmap"):
        obstacle_layer = params[costmap][costmap]["ros__parameters"]["obstacle_layer"]
        assert obstacle_layer["pointcloud"]["topic"] == "/task2/points_filtered"


def test_quay_perception_stays_out_of_planner_and_costmaps():
    planner_source = _read(PLANNER_NODE_FILE)
    assert "quay_wall" not in planner_source
    assert 'declare_parameter("other_ship_timeout_sec", 2.0)' in planner_source
    assert "other_twist=other_twist" in planner_source
    with open(MPPI_PARAMS_FILE, "r", encoding="utf-8") as stream:
        assert not _contains_key_fragment(yaml.safe_load(stream), "quay")
    with open(NAV2_PARAMS_FILE, "r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    for costmap in ("local_costmap", "global_costmap"):
        obstacle_layer = params[costmap][costmap]["ros__parameters"]["obstacle_layer"]
        assert not _contains_key_fragment(obstacle_layer, "quay")
