"""Static (AST/text-only) checks for launch/task2_real.launch.py.

No `launch` / ROS imports: safe to run on machines without ROS 2. Asserts:

  (a) none of the six sim-only executables are referenced outside
      comments/docstring,
  (b) the dry_run / send_thruster_commands arguments exist with SAFE defaults
      ('true' / 'false'),
  (c) the thruster enable condition references all three gates
      (enable_thrusters, dry_run, send_thruster_commands),
  (d) quay-wall perception stays removed from the Task 2 integration
      (it is preserved on the feature/quay-perception branch).
"""

import ast
import io
import os
import tokenize

import yaml

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "launch",
    "task2_real.launch.py",
)

REPO_ROOT = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

NAV2_PARAMS_FILE = os.path.join(
    REPO_ROOT, "src", "robot", "config", "nav2_params_task2.yaml"
)
PLANNER_NODE_FILE = os.path.join(
    REPO_ROOT,
    "src", "navigation", "path_generator", "mppi",
    "asv_trajectory_planner", "planner_node.py",
)
MPPI_PARAMS_FILE = os.path.join(
    REPO_ROOT,
    "src", "navigation", "path_generator", "mppi",
    "config", "mppi_params.yaml",
)
RECORD_BAG_SCRIPT = os.path.join(
    REPO_ROOT, "scripts", "task2", "record_task2_bag.sh"
)

SIM_ONLY_EXECUTABLES = [
    "opponent_twist_from_tf_node",
    "task2_gps_waypoint_publisher",
    "opponent_vessel_node",
    "ideal_lidar_pointcloud_node",
    "dutyed_tf_pub_with_disturbance",
    "sim_thruster_command_adapter",
]


def _read_source():
    with open(LAUNCH_FILE, "r", encoding="utf-8") as f:
        return f.read()


def _code_without_comments_and_docstring(source):
    """Return source text with comments and the module docstring removed."""
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
    for tok in tokenize.generate_tokens(io.StringIO(source).readline):
        if tok.type == tokenize.COMMENT:
            continue
        if (
            tok.type == tokenize.STRING
            and docstring_pos is not None
            and tok.start == docstring_pos
        ):
            continue
        kept.append(tok.string)

    return " ".join(kept)


def _declared_arguments(source):
    """Map DeclareLaunchArgument name -> default_value (constants only)."""
    tree = ast.parse(source)
    args = {}

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue

        func = node.func
        func_name = getattr(func, "id", None) or getattr(func, "attr", None)
        if func_name != "DeclareLaunchArgument":
            continue

        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue

        name = node.args[0].value
        default = None
        for kw in node.keywords:
            if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                default = kw.value.value

        args[name] = default

    return args


def test_launch_file_exists():
    assert os.path.isfile(LAUNCH_FILE), f"Missing launch file: {LAUNCH_FILE}"


def test_launch_file_parses():
    ast.parse(_read_source())


def test_no_sim_only_executables_in_code():
    code = _code_without_comments_and_docstring(_read_source())
    for name in SIM_ONLY_EXECUTABLES:
        assert name not in code, (
            f"Sim-only executable '{name}' is referenced in task2_real.launch.py "
            "outside of comments/docstring"
        )


def test_sim_only_warning_documented():
    # The docstring must warn about every sim-only executable.
    docstring = ast.get_docstring(ast.parse(_read_source())) or ""
    for name in SIM_ONLY_EXECUTABLES:
        assert name in docstring, (
            f"Docstring warning must mention sim-only executable '{name}'"
        )


def test_dry_run_default_is_safe():
    args = _declared_arguments(_read_source())
    assert "dry_run" in args, "dry_run launch argument must be declared"
    assert args["dry_run"] == "true", "dry_run must default to 'true' (SAFE)"


def test_send_thruster_commands_default_is_safe():
    args = _declared_arguments(_read_source())
    assert "send_thruster_commands" in args, (
        "send_thruster_commands launch argument must be declared"
    )
    assert args["send_thruster_commands"] == "false", (
        "send_thruster_commands must default to 'false' (SAFE)"
    )


def test_thruster_condition_references_all_three_gates():
    tree = ast.parse(_read_source())
    source = _read_source()

    segment = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if getattr(target, "id", None) == "thruster_allowed":
                    segment = ast.get_source_segment(source, node.value)

    assert segment is not None, (
        "Expected an assignment to 'thruster_allowed' holding the thruster "
        "enable expression"
    )

    for gate in ("enable_thrusters", "dry_run", "send_thruster_commands"):
        assert gate in segment, (
            f"Thruster enable expression must reference the '{gate}' gate"
        )


def test_thruster_gate_truth_table():
    # Reconstruct the PythonExpression fragment list and evaluate it for all
    # eight gate combinations. Only (enable=true, dry_run=false, send=true)
    # may enable thrusters — this pins operators and polarity, not just the
    # presence of the three names.
    tree = ast.parse(_read_source())

    fragments = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if getattr(target, "id", None) == "thruster_allowed":
                    call = node.value
                    assert isinstance(call, ast.Call), (
                        "thruster_allowed must be a PythonExpression(...) call"
                    )
                    assert isinstance(call.args[0], ast.List)
                    fragments = call.args[0].elts

    assert fragments is not None, "thruster_allowed assignment not found"

    import itertools

    for et, dr, stc in itertools.product(("true", "false"), repeat=3):
        values = {
            "enable_thrusters": et,
            "dry_run": dr,
            "send_thruster_commands": stc,
        }
        parts = []
        for frag in fragments:
            if isinstance(frag, ast.Constant):
                parts.append(str(frag.value))
            elif isinstance(frag, ast.Name):
                assert frag.id in values, (
                    f"Unexpected name '{frag.id}' in thruster gate expression"
                )
                parts.append(values[frag.id])
            else:
                raise AssertionError(
                    "Thruster gate expression must consist only of string "
                    "constants and the three gate LaunchConfiguration names"
                )
        result = eval("".join(parts))  # noqa: S307 - reconstructed literal
        expected = et == "true" and dr == "false" and stc == "true"
        assert result == expected, (
            f"Gate truth table mismatch for enable_thrusters={et}, "
            f"dry_run={dr}, send_thruster_commands={stc}: "
            f"got {result}, expected {expected}"
        )


def test_thruster_expression_is_used_for_enable_thruster():
    # The real_bringup include must receive enable_thruster from the gated
    # thruster_allowed expression, not a constant.
    tree = ast.parse(_read_source())

    wired = False
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if (
                isinstance(key, ast.Constant)
                and key.value == "enable_thruster"
                and isinstance(value, ast.Name)
                and value.id == "thruster_allowed"
            ):
                wired = True

    assert wired, (
        "enable_thruster must be wired to the thruster_allowed expression in "
        "the real_bringup launch_arguments dict"
    )


# ----------------------------------------------------------------------
# Quay-wall removal guards: quay perception was moved out of Task 2 (it is
# preserved on the feature/quay-perception branch) and must not silently
# reappear in the integration files.
# ----------------------------------------------------------------------

def _contains_quay_key(node):
    """Recursively check a yaml.safe_load result for keys containing 'quay'."""
    if isinstance(node, dict):
        for key, value in node.items():
            if isinstance(key, str) and "quay" in key.lower():
                return True
            if _contains_quay_key(value):
                return True
    elif isinstance(node, list):
        for item in node:
            if _contains_quay_key(item):
                return True
    return False


def test_launch_file_has_no_quay_references():
    source = _read_source()
    for needle in ("quay_wall_detector", "enable_quay_detection", "/quay_wall"):
        assert needle not in source, (
            f"task2_real.launch.py must not reference '{needle}' (quay "
            "perception was removed from Task 2)"
        )


def test_nav2_params_have_no_quay_source():
    with open(NAV2_PARAMS_FILE, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)

    for costmap in ("local_costmap", "global_costmap"):
        ros_params = params[costmap][costmap]["ros__parameters"]
        obstacle_layer = ros_params["obstacle_layer"]

        sources = obstacle_layer["observation_sources"]
        assert "quay" not in sources, (
            f"{costmap} observation_sources must not include a quay source"
        )
        assert not _contains_quay_key(obstacle_layer), (
            f"{costmap} obstacle_layer must not contain a quay block"
        )


def test_planner_node_has_no_quay_subscription():
    with open(PLANNER_NODE_FILE, "r", encoding="utf-8") as f:
        source = f.read()

    assert "/quay_wall" not in source, (
        "planner_node.py must not subscribe to any /quay_wall topic"
    )
    assert "enable_mppi_quay_cost" not in source, (
        "planner_node.py must not declare enable_mppi_quay_cost"
    )


def test_mppi_params_have_no_quay_keys():
    with open(MPPI_PARAMS_FILE, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)

    assert not _contains_quay_key(params), (
        "mppi_params.yaml must not contain any quay parameter"
    )


def test_record_bag_script_has_no_quay_topics():
    with open(RECORD_BAG_SCRIPT, "r", encoding="utf-8") as f:
        script = f.read()

    assert "/quay_wall" not in script, (
        "record_task2_bag.sh must not record /quay_wall topics"
    )
