from pathlib import Path


def test_task2_autonomy_launch_wires_mppi_velocity_through_arbiter():
    text = (Path(__file__).parents[1] / "launch" / "task2_autonomy.launch.py").read_text()

    assert '"planner_real.launch.py"' in text
    assert '"navigation_launch_task2.py"' in text
    assert '"auto_cmd_vel_topic": "/cmd_vel_nav"' in text
    assert 'executable="task2_autonomy_ready_node"' in text
    assert '"params_file"' in text
    assert '"params_file": params_file' in text
    assert '"task2_params_file": params_file' in text
    assert '"opponent_motion_mode"' not in text
    assert '"path_topic": "/planned_path_pruned"' in text
    assert '"enable_follow_path_client": enable_nav2' in text
    assert '"enable_safety_cloud": "true"' in text
    assert 'executable="safety_cloud_gate_node"' in text
    assert "manual_control.launch.py" in text
    assert "command_arbiter_node" not in text
    assert "real_bringup.launch.py" not in text


def test_task2_nav2_launch_is_follow_path_only():
    text = (Path(__file__).parents[1] / "launch" / "navigation_launch_task2.py").read_text()

    assert '"node_names": ["controller_server", "velocity_smoother", "collision_monitor"]' in text
    assert 'package="nav2_collision_monitor"' in text
    for unused_node in (
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ):
        assert f"name='{unused_node}'" not in text


def test_task2_two_machine_split_keeps_controller_and_stop_on_minipc():
    root = Path(__file__).parents[1]
    controller = (root / "launch" / "task2_controller.launch.py").read_text()
    perception = (
        root.parent / "detection" / "task2_perception" / "launch" /
        "task2_perception.launch.py"
    ).read_text()
    humble_params = (root / "config" / "nav2_params_task2_humble.yaml").read_text()

    assert '"navigation_launch_task2.py"' in controller
    assert 'executable="follow_path_client_node"' in controller
    assert 'executable="task2_autonomy_ready_node"' in controller
    assert 'executable="safety_cloud_gate_node"' in controller
    assert "command_arbiter" not in controller
    assert '"enable_safety_cloud"' in perception
    assert '"params_file", default_value=default_params_file' in perception
    assert 'name="task2_safety_cloud_filter"' in perception
    assert 'package="nav2_collision_monitor"' in (
        root / "launch" / "navigation_launch_task2.py"
    ).read_text()
    assert 'topic: "/task2/safety_points"' in humble_params
    assert 'action_type: "stop"' in humble_params
    assert 'cmd_vel_out_topic: "/cmd_vel_nav"' in humble_params
    assert 'min_points: 1' in humble_params
