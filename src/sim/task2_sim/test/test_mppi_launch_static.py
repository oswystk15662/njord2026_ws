"""Static wiring checks for the Task 2 MPPI simulator."""

import ast
import os


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PACKAGE_DIR, "launch", "task2_sim.launch.py")


def _source():
    with open(LAUNCH_FILE, "r", encoding="utf-8") as stream:
        return stream.read()


def test_launch_parses_and_enables_mppi_by_default():
    source = _source()
    ast.parse(source)
    assert 'DeclareLaunchArgument("use_mppi", default_value="true")' in source
    assert '"planner_with_follow_path.launch.py"' in source


def test_simulator_uses_dedicated_command_adapter():
    source = _source()
    assert '"sim_thruster_command_adapter"' in source
    assert '"/sim/thruster_duty"' in source
    assert "/cmd_vel_thruster" not in source


def test_simulation_keeps_ideal_opponent_sources_out_of_real_launch():
    source = _source()
    assert '"opponent_vessel_node"' in source
    assert '"ideal_lidar_pointcloud_node"' in source
    assert '"navigation_launch_task2.py"' in source
