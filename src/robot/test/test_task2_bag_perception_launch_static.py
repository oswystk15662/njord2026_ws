"""Static checks for the self-contained Task 2 rosbag perception launch."""

import ast
import os


LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "launch",
    "task2_bag_perception.launch.py",
)


def _source():
    with open(LAUNCH_FILE, encoding="utf-8") as file:
        return file.read()


def test_launch_parses():
    ast.parse(_source())


def test_launches_bag_and_perception_chain():
    source = _source()
    for required in (
        '"ros2", "bag", "play"',
        '"task2_perception"',
        '"enable_buoy_selector": "true"',
        '"publish_buoy_detection_markers": "true"',
        '"pcl_preprocessing"',
        '"preprocessing.launch.py"',
        '"pcl_segmentation"',
        '"segmentation.launch.py"',
        'package="ship_tracking", executable="ship_tracker_node"',
        '"use_sim_time": "true"',
        'executable="bag_odometry_selector.py"',
        '"/task2/ego_odom"',
        'parameters=[LaunchConfiguration("params_file"), {"use_sim_time": True}]',
        '"task2_params.yaml"',
        '"map_frame": "odom"',
    ):
        assert required in source


def test_launch_is_perception_only():
    source = _source()
    for forbidden in (
        "thruster_driver",
        "serial_writer",
        "real_bringup.launch.py",
        "navigation_launch_task2.py",
    ):
        assert forbidden not in source


def test_bag_path_and_loop_are_configurable():
    source = _source()
    assert '"bag_path"' in source
    assert '"loop"' in source
    assert '"--loop"' in source


def test_bag_storage_is_auto_detected_by_default():
    source = _source()
    assert '"storage_id", default_value=""' in source
    assert 'if storage_id:' in source
    assert 'command.extend(["-s", storage_id])' in source


def test_bag_player_is_non_interactive():
    source = _source()
    assert '"--disable-keyboard-controls"' in source


def test_bag_uses_the_packaged_qos_override_by_default():
    source = _source()
    assert '"collision_avoidance_qos.yaml"' in source


def test_odometry_selector_keeps_ros_node_subscription_attribute_intact():
    selector = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "scripts", "bag_odometry_selector.py",
    )
    text = open(selector, encoding="utf-8").read()
    assert "self.odom_subscriptions" in text
    assert "self.subscriptions =" not in text
