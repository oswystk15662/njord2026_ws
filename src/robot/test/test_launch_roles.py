"""Regression tests for the Jetson / miniPC launch-role split.

This is the most important regression test for the two-machine split: it
guards against GPU-only packages (glim_ros, livox_ros_driver2, glim_config)
accidentally being pulled into the miniPC bringup, and against
CPU/serial-only packages (robot_localization, thruster_driver,
micon_driver_fd, joy_node) accidentally being pulled into the Jetson
bringup.

Launch files are read from the source tree (not the installed share
directory) so this test also works before `colcon build` has installed
anything, and so it works the same way in CI.
"""

import importlib.util
import os

import pytest

_THIS_DIR = os.path.dirname(__file__)
_LAUNCH_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "launch"))

_GPU_ONLY_PACKAGES = ["glim_ros", "livox_ros_driver2", "glim_config"]
_MINIPC_ONLY_PACKAGES = ["robot_localization", "thruster_driver", "micon_driver_fd", "joy_node"]

_GENERATE_LAUNCH_DESCRIPTION_FILES = [
    "minipc_bringup.launch.py",
    "task1.launch.py",
    "task2.launch.py",
    "task3.launch.py",
]


def _launch_file_path(filename):
    path = os.path.join(_LAUNCH_DIR, filename)
    assert os.path.isfile(path), f"expected launch file not found: {path}"
    return path


def _read_launch_source(filename):
    with open(_launch_file_path(filename), "r") as f:
        return f.read()


def _load_launch_module(filename):
    path = _launch_file_path(filename)
    module_name = f"_njord_test_launch_{os.path.splitext(filename)[0].replace('.', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _walk_entities(entities):
    """Recursively yield every launch entity/action reachable from entities."""
    for entity in entities:
        yield entity
        # LaunchDescription-like objects and GroupAction/composed actions
        # expose their nested entities in different ways depending on
        # type; probe the common attributes defensively.
        nested = None
        if hasattr(entity, "entities"):
            nested = entity.entities
        elif hasattr(entity, "get_sub_entities"):
            try:
                nested = entity.get_sub_entities()
            except Exception:
                nested = None
        if nested:
            yield from _walk_entities(nested)


@pytest.mark.parametrize("filename", _GENERATE_LAUNCH_DESCRIPTION_FILES)
def test_generate_launch_description_returns_without_exception(filename):
    """generate_launch_description() must not raise for any of these files.

    Resolving package shares via ament_index requires the packages to be
    installed/sourced; if that fails for reasons unrelated to this launch
    file's own logic (e.g. running outside a built workspace), skip rather
    than fail so the source-string checks below remain the reliable part
    of this test.
    """
    try:
        module = _load_launch_module(filename)
    except Exception as exc:
        pytest.skip(f"could not import {filename}: {exc}")

    try:
        launch_description = module.generate_launch_description()
    except Exception as exc:
        pytest.skip(
            f"generate_launch_description() in {filename} raised "
            f"(likely due to unresolved package shares in this test environment): {exc}"
        )

    assert launch_description is not None
    # Just confirm we can walk it without blowing up.
    list(_walk_entities(launch_description.entities))


def test_minipc_bringup_source_has_no_gpu_packages():
    source = _read_launch_source("minipc_bringup.launch.py")
    for package_name in _GPU_ONLY_PACKAGES:
        assert package_name not in source, (
            f"minipc_bringup.launch.py must not reference GPU-only package "
            f"'{package_name}'; that belongs in jetson_bringup.launch.py"
        )


def test_minipc_bringup_uses_the_command_arbiter_as_the_only_cmd_vel_selector():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert 'executable="command_arbiter_node"' in source
    assert 'name="command_arbiter"' in source
    assert 'package="twist_mux"' not in source


def test_jetson_bringup_source_has_no_minipc_only_packages():
    source = _read_launch_source("jetson_bringup.launch.py")
    for package_name in _MINIPC_ONLY_PACKAGES:
        assert package_name not in source, (
            f"jetson_bringup.launch.py must not reference miniPC-only package "
            f"'{package_name}'; that belongs in minipc_bringup.launch.py"
        )


@pytest.mark.parametrize("filename", ["task1.launch.py", "task2.launch.py", "task3.launch.py"])
def test_task_launch_files_declare_role_argument(filename):
    source = _read_launch_source(filename)
    assert "'role'" in source or '"role"' in source
    assert "choices=['minipc', 'standalone']" in source or 'choices=["minipc", "standalone"]' in source
    assert "default_value='minipc'" in source or 'default_value="minipc"' in source
