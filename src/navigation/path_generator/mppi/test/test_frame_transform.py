"""Unit tests for planner frame conversion without ROS or PyTorch."""

import math

import pytest

from asv_trajectory_planner.frame_transform import (
    transform_planar_pose,
    yaw_from_quaternion,
)


def test_planar_transform_rotates_then_translates():
    x, y, yaw = transform_planar_pose(
        2.0, 0.0, math.pi / 4.0, 10.0, -3.0, math.pi / 2.0
    )
    assert x == pytest.approx(10.0)
    assert y == pytest.approx(-1.0)
    assert yaw == pytest.approx(3.0 * math.pi / 4.0)


def test_yaw_round_trip():
    expected = -1.2
    assert yaw_from_quaternion(
        0.0, 0.0, math.sin(expected / 2.0), math.cos(expected / 2.0)
    ) == pytest.approx(expected)
