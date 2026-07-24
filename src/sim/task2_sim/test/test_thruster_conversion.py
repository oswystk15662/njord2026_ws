"""Tests for the simulator's real-command compatibility adapter."""

import math

import pytest

from task2_sim.thruster_conversion import (
    four_thruster_forces_to_differential_duties,
)


ANGLES = [math.pi / 4.0, -math.pi / 4.0, 3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0]
REVERSE = [False, True, False, True]
POSITIONS = [(0.1803, -0.25), (0.1803, 0.25), (-0.1803, -0.25), (-0.1803, 0.25)]


def convert(forces):
    return four_thruster_forces_to_differential_duties(
        forces, ANGLES, REVERSE, POSITIONS, 0.35, 50.0, 40.0, 1000
    )


def test_symmetric_physical_surge_maps_to_equal_sides():
    left, right = convert([10.0, -10.0, -10.0, 10.0])
    assert left == pytest.approx(right, abs=1)
    assert left > 0


def test_yaw_wrench_maps_to_opposed_sides():
    left, right = convert([-10.0, -10.0, 10.0, 10.0])
    assert left > 0
    assert right < 0


def test_output_is_two_channels_and_clamped():
    assert convert([1000.0, -1000.0, -1000.0, 1000.0]) == [1000, 1000]


@pytest.mark.parametrize("resolution", [0, -1])
def test_invalid_resolution_is_rejected(resolution):
    with pytest.raises(ValueError):
        four_thruster_forces_to_differential_duties(
            [1.0] * 4, ANGLES, REVERSE, POSITIONS, 0.35, 50.0, 40.0, resolution
        )
