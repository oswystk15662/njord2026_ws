"""Pure conversion helpers for the Task 2 simulator."""

import math


def _force_to_duty(newtons, max_forward_newton, max_reverse_newton, resolution):
    maximum = max_forward_newton if newtons >= 0.0 else max_reverse_newton
    if maximum <= 0.0:
        raise ValueError("simulator maximum thrust must be positive")
    return max(
        -resolution,
        min(resolution, int(round(float(newtons) / maximum * resolution))),
    )


def four_thruster_forces_to_differential_duties(
    forces_newtons,
    angles_rad,
    reverse,
    positions_xy,
    simulator_half_beam_m,
    simulator_max_forward_newton,
    simulator_max_reverse_newton,
    duty_resolution,
):
    """Map the real four-thruster wrench to simulator [left, right] duties.

    The real driver publishes motor-signed forces after applying each
    ``reverse`` flag.  Undoing that flag recovers physical thrust along the
    configured angle.  Surge and yaw are then projected onto the simulator's
    differential pair; sway cannot be represented by that model.
    """
    resolution = int(duty_resolution)
    if resolution <= 0:
        raise ValueError("duty_resolution must be positive")
    if simulator_half_beam_m <= 0.0:
        raise ValueError("simulator_half_beam_m must be positive")
    if not (
        len(forces_newtons)
        == len(angles_rad)
        == len(reverse)
        == len(positions_xy)
        == 4
    ):
        raise ValueError("exactly four matched real-thruster values are required")

    surge = 0.0
    yaw_moment = 0.0
    for force, angle, is_reversed, position in zip(
        forces_newtons, angles_rad, reverse, positions_xy
    ):
        physical_force = float(force) * (-1.0 if is_reversed else 1.0)
        force_x = physical_force * math.cos(float(angle))
        force_y = physical_force * math.sin(float(angle))
        x, y = (float(value) for value in position)
        surge += force_x
        yaw_moment += x * force_y - y * force_x

    left_force = 0.5 * (surge - yaw_moment / simulator_half_beam_m)
    right_force = 0.5 * (surge + yaw_moment / simulator_half_beam_m)
    return [
        _force_to_duty(
            left_force,
            simulator_max_forward_newton,
            simulator_max_reverse_newton,
            resolution,
        ),
        _force_to_duty(
            right_force,
            simulator_max_forward_newton,
            simulator_max_reverse_newton,
            resolution,
        ),
    ]
