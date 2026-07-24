"""Small planar-frame helpers used by the ROS planner node."""

import math


def yaw_from_quaternion(x, y, z, w):
    """Return yaw from a quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def transform_planar_pose(x, y, yaw, transform_x, transform_y, transform_yaw):
    """Apply a target<-source planar transform to a pose in source."""
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    return (
        transform_x + cosine * x - sine * y,
        transform_y + sine * x + cosine * y,
        yaw + transform_yaw,
    )
