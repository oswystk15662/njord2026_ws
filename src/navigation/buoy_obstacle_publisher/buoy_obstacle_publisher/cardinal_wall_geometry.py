"""ROS-independent geometry used by the cardinal wall publisher."""

import math


CARDINAL_DIRECTIONS = {
    2: "N",
    3: "E",
    4: "S",
    5: "W",
}


def direction_for_class(class_id, course_heading_rad):
    """Return the forbidden-side direction for cardinal or lateral marks.

    Red lateral marks must stay to port (left of the course heading), so the
    wall blocks starboard.  Green marks do the inverse.
    """
    if class_id in CARDINAL_DIRECTIONS:
        # ``course_heading_rad`` is the map-frame angle of true north. It is
        # estimated from the GNSS dual-antenna heading and map->base TF.
        # The wall blocks the opposite geographical side.
        north_x = math.cos(course_heading_rad)
        north_y = math.sin(course_heading_rad)
        east_x, east_y = north_y, -north_x
        cardinal = CARDINAL_DIRECTIONS[class_id]
        if cardinal == "N":
            return -north_x, -north_y
        if cardinal == "E":
            return -east_x, -east_y
        if cardinal == "S":
            return north_x, north_y
        return east_x, east_y
    forward_x = math.cos(course_heading_rad)
    forward_y = math.sin(course_heading_rad)
    if class_id == 1:  # red: keep mark on port, exclude starboard
        return forward_y, -forward_x
    if class_id == 0:  # green: keep mark on starboard, exclude port
        return -forward_y, forward_x
    return None


def _ray_end(bounds, x, y, dx, dy):
    """Return where the positive ray (x,y)+t*(dx,dy) meets the rectangle."""
    min_x, max_x, min_y, max_y = bounds
    candidates = []
    if dx > 1.0e-9:
        candidates.append((max_x - x) / dx)
    elif dx < -1.0e-9:
        candidates.append((min_x - x) / dx)
    if dy > 1.0e-9:
        candidates.append((max_y - y) / dy)
    elif dy < -1.0e-9:
        candidates.append((min_y - y) / dy)
    valid = [t for t in candidates if t >= 0.0]
    if not valid:
        return x, y
    t = min(valid)
    return x + t * dx, y + t * dy


def is_behind_retirement_frontier(marker_x, marker_y, frontier_x, frontier_y,
                                  course_heading_rad, margin_m=0.0):
    """Whether a marker is behind the reached next-waypoint frontier.

    The frontier advances along the main waypoint direction.  A marker's
    track remains available for display/re-identification, but its wall is
    retired once the frontier has passed the marker by ``margin_m``.
    """
    forward_x = math.cos(course_heading_rad)
    forward_y = math.sin(course_heading_rad)
    return (
        (marker_x - frontier_x) * forward_x
        + (marker_y - frontier_y) * forward_y
    ) < -max(0.0, margin_m)


def wall_points(bounds, wall_width, spacing, x, y, class_id, course_heading_rad=0.0):
    """Return a filled wall from a marker to its forbidden course edge."""
    direction = direction_for_class(class_id, course_heading_rad)
    if direction is None:
        return []
    dx, dy = direction
    length = math.hypot(dx, dy)
    dx, dy = dx / length, dy / length
    end_x, end_y = _ray_end(bounds, x, y, dx, dy)

    spacing = max(0.02, spacing)
    longitudinal_steps = max(1, math.ceil(math.hypot(end_x - x, end_y - y) / spacing))
    lateral_steps = max(1, math.ceil(wall_width / spacing))
    points = []
    for i in range(longitudinal_steps + 1):
        fraction = i / longitudinal_steps
        px = x + (end_x - x) * fraction
        py = y + (end_y - y) * fraction
        for j in range(lateral_steps + 1):
            offset = -wall_width / 2.0 + wall_width * j / lateral_steps
            points.append((px - offset * dy, py + offset * dx, 0.0))
    return points
