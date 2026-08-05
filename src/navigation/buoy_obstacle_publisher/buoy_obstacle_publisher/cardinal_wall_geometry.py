"""ROS-independent geometry used by the cardinal wall publisher."""

import math


CARDINAL_DIRECTIONS = {
    2: (0, -1),  # north marker: exclude south
    3: (-1, 0),  # east marker: exclude west
    4: (0, 1),   # south marker: exclude north
    5: (1, 0),   # west marker: exclude east
}


def wall_points(bounds, wall_width, spacing, x, y, class_id):
    """Return a filled wall from a marker to its forbidden course edge."""
    dx, dy = CARDINAL_DIRECTIONS[class_id]
    min_x, max_x, min_y, max_y = bounds
    if dx < 0:
        end_x, end_y = min_x, y
    elif dx > 0:
        end_x, end_y = max_x, y
    elif dy < 0:
        end_x, end_y = x, min_y
    else:
        end_x, end_y = x, max_y

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
            points.append((px + (offset if dy else 0.0), py + (offset if dx else 0.0), 0.0))
    return points
