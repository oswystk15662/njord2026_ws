import math


def catmull_rom_point(p0, p1, p2, p3, t):
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    t2 = t * t
    t3 = t2 * t

    x = 0.5 * (
        2.0 * x1
        + (-x0 + x2) * t
        + (2.0 * x0 - 5.0 * x1 + 4.0 * x2 - x3) * t2
        + (-x0 + 3.0 * x1 - 3.0 * x2 + x3) * t3
    )

    y = 0.5 * (
        2.0 * y1
        + (-y0 + y2) * t
        + (2.0 * y0 - 5.0 * y1 + 4.0 * y2 - y3) * t2
        + (-y0 + 3.0 * y1 - 3.0 * y2 + y3) * t3
    )

    return x, y


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def generate_catmull_rom_path(waypoints, sample_interval):
    if len(waypoints) < 2:
        return waypoints

    points = [waypoints[0]] + waypoints + [waypoints[-1]]
    result = []

    for i in range(1, len(points) - 2):
        p0 = points[i - 1]
        p1 = points[i]
        p2 = points[i + 1]
        p3 = points[i + 2]

        seg_len = distance(p1, p2)
        n_samples = max(2, int(seg_len / sample_interval))

        for j in range(n_samples):
            t = j / float(n_samples)
            result.append(catmull_rom_point(p0, p1, p2, p3, t))

    result.append(waypoints[-1])
    return result