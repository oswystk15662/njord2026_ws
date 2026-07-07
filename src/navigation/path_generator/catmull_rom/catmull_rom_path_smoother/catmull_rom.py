import math


def _point(p0, p1, p2, p3, t):
    t2 = t * t
    t3 = t2 * t

    x = 0.5 * (
        2.0 * p1[0]
        + (-p0[0] + p2[0]) * t
        + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        2.0 * p1[1]
        + (-p0[1] + p2[1]) * t
        + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
    )
    return x, y


def _distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def generate_catmull_rom_path(points, sample_interval):
    if len(points) < 2:
        return list(points)

    interval = max(float(sample_interval), 1.0e-6)
    padded = [points[0]] + list(points) + [points[-1]]
    result = []

    for i in range(1, len(padded) - 2):
        p0 = padded[i - 1]
        p1 = padded[i]
        p2 = padded[i + 1]
        p3 = padded[i + 2]

        segment_length = _distance(p1, p2)
        sample_count = max(2, int(math.ceil(segment_length / interval)))

        for j in range(sample_count):
            t = j / float(sample_count)
            result.append(_point(p0, p1, p2, p3, t))

    result.append(points[-1])
    return result
