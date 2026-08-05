from buoy_obstacle_publisher.cardinal_wall_geometry import wall_points


def test_north_marker_blocks_the_south_side():
    points = wall_points([-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 2.0, 2)
    assert points
    assert min(y for _, y, _ in points) == -4.0
    assert max(y for _, y, _ in points) == 2.0


def test_east_marker_blocks_the_west_side():
    points = wall_points([-5.0, 5.0, -4.0, 4.0], 0.2, 0.1, 1.0, 2.0, 3)
    assert points
    assert min(x for x, _, _ in points) == -5.0
    assert max(x for x, _, _ in points) == 1.0
