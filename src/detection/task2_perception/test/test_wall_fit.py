"""Test 8: wall_fit recovers a known noisy line segment (numpy only)."""

import numpy as np

from task2_perception import wall_fit


def make_wall_points(p0, p1, n=400, noise=0.05, z_lo=-0.3, z_hi=2.5, seed=0):
    rng = np.random.default_rng(seed)
    t = rng.uniform(0.0, 1.0, n)
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    xy = p0 + t[:, None] * (p1 - p0)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    normal = np.array([-d[1], d[0]])
    xy = xy + rng.normal(0.0, noise, n)[:, None] * normal
    z = rng.uniform(z_lo, z_hi, n)
    return np.column_stack([xy, z])


def make_water_points(n=500, z=-0.3, seed=1):
    rng = np.random.default_rng(seed)
    xy = rng.uniform([-25.0, -25.0], [25.0, 25.0], size=(n, 2))
    return np.column_stack([xy, z + rng.normal(0, 0.02, n)])


class TestWallFit:

    P0 = np.array([5.0, 10.0])
    P1 = np.array([25.0, 14.0])

    def test_recovers_known_segment(self):
        """Test 8: endpoints and length within tolerance despite noise."""
        wall = make_wall_points(self.P0, self.P1, noise=0.05)
        water = make_water_points()
        pts = np.vstack([wall, water])

        candidates = wall_fit.extract_wall_candidates(
            pts, min_z_extent_m=0.5, cell_size_m=0.5)
        # Water cells are flat -> rejected; wall cells kept.
        assert candidates[: len(wall)].mean() > 0.8
        assert candidates[len(wall):].mean() < 0.1

        segments = wall_fit.fit_lines_ransac(
            pts,
            distance_threshold_m=0.2,
            min_points=30,
            min_length_m=2.0,
            max_distance_m=40.0,
            candidate_mask=candidates,
            rng=np.random.default_rng(3))
        assert len(segments) == 1
        seg = segments[0]

        true_len = float(np.linalg.norm(self.P1 - self.P0))
        assert abs(seg.length - true_len) < 1.0  # endpoint spread tolerance
        ends = sorted([seg.p0, seg.p1], key=lambda p: p[0])
        np.testing.assert_allclose(ends[0], self.P0, atol=0.7)
        np.testing.assert_allclose(ends[1], self.P1, atol=0.7)
        assert seg.inlier_count >= 0.9 * len(wall)
        assert seg.inlier_points.shape[1] == 3

    def test_two_segments(self):
        wall_a = make_wall_points([0.0, 10.0], [20.0, 10.0], seed=2)
        wall_b = make_wall_points([-8.0, -20.0], [-8.0, 5.0], seed=3)
        pts = np.vstack([wall_a, wall_b])
        segments = wall_fit.fit_lines_ransac(
            pts, distance_threshold_m=0.2, min_points=30,
            min_length_m=2.0, max_distance_m=40.0,
            rng=np.random.default_rng(4))
        assert len(segments) == 2

    def test_distance_and_length_gates(self):
        far_wall = make_wall_points([100.0, 100.0], [120.0, 100.0], seed=5)
        segments = wall_fit.fit_lines_ransac(
            far_wall, distance_threshold_m=0.2, min_points=30,
            min_length_m=2.0, max_distance_m=40.0,
            rng=np.random.default_rng(6))
        assert segments == []

    def test_temporal_confirmation_and_timeout(self):
        tracker = wall_fit.WallTracker(confirmations=3, timeout_sec=2.0)
        wall = make_wall_points(self.P0, self.P1)
        segs = wall_fit.fit_lines_ransac(
            wall, distance_threshold_m=0.2, min_points=30,
            min_length_m=2.0, max_distance_m=40.0,
            rng=np.random.default_rng(5))
        assert len(segs) == 1

        # Frames 1-2: not yet confirmed. Frame 3: confirmed.
        assert tracker.update(segs, 0.0) == []
        assert tracker.update(segs, 0.1) == []
        confirmed = tracker.update(segs, 0.2)
        assert len(confirmed) == 1

        # Still reported while fresh even with no new detection...
        assert len(tracker.update([], 1.0)) == 1
        # ...and dropped after the timeout.
        assert tracker.update([], 5.0) == []

    def test_segments_to_occupancy(self):
        # A segment along +x through the grid center, margin 1 m, 0.5 m cells.
        grid = wall_fit.segments_to_occupancy(
            [(np.array([-5.0, 0.0]), np.array([5.0, 0.0]))],
            margin_m=1.0, resolution_m=0.5, size_m=20.0)
        assert grid.shape == (40, 40)
        # Center cell occupied.
        assert grid[20, 20] == 100
        # A far corner stays free.
        assert grid[0, 0] == 0
        # Occupied band is symmetric about the segment (row = y index).
        occupied_rows = np.flatnonzero(grid.any(axis=1))
        assert occupied_rows.min() >= 16 and occupied_rows.max() <= 23
