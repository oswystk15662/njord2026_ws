"""Pure-numpy quay-wall line extraction for Task 2 perception.

No ROS dependency (pytest-able with numpy only).

Pipeline:
  1. extract_wall_candidates(): keep points that belong to xy-grid cells with
     a large vertical (z) extent — a cheap proxy for "vertical structure".
  2. fit_lines_ransac(): sequential 2D RANSAC line fitting on the xy
     projection, segment endpoints from the principal-axis projection of the
     inliers, gates on length / point count / distance from the origin.
  3. WallTracker: temporal confirmation (a segment must recur N times within
     the timeout to be confirmed; unseen tracks time out).
  4. segments_to_occupancy(): rasterize confirmed segments (dilated by a
     safety margin) into an occupancy grid.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# ---------------------------------------------------------------------------
# Candidate extraction (vertical-extent grid)
# ---------------------------------------------------------------------------

def extract_wall_candidates(
    points: np.ndarray,
    min_z_extent_m: float = 0.5,
    cell_size_m: float = 0.5,
) -> np.ndarray:
    """Mask of points lying in xy-grid cells whose z extent >= min_z_extent_m.

    Cells with a large vertical spread contain vertical structure (quay wall,
    ship hull); flat water/ground cells are rejected. `points` is (N, 3+).
    Returns a boolean keep-mask of length N.
    """
    pts = np.asarray(points, dtype=float)
    n = pts.shape[0]
    if n == 0:
        return np.zeros(0, dtype=bool)
    cells = np.floor(pts[:, :2] / cell_size_m).astype(np.int64)
    # Map each 2D cell to a scalar key for grouping.
    keys = cells[:, 0] * 1_000_003 + cells[:, 1]
    order = np.argsort(keys, kind="stable")
    sorted_keys = keys[order]
    sorted_z = pts[order, 2]
    boundaries = np.flatnonzero(np.diff(sorted_keys)) + 1
    starts = np.concatenate(([0], boundaries))
    ends = np.concatenate((boundaries, [n]))
    mask = np.zeros(n, dtype=bool)
    for s, e in zip(starts, ends):
        z = sorted_z[s:e]
        if z.max() - z.min() >= min_z_extent_m:
            mask[order[s:e]] = True
    return mask


# ---------------------------------------------------------------------------
# Sequential 2D RANSAC line segments
# ---------------------------------------------------------------------------

@dataclass
class WallSegment:
    """A 2D wall line segment in the sensor (base_link) xy plane."""

    p0: np.ndarray  # (2,) endpoint [m]
    p1: np.ndarray  # (2,) endpoint [m]
    inlier_count: int
    inlier_indices: np.ndarray  # indices into the input array
    inlier_points: np.ndarray   # (M, 3) inlier xyz points (carried for output)

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))

    @property
    def midpoint(self) -> np.ndarray:
        return 0.5 * (self.p0 + self.p1)

    @property
    def direction(self) -> np.ndarray:
        d = self.p1 - self.p0
        n = np.linalg.norm(d)
        return d / n if n > 1e-12 else np.array([1.0, 0.0])

    @property
    def distance_from_origin(self) -> float:
        return _point_segment_distance(np.zeros(2), self.p0, self.p1)


def _point_segment_distance(q: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return float(np.linalg.norm(q - a))
    t = np.clip(float((q - a) @ ab) / denom, 0.0, 1.0)
    return float(np.linalg.norm(a + t * ab - q))


def _fit_line_ransac_once(
    xy: np.ndarray,
    distance_threshold_m: float,
    max_iterations: int,
    rng: np.random.Generator,
):
    """Single RANSAC 2D line fit. Returns (origin, unit_direction, inlier_mask) or None."""
    pts = np.asarray(xy, dtype=float)
    n = pts.shape[0]
    if n < 2:
        return None

    best_mask = None
    best_count = 0
    best_line = None
    for _ in range(max_iterations):
        idx = rng.choice(n, size=2, replace=False)
        a, b = pts[idx]
        d = b - a
        norm = np.linalg.norm(d)
        if norm < 1e-9:
            continue
        d = d / norm
        normal = np.array([-d[1], d[0]])
        dist = np.abs((pts - a) @ normal)
        mask = dist <= distance_threshold_m
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
            best_line = (a, d)
    if best_line is None:
        return None
    # Refine with PCA on the inliers.
    inliers = pts[best_mask]
    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    direction = vt[0] / np.linalg.norm(vt[0])
    normal = np.array([-direction[1], direction[0]])
    dist = np.abs((pts - centroid) @ normal)
    mask = dist <= distance_threshold_m
    return centroid, direction, mask


def fit_lines_ransac(
    points: np.ndarray,
    distance_threshold_m: float = 0.15,
    min_points: int = 30,
    max_lines: int = 4,
    min_length_m: float = 2.0,
    max_distance_m: float = 40.0,
    candidate_mask: np.ndarray | None = None,
    max_iterations: int = 100,
    rng: np.random.Generator | None = None,
) -> list[WallSegment]:
    """Sequentially extract up to max_lines 2D line segments.

    `points` is (N, 3+); fitting uses the xy projection. Endpoints come from
    the min/max projection of the inliers on the principal axis. Segments
    shorter than min_length_m or farther than max_distance_m from the origin
    are rejected. Each WallSegment carries (p0, p1, length, inlier_count,
    distance_from_origin, direction) plus the 3D inlier points.
    """
    pts = np.asarray(points, dtype=float)
    if rng is None:
        rng = np.random.default_rng(0)
    if candidate_mask is None:
        candidate_mask = np.ones(pts.shape[0], dtype=bool)

    remaining = np.flatnonzero(candidate_mask)
    segments: list[WallSegment] = []
    for _ in range(max_lines):
        if remaining.size < min_points:
            break
        result = _fit_line_ransac_once(
            pts[remaining, :2],
            distance_threshold_m=distance_threshold_m,
            max_iterations=max_iterations,
            rng=rng,
        )
        if result is None:
            break
        centroid, direction, mask = result
        inlier_local = np.flatnonzero(mask)
        if inlier_local.size < min_points:
            break
        inlier_global = remaining[inlier_local]
        proj = (pts[inlier_global, :2] - centroid) @ direction
        p0 = centroid + proj.min() * direction
        p1 = centroid + proj.max() * direction
        seg = WallSegment(
            p0=p0, p1=p1,
            inlier_count=int(inlier_global.size),
            inlier_indices=inlier_global,
            inlier_points=pts[inlier_global, :3].copy(),
        )
        # Remove inliers from the pool regardless of the gates so that a
        # rejected big structure does not block later segments.
        remaining = remaining[~mask]
        if seg.length < min_length_m:
            continue
        if seg.distance_from_origin > max_distance_m:
            continue
        segments.append(seg)
    return segments


# ---------------------------------------------------------------------------
# Temporal confirmation
# ---------------------------------------------------------------------------

@dataclass
class _WallTrack:
    segment: WallSegment
    hits: int
    last_seen: float
    confirmed: bool = False


@dataclass
class WallTracker:
    """Confirms wall segments seen repeatedly within the timeout window.

    A detection matches an existing track when the midpoints are within
    match_midpoint_dist_m and the directions agree within match_max_angle_rad.
    A track becomes confirmed after `confirmations` hits and is dropped when
    not seen for `timeout_sec`.
    """

    confirmations: int = 3
    timeout_sec: float = 2.0
    match_midpoint_dist_m: float = 3.0
    match_max_angle_rad: float = 0.35  # ~20 deg
    _tracks: list[_WallTrack] = field(default_factory=list)

    def update(self, segments: list[WallSegment], stamp_sec: float) -> list[WallSegment]:
        """Feed one frame of detections; returns currently confirmed segments."""
        unmatched = list(segments)
        for track in self._tracks:
            best = None
            best_dist = self.match_midpoint_dist_m
            for seg in unmatched:
                dist = float(np.linalg.norm(seg.midpoint - track.segment.midpoint))
                cos_angle = abs(float(seg.direction @ track.segment.direction))
                angle = float(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
                if dist <= best_dist and angle <= self.match_max_angle_rad:
                    best = seg
                    best_dist = dist
            if best is not None:
                unmatched.remove(best)
                track.segment = best
                track.hits += 1
                track.last_seen = stamp_sec
                if track.hits >= self.confirmations:
                    track.confirmed = True
        for seg in unmatched:
            self._tracks.append(_WallTrack(segment=seg, hits=1, last_seen=stamp_sec))
        # Drop stale tracks.
        self._tracks = [t for t in self._tracks
                        if stamp_sec - t.last_seen <= self.timeout_sec]
        return [t.segment for t in self._tracks if t.confirmed]


# ---------------------------------------------------------------------------
# Occupancy rasterization
# ---------------------------------------------------------------------------

def segments_to_occupancy(
    segments: list,
    margin_m: float,
    resolution_m: float,
    size_m: float,
) -> np.ndarray:
    """Rasterize segments into a square occupancy grid centered on the origin.

    `segments` is a list of (p0, p1) endpoint pairs (2-vectors) expressed in
    the grid frame, whose origin (0, 0) is the grid center. Cells whose center
    lies within margin_m of any segment are set to 100; all others stay 0.
    Returns an int8 (n, n) row-major array (row = y index, col = x index),
    matching nav_msgs/OccupancyGrid data layout with
    origin = (-size_m / 2, -size_m / 2).
    """
    n_cells = max(1, int(round(size_m / resolution_m)))
    coords = -size_m / 2.0 + (np.arange(n_cells) + 0.5) * resolution_m
    gx, gy = np.meshgrid(coords, coords)  # row-major: y is the row index
    cells = np.column_stack([gx.ravel(), gy.ravel()])

    occupied = np.zeros(cells.shape[0], dtype=bool)
    for p0, p1 in segments:
        a = np.asarray(p0, dtype=float)[:2]
        b = np.asarray(p1, dtype=float)[:2]
        ab = b - a
        denom = float(ab @ ab)
        if denom < 1e-12:
            dist = np.linalg.norm(cells - a, axis=1)
        else:
            s = np.clip((cells - a) @ ab / denom, 0.0, 1.0)
            closest = a + s[:, None] * ab
            dist = np.linalg.norm(cells - closest, axis=1)
        occupied |= dist <= margin_m

    grid = np.zeros(cells.shape[0], dtype=np.int8)
    grid[occupied] = 100
    return grid.reshape(n_cells, n_cells)
