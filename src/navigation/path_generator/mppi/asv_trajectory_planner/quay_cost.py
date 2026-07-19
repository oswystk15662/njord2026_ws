"""EXPERIMENTAL quay wall distance penalty for the Task2 MPPI planner.

Nav2 costmap (fed by /quay_wall/costmap from task2_perception) is the PRIMARY
quay safety path. This module adds an optional, additional soft penalty inside
the MPPI rollout cost so sampled trajectories are biased away from detected
quay wall segments. It is only active when the planner_node parameter
``enable_mppi_quay_cost`` is true; otherwise this module is never invoked and
the MPPI behavior is identical to the historical implementation.

Inputs are quay wall line segments (from /quay_wall/markers, a
visualization_msgs/MarkerArray of LINE_LIST markers) expressed in the SAME
frame as the MPPI prediction points (base_link-centric planning frame).
"""

import torch


def quay_segment_cost(X, Y, segments, margin_m, weight):
    """Penalty for predicted points that come within ``margin_m`` of any quay
    wall segment.

    Parameters
    ----------
    X, Y : torch.Tensor
        MPPI predicted positions, shape [num_samples, num_timesteps].
    segments : sequence | torch.Tensor
        Quay wall segments as (x0, y0, x1, y1) rows, shape [M, 4], in the same
        frame as X/Y.
    margin_m : float
        Safety margin. Points farther than this from every segment incur zero
        cost.
    weight : float
        Cost weight multiplying the squared margin violation.

    Returns
    -------
    torch.Tensor
        Per-point cost map with the same shape as X (caller reduces over the
        time axis, mirroring buoy_outside_cost usage in mppi_torch.get_opt).
    """
    seg = torch.as_tensor(segments, device=X.device, dtype=X.dtype).reshape(-1, 4)

    total = torch.zeros_like(X)

    if seg.shape[0] == 0 or weight == 0.0 or margin_m <= 0.0:
        return total

    for i in range(seg.shape[0]):
        x0 = seg[i, 0]
        y0 = seg[i, 1]
        x1 = seg[i, 2]
        y1 = seg[i, 3]

        dx = x1 - x0
        dy = y1 - y0
        len2 = (dx * dx + dy * dy).clamp_min(1.0e-9)

        # Projection parameter of each predicted point onto the segment.
        t = ((X - x0) * dx + (Y - y0) * dy) / len2
        t = torch.clamp(t, 0.0, 1.0)

        closest_x = x0 + t * dx
        closest_y = y0 + t * dy

        dist = torch.sqrt(
            (X - closest_x) ** 2 + (Y - closest_y) ** 2
        )

        # Quadratic penalty inside the safety margin, zero outside.
        total = total + torch.relu(margin_m - dist) ** 2

    return weight * total
