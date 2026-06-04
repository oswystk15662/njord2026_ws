#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
buoy_obstacle_publisher_node.py

Reads buoy TF frames (any child frame matching a configurable prefix list,
e.g. 'red_buoy_*', 'green_buoy_*') and publishes a nav_msgs/OccupancyGrid
that marks lethal cost around each buoy.

This OccupancyGrid can be consumed by Nav2's StaticLayer via
  map_topic: /buoy_costmap
in either local or global costmap config.

Design rationale:
  - Works with *both* real detection (buoy TFs from pcl_det / yolo pipeline)
    and simulation (buoy TFs from task3_orchestrator).
  - Grid origin shifts every cycle to cover a region around the robot
    so it can be used as a rolling costmap source.
  - Buoy inflation radius is configurable (default 1.0 m for 0.45m-radius buoy).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class BuoyObstaclePublisher(Node):
    def __init__(self):
        super().__init__('buoy_obstacle_publisher')

        # --- Parameters ---
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.1)          # m/cell
        self.declare_parameter('grid_size_m', 60.0)        # total grid side length [m]
        self.declare_parameter('buoy_radius_m', 0.45)      # physical buoy radius
        self.declare_parameter('inflation_radius_m', 1.2)  # extra inflation around buoy
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('buoy_prefixes', ['red_buoy_', 'green_buoy_'])
        self.declare_parameter('max_buoys', 32)            # safety limit for TF scan

        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = self.get_parameter('resolution').value
        self.grid_size_m = self.get_parameter('grid_size_m').value
        self.buoy_radius = self.get_parameter('buoy_radius_m').value
        self.inflation_radius = self.get_parameter('inflation_radius_m').value
        self.buoy_prefixes = self.get_parameter('buoy_prefixes').value
        self.max_buoys = self.get_parameter('max_buoys').value

        self.grid_cells = int(math.ceil(self.grid_size_m / self.resolution))
        # Make odd so origin can be exactly centered
        if self.grid_cells % 2 == 0:
            self.grid_cells += 1
        self.half_cells = self.grid_cells // 2

        # Lethal cost cell radius in cells (physical + inflation)
        obstacle_r_m = self.buoy_radius + self.inflation_radius
        self.obstacle_r_cells = int(math.ceil(obstacle_r_m / self.resolution))
        # Build a 2-D boolean stamp for efficiency
        r = self.obstacle_r_cells
        self._stamp = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self._stamp.append((dx, dy))

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(OccupancyGrid, '/buoy_costmap', 10)

        rate = max(0.1, self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'BuoyObstaclePublisher started | frame={self.map_frame} '
            f'res={self.resolution}m grid={self.grid_cells}x{self.grid_cells} '
            f'obs_r={obstacle_r_m:.2f}m'
        )

    # ------------------------------------------------------------------
    def _on_timer(self):
        buoy_positions = self._collect_buoy_positions()
        msg = self._build_grid(buoy_positions)
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    def _collect_buoy_positions(self):
        """Return list of (x, y) in map frame for all matching buoy TF frames."""
        positions = []
        try:
            frame_ids = self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return positions

        import yaml as _yaml
        try:
            frames_dict = _yaml.safe_load(frame_ids) or {}
        except Exception:
            return positions

        for frame_id in frames_dict.keys():
            if not any(frame_id.startswith(p) for p in self.buoy_prefixes):
                continue
            try:
                t: TransformStamped = self.tf_buffer.lookup_transform(
                    self.map_frame, frame_id, rclpy.time.Time()
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                positions.append((x, y))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
            if len(positions) >= self.max_buoys:
                break

        return positions

    # ------------------------------------------------------------------
    def _build_grid(self, buoy_positions):
        """Build an OccupancyGrid centered at map origin (fixed frame)."""
        n = self.grid_cells
        origin_x = -self.half_cells * self.resolution
        origin_y = -self.half_cells * self.resolution

        # Start with unknown (-1) → free (0)
        data = [0] * (n * n)

        for (bx, by) in buoy_positions:
            # Convert buoy world coord to grid cell
            cx = int(round((bx - origin_x) / self.resolution))
            cy = int(round((by - origin_y) / self.resolution))

            for dx, dy in self._stamp:
                gx = cx + dx
                gy = cy + dy
                if 0 <= gx < n and 0 <= gy < n:
                    idx = gy * n + gx
                    data[idx] = 100  # LETHAL

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BuoyObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
