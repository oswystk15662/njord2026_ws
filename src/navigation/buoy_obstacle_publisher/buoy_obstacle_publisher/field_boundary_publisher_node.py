#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
field_boundary_publisher_node.py

Publishes a static nav_msgs/OccupancyGrid that marks the outside of the
competition field as LETHAL (cost=100).

For Task 3 the field is a 40 m x 40 m square centered at (0, 0) in the map frame.
Everything outside is marked 100; everything inside is 0.

The map is published LATCHED (transient_local QoS) so Nav2 StaticLayer
receives it even if it subscribes late.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import math


class FieldBoundaryPublisher(Node):
    def __init__(self):
        super().__init__('field_boundary_publisher')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.2)          # coarser for large static map
        self.declare_parameter('map_size_m', 80.0)         # total map side
        self.declare_parameter('field_size_m', 40.0)       # interior free zone
        self.declare_parameter('field_center_x', 0.0)
        self.declare_parameter('field_center_y', 0.0)
        self.declare_parameter('boundary_cost', 100)

        frame = self.get_parameter('map_frame').value
        res = self.get_parameter('resolution').value
        map_size = self.get_parameter('map_size_m').value
        field_size = self.get_parameter('field_size_m').value
        cx = self.get_parameter('field_center_x').value
        cy = self.get_parameter('field_center_y').value
        boundary_cost = int(self.get_parameter('boundary_cost').value)

        half_field = field_size / 2.0
        n = int(math.ceil(map_size / res))
        if n % 2 == 0:
            n += 1
        origin_x = cx - (n // 2) * res
        origin_y = cy - (n // 2) * res

        data = []
        for row in range(n):
            wy = origin_y + row * res
            for col in range(n):
                wx = origin_x + col * res
                in_field = (abs(wx - cx) <= half_field and abs(wy - cy) <= half_field)
                data.append(0 if in_field else boundary_cost)

        # Build OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.info.resolution = res
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data

        # Latched publisher (transient_local)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/field_boundary_costmap', qos)
        self.pub.publish(msg)

        self.get_logger().info(
            f'FieldBoundaryPublisher: published {n}x{n} map | '
            f'field={field_size}m x {field_size}m | res={res}m | '
            f'origin=({origin_x:.1f},{origin_y:.1f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FieldBoundaryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
