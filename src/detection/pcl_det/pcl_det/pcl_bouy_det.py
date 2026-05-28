#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCL-based Buoy Detection Node.

Subscribes to point cloud data, detects buoys, and publishes:
- TF frames for each detected buoy relative to base_link
- Point2D (PointStamped) output with detected buoy positions
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, point_cloud2
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.distance import cdist
from scipy.cluster.hierarchy import fclusterdata


class PCLBuoyDetectionNode(Node):
    """Node for detecting buoys from point cloud data."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('pcl_bouy_det_node')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/buoy_detections')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('height_min', -1.0)  # Minimum height filter (m)
        self.declare_parameter('height_max', 0.5)   # Maximum height filter (m)
        self.declare_parameter('distance_threshold', 0.05)  # Clustering threshold (m)
        self.declare_parameter('min_points_per_cluster', 5)  # Minimum points in a cluster

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.min_points_per_cluster = self.get_parameter('min_points_per_cluster').value

        # Create subscriptions and publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PointStamped,
            self.output_topic,
            10
        )

        # TF broadcaster for buoy positions
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(
            f'PCL Buoy Detection Node initialized.\n'
            f'  Input topic: {self.input_topic}\n'
            f'  Output topic: {self.output_topic}\n'
            f'  Frame ID: {self.frame_id}'
        )

    def pointcloud_callback(self, msg: PointCloud2):
        """
        Process incoming point cloud and detect buoys.
        
        Args:
            msg: PointCloud2 message
        """
        try:
            # Convert PointCloud2 to numpy array
            points = np.array(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'))))
            
            if len(points) == 0:
                return
            
            # Filter points by height (z-coordinate)
            mask = (points[:, 2] >= self.height_min) & (points[:, 2] <= self.height_max)
            filtered_points = points[mask]
            
            if len(filtered_points) < self.min_points_per_cluster:
                return
            
            # Cluster the points using hierarchical clustering
            buoy_clusters = fclusterdata(
                filtered_points,
                t=self.distance_threshold,
                criterion='distance',
                method='complete'
            )
            
            # Process each cluster
            buoy_count = 0
            for cluster_id in np.unique(buoy_clusters):
                cluster_points = filtered_points[buoy_clusters == cluster_id]
                
                # Skip small clusters
                if len(cluster_points) < self.min_points_per_cluster:
                    continue
                
                # Calculate cluster centroid
                centroid = np.mean(cluster_points, axis=0)
                
                # Publish as Point2D (PointStamped with z=0)
                detection = PointStamped()
                detection.header.stamp = msg.header.stamp
                detection.header.frame_id = msg.header.frame_id
                detection.point.x = float(centroid[0])
                detection.point.y = float(centroid[1])
                detection.point.z = 0.0  # Point2D representation
                
                self.publisher.publish(detection)
                
                # Publish TF for this buoy
                tf_frame_id = f'buoy_{cluster_id}'
                transform = TransformStamped()
                transform.header.stamp = msg.header.stamp
                transform.header.frame_id = msg.header.frame_id
                transform.child_frame_id = tf_frame_id
                transform.transform.translation.x = float(centroid[0])
                transform.transform.translation.y = float(centroid[1])
                transform.transform.translation.z = float(centroid[2])
                transform.transform.rotation.w = 1.0
                
                self.tf_broadcaster.sendTransform(transform)
                
                buoy_count += 1
            
            if buoy_count > 0:
                self.get_logger().debug(f'Detected {buoy_count} buoys')
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')


def main(args=None):
    """Main function to run the node."""
    rclpy.init(args=args)
    node = PCLBuoyDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
