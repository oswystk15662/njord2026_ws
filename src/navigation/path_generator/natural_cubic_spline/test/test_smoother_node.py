#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from builtin_interfaces.msg import Duration
from nav2_msgs.action import ComputePathToPose, SmoothPath
import tf2_ros
import numpy as np

class SmootherTestNode(Node):
    def __init__(self):
        super().__init__('smoother_test_node')
        
        # Publishers for visualizing results in RViz
        self.map_pub = self.create_publisher(
            OccupancyGrid, 
            '/map', 
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.raw_path_pub = self.create_publisher(Path, '/unsmoothed_path', 10)
        self.smooth_path_pub = self.create_publisher(Path, '/smoothed_path', 10)
        
        # Action Clients
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        
        # Map parameters
        self.width = 150
        self.height = 150
        self.resolution = 0.1
        self.origin_x = -7.5
        self.origin_y = -7.5
        
        # Timer to publish map periodically
        self.map_timer = self.create_timer(0.5, self.publish_map)
        
        # Map data initialization
        self.map_msg = self.create_test_map()
        
        # Start state machine / sequence
        self.create_timer(1.0, self.run_test_sequence)
        self.test_started = False

    def create_test_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        
        data = np.zeros(self.width * self.height, dtype=np.int8)
        
        # Create two obstacles (cost 100)
        # Obstacle 1: Center x = -2.0, y = 0.0, Size = 1.5m x 1.5m
        # Obstacle 2: Center x = 2.0, y = 0.0, Size = 1.5m x 1.5m
        for row in range(self.height):
            for col in range(self.width):
                x = self.origin_x + col * self.resolution
                y = self.origin_y + row * self.resolution
                
                # Check Obstacle 1
                if -2.75 <= x <= -1.25 and -0.75 <= y <= 0.75:
                    data[row * self.width + col] = 100
                # Check Obstacle 2
                elif 1.25 <= x <= 2.75 and -0.75 <= y <= 0.75:
                    data[row * self.width + col] = 100
                    
        grid.data = data.tolist()
        return grid

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def run_test_sequence(self):
        if self.test_started:
            return
        self.test_started = True
        
        self.get_logger().info('Waiting for action servers...')
        if not self.planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Planner action server not available!')
            self.test_started = False
            return
            
        if not self.smoother_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Smoother action server not available!')
            self.test_started = False
            return
            
        self.get_logger().info('Servers ready! Sending path planning request...')
        
        # Create start and goal poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = -5.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.orientation.w = 1.0
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 5.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = True
        
        self.planner_client.send_goal_async(goal_msg).add_done_callback(self.planner_response_callback)

    def planner_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planner rejected the goal request.')
            self.test_started = False
            return
        
        self.get_logger().info('Planner accepted goal. Waiting for path...')
        goal_handle.get_result_async().add_done_callback(self.planner_result_callback)

    def planner_result_callback(self, future):
        result = future.result().result
        path = result.path
        
        self.get_logger().info(f'Planner successfully generated path with {len(path.poses)} poses!')
        
        # Publish raw path to topic
        self.raw_path_pub.publish(path)
        self.raw_path = path  # Keep reference for continuous publishing if needed
        
        # Send path to smoother
        self.get_logger().info('Sending path to smoother action server...')
        smooth_goal = SmoothPath.Goal()
        smooth_goal.path = path
        smooth_goal.smoother_id = 'natural_cubic_spline'
        smooth_goal.max_smoothing_duration = Duration(sec=2, nanosec=0)
        smooth_goal.check_for_collisions = False
        
        self.smoother_client.send_goal_async(smooth_goal).add_done_callback(self.smoother_response_callback)

    def smoother_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Smoother rejected the goal request.')
            self.test_started = False
            return
            
        self.get_logger().info('Smoother accepted goal. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.smoother_result_callback)

    def smoother_result_callback(self, future):
        result = future.result().result
        smoothed_path = result.smoothed_path
        
        self.get_logger().info(f'Smoother successfully smoothed path to {len(smoothed_path.poses)} poses!')
        
        # Publish smoothed path to topic
        self.smooth_path_pub.publish(smoothed_path)
        self.smoothed_path = smoothed_path  # Keep reference
        
        # Start a loop to republish paths so they persist in RViz
        self.republish_timer = self.create_timer(1.0, self.republish_paths)

    def republish_paths(self):
        if hasattr(self, 'raw_path'):
            self.raw_path.header.stamp = self.get_clock().now().to_msg()
            self.raw_path_pub.publish(self.raw_path)
        if hasattr(self, 'smoothed_path'):
            self.smoothed_path.header.stamp = self.get_clock().now().to_msg()
            self.smooth_path_pub.publish(self.smoothed_path)

def main(args=None):
    rclpy.init(args=args)
    node = SmootherTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
