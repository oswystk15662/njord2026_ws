#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Waypoint Publisher Node
Publishes task-specific waypoint sequences using Nav2 NavigateThroughPoses action

Supports:
- Task 1: Maneuvering and Path Finding
- Task 2: Collision Avoidance
- Task 3: Docking (normal and parallel)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from builtin_interfaces.msg import Duration
import yaml
import os
from pathlib import Path
import asyncio
from enum import Enum

class TaskType(Enum):
    """Enumeration for supported task types"""
    TASK1 = "task1"
    TASK2 = "task2"
    TASK3_1 = "task3_1"
    TASK3_2 = "task3_2"

class DockingState(Enum):
    """State machine for task3 multi-stage docking"""
    IDLE = 0
    APPROACHING = 1
    DOCKING = 2
    DOCKED_WAIT = 3
    UNDOCKING = 4
    EXITING = 5
    COMPLETE = 6

class WaypointPublisher(Node):
    """
    Waypoint publisher node for NJORD competition tasks
    Publishes waypoints via Nav2's NavigateThroughPoses action
    """
    
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Declare parameters
        self.declare_parameter('task_type', 'task1')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 2.0)
        
        # Get parameters
        self.task_type_str = self.get_parameter('task_type').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        
        # Validate task type
        try:
            self.task_type = TaskType(self.task_type_str)
        except ValueError:
            self.get_logger().error(f"Unknown task type: {self.task_type_str}")
            raise
        
        # Load configuration
        self.config = self._load_config()
        
        # Initialize action client for NavigateThroughPoses
        self.nav_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        
        # Task3 state machine
        self.docking_state = DockingState.IDLE
        self.dock_wait_timer = None
        self.goal_handle = None
        
        # Subscription to navigation feedback
        self.goal_reached_subscription = None
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self.first_publish = True
        
        self.get_logger().info(f"WaypointPublisher initialized for task: {self.task_type.value}")
        
    def _load_config(self) -> dict:
        """
        Load waypoint configuration from YAML file
        Returns: dict with task configuration
        """
        # Construct config file path
        package_share_dir = Path(__file__).parent.parent / 'config'
        
        config_mapping = {
            TaskType.TASK1: 'task1_waypoints.yaml',
            TaskType.TASK2: 'task2_waypoints.yaml',
            TaskType.TASK3_1: 'task3_waypoints.yaml',
            TaskType.TASK3_2: 'task3_waypoints.yaml',
        }
        
        config_file = package_share_dir / config_mapping[self.task_type]
        
        if not config_file.exists():
            self.get_logger().error(f"Config file not found: {config_file}")
            raise FileNotFoundError(f"Config file not found: {config_file}")
        
        with open(config_file, 'r') as f:
            full_config = yaml.safe_load(f)
        
        # Extract relevant config based on task type
        if self.task_type == TaskType.TASK1:
            return full_config['task1_config']
        elif self.task_type == TaskType.TASK2:
            return full_config['task2_config']
        elif self.task_type == TaskType.TASK3_1:
            return full_config['task3_1_config']
        elif self.task_type == TaskType.TASK3_2:
            return full_config['task3_2_config']
    
    def _timer_callback(self):
        """Timer callback to publish waypoints"""
        if not self.first_publish:
            return
        
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateThroughPoses action server not ready, waiting...")
            return
        
        self.first_publish = False
        
        if self.task_type in [TaskType.TASK1, TaskType.TASK2]:
            self._publish_waypoints_single_stage()
        elif self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2]:
            self._publish_task3_first_stage()
    
    def _publish_waypoints_single_stage(self):
        """Publish all waypoints for task1 and task2"""
        waypoints = self._build_poses_from_config()
        self._send_navigate_through_poses_goal(waypoints)
        
        task_name = self.task_type.value
        self.get_logger().info(f"Published {len(waypoints)} waypoints for {task_name}")
    
    def _publish_task3_first_stage(self):
        """Publish first stage waypoints for task3 (approach + dock)"""
        # Get waypoints for approach and docking
        waypoints = self._build_poses_from_config()
        
        # For task3, publish all waypoints except the last one (exit) for stage 1
        # The undocking waypoint will be published after wait timer
        if len(waypoints) >= 2:
            stage1_waypoints = waypoints[:-1]
            self._send_navigate_through_poses_goal(stage1_waypoints)
            self.docking_state = DockingState.APPROACHING
            self.get_logger().info(f"Task3: Published stage 1 waypoints (approach + dock)")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 1")
    
    def _build_poses_from_config(self) -> list:
        """
        Build list of PoseStamped from configuration waypoints
        Returns: list of geometry_msgs.msg.PoseStamped
        """
        poses = []
        waypoints = self.config.get('waypoints', [])
        
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # Position
            pose.pose.position.x = float(wp.get('x', 0.0))
            pose.pose.position.y = float(wp.get('y', 0.0))
            pose.pose.position.z = 0.0
            
            # Orientation (yaw to quaternion)
            yaw = float(wp.get('yaw', 0.0))
            pose.pose.orientation = self._yaw_to_quaternion(yaw)
            
            poses.append(pose)
        
        return poses
    
    def _yaw_to_quaternion(self, yaw: float):
        """Convert yaw angle to quaternion"""
        from geometry_msgs.msg import Quaternion
        import math
        
        half_yaw = yaw / 2.0
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(half_yaw)
        quat.w = math.cos(half_yaw)
        
        return quat
    
    def _send_navigate_through_poses_goal(self, poses: list):
        """
        Send NavigateThroughPoses action goal
        
        Args:
            poses: list of PoseStamped messages
        """
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateThroughPoses action server not ready, retrying...")
            return
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        # Set goal tolerance
        tolerance = self.config.get('constraints', {}).get('goal_tolerance', 0.5)
        
        self.get_logger().debug(f"Sending goal with {len(poses)} poses")
        
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Callback for NavigateThroughPoses goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by NavigateThroughPoses action server")
            self.first_publish = True
            return
        
        self.get_logger().info("Goal accepted by NavigateThroughPoses action server")
        
        # For task3, wait for goal completion and then trigger stage 2
        if self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2]:
            self.goal_handle.get_result_async().add_done_callback(self._task3_goal_result_callback)
    
    def _task3_goal_result_callback(self, future):
        """Callback for task3 goal result"""
        result = future.result()
        
        if result:
            self.get_logger().info("Task3: Stage 1 waypoints (approach + dock) reached")
            self.docking_state = DockingState.DOCKED_WAIT
            
            # Start wait timer
            wait_time = self.config.get('constraints', {}).get('wait_time_s', 10)
            self.dock_wait_timer = self.create_timer(float(wait_time), self._task3_wait_complete)
            self.get_logger().info(f"Task3: Waiting {wait_time} seconds before undocking...")
        else:
            self.get_logger().error("Task3: Stage 1 goal failed")
    
    def _task3_wait_complete(self):
        """Called when dock wait timer completes"""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        
        self.docking_state = DockingState.UNDOCKING
        self._publish_task3_second_stage()
    
    def _publish_task3_second_stage(self):
        """Publish second stage waypoints for task3 (undocking + exit)"""
        waypoints = self._build_poses_from_config()
        
        # For task3, publish only last waypoint (exit)
        # The undocking waypoint is implicit - we just move to exit point
        if len(waypoints) > 0:
            # Send the last waypoint (goal/exit point)
            exit_waypoint = [waypoints[-1]]
            self._send_navigate_through_poses_goal(exit_waypoint)
            self.get_logger().info(f"Task3: Published stage 2 waypoint (exit)")
        else:
            self.get_logger().warn("Task3: No exit waypoint found")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
