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
from rclpy.qos import DurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from pathlib import Path
from enum import Enum
from ament_index_python.packages import get_package_share_directory

class TaskType(Enum):
    """Enumeration for supported task types"""
    TASK1 = "task1"
    TASK2 = "task2"
    TASK3_1 = "task3_1"
    TASK3_2 = "task3_2"

class DockingState(Enum):
    """State machine for task3 multi-stage docking (two berths)"""
    IDLE = 0
    STAGE1_APPROACH = 1   # 7 -> 8 -> berth1
    STAGE1_WAIT = 2       # stationary at berth1
    STAGE2_APPROACH = 3   # 8_exit -> 9 -> berth2
    STAGE2_WAIT = 4       # stationary at berth2
    STAGE3_EXIT = 5       # 10
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
        self.frame_id = self.frame_id or self.config.get('frame_id', 'map')
        
        # Initialize action client for NavigateThroughPoses
        self.nav_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = self.create_publisher(PathMsg, '/plan', path_qos)
        self.marker_publisher = self.create_publisher(MarkerArray, '/waypoint_markers', path_qos)
        
        # Task3 state machine
        self.docking_state = DockingState.IDLE
        self.dock_wait_timer = None
        self.goal_handle = None
        
        # Subscription to navigation feedback
        self.goal_reached_subscription = None
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self.first_publish = True
        self.plan_published = False
        
        self.get_logger().info(f"WaypointPublisher initialized for task: {self.task_type.value}")
        
    def _load_config(self) -> dict:
        """
        Load waypoint configuration from YAML file
        Returns: dict with task configuration
        """
        package_share_dir = Path(get_package_share_directory('waypoint_publisher')) / 'config'
        
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
        if not self.plan_published:
            self._publish_plan(self._build_poses_from_config())
            self.plan_published = True

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
        self._publish_plan(waypoints)
        self._send_navigate_through_poses_goal(waypoints)
        
        task_name = self.task_type.value
        self.get_logger().info(f"Published {len(waypoints)} waypoints for {task_name}")
    
    def _publish_task3_first_stage(self):
        """Publish the docking approach stage for the selected Task3 mode."""
        waypoints = self._build_poses_from_config()
        if len(waypoints) >= 3:
            stage1 = waypoints[:3]
            self._publish_plan(stage1)
            self._send_navigate_through_poses_goal(stage1)
            self.docking_state = DockingState.STAGE1_APPROACH
            if self.task_type == TaskType.TASK3_2:
                self.get_logger().info("Task3.2: approach sent [gps9 -> gate -> berth2]")
            else:
                self.get_logger().info("Task3.1: Stage 1 sent [gps7 -> gps8 -> berth1]")
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
        """Dispatch to correct handler based on current docking state"""
        wrapped_result = future.result()
        if wrapped_result is None or wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if wrapped_result is None else wrapped_result.status
            self.get_logger().error(
                f"Task3: Goal failed at state {self.docking_state}, status={status}"
            )
            return

        if self.docking_state == DockingState.STAGE1_APPROACH:
            self.docking_state = DockingState.STAGE1_WAIT
            wait_time = float(self.config.get('constraints', {}).get('wait_time_s', 10))
            berth_name = "Berth 2" if self.task_type == TaskType.TASK3_2 else "Berth 1"
            self.get_logger().info(f"Task3: {berth_name} reached, waiting {wait_time:.0f}s...")
            self.dock_wait_timer = self.create_timer(wait_time, self._stage1_wait_complete)

        elif self.docking_state == DockingState.STAGE2_APPROACH:
            # Arrived at berth2 -> start wait
            self.docking_state = DockingState.STAGE2_WAIT
            wait_time = float(self.config.get('constraints', {}).get('wait_time_s', 10))
            self.get_logger().info(f"Task3: Berth 2 reached, waiting {wait_time:.0f}s...")
            self.dock_wait_timer = self.create_timer(wait_time, self._stage2_wait_complete)

        elif self.docking_state == DockingState.STAGE3_EXIT:
            self.docking_state = DockingState.COMPLETE
            self.get_logger().info("Task3: Complete! Reached gps10.")

    def _stage1_wait_complete(self):
        """Continue to the second berth for Task3.1, or exit for Task3.2."""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        if self.task_type == TaskType.TASK3_2:
            self.docking_state = DockingState.STAGE3_EXIT
            self._publish_task3_third_stage()
        else:
            self.docking_state = DockingState.STAGE2_APPROACH
            self._publish_task3_second_stage()

    def _publish_task3_second_stage(self):
        """Stage 2: gps8_exit -> gps9 -> berth2"""
        waypoints = self._build_poses_from_config()
        # indices 3=8_exit, 4=gps9, 5=berth2
        if len(waypoints) >= 6:
            stage2 = waypoints[3:6]  # 8_exit, gps9, berth2
            self._publish_plan(stage2)
            self._send_navigate_through_poses_goal(stage2)
            self.get_logger().info("Task3: Stage 2 sent [gps8_exit -> gps9 -> berth2]")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 2")

    def _stage2_wait_complete(self):
        """After berth2 wait: exit to gps10"""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        self.docking_state = DockingState.STAGE3_EXIT
        self._publish_task3_third_stage()

    def _publish_task3_third_stage(self):
        """Stage 3: gps10 (finish)"""
        waypoints = self._build_poses_from_config()
        finish_index = 3 if self.task_type == TaskType.TASK3_2 else 6
        if len(waypoints) > finish_index:
            stage3 = [waypoints[finish_index]]
            self._publish_plan(stage3)
            self._send_navigate_through_poses_goal(stage3)
            self.get_logger().info("Task3: Stage 3 sent [gps10 - finish]")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 3")

    def _publish_plan(self, poses: list):
        """Publish the intended route for validators and GUI overlays."""
        if not poses:
            return

        stamp = self.get_clock().now().to_msg()
        path = PathMsg()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp

        for pose in poses:
            pose.header.stamp = stamp
            path.poses.append(pose)

        self.path_publisher.publish(path)
        self._publish_waypoint_markers(poses, stamp)
        self.get_logger().info(f"Published /plan with {len(path.poses)} poses")

    def _publish_waypoint_markers(self, poses: list, stamp):
        """Publish RViz markers that match the currently published path."""
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        waypoint_defs = self.config.get('waypoints', [])
        for idx, pose in enumerate(poses):
            wp = waypoint_defs[idx] if idx < len(waypoint_defs) else {}
            marker_array.markers.append(self._make_waypoint_marker(idx, pose, wp, stamp))
            marker_array.markers.append(self._make_waypoint_label_marker(idx, pose, wp, stamp))

        self.marker_publisher.publish(marker_array)

    def _waypoint_color(self, wp_type: str) -> ColorRGBA:
        color = ColorRGBA()
        color.a = 0.95
        if wp_type == 'start':
            color.r, color.g, color.b = 0.1, 0.9, 0.2
        elif wp_type == 'goal':
            color.r, color.g, color.b = 1.0, 0.1, 0.1
        elif wp_type == 'gps':
            color.r, color.g, color.b = 0.1, 0.35, 1.0
        else:
            color.r, color.g, color.b = 1.0, 0.85, 0.05
        return color

    def _make_waypoint_marker(self, idx: int, pose: PoseStamped, wp: dict, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'waypoints'
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color = self._waypoint_color(str(wp.get('type', 'intermediate')))
        return marker

    def _make_waypoint_label_marker(self, idx: int, pose: PoseStamped, wp: dict, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'waypoint_labels'
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = 1.1
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.8
        marker.color = self._waypoint_color(str(wp.get('type', 'intermediate')))
        marker.text = str(wp.get('name', f'WP {idx + 1}'))
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
