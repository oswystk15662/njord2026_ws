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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from tf2_ros import Buffer, TransformListener
import yaml
from pathlib import Path
from enum import Enum

class TaskType(Enum):
    """Enumeration for supported task types"""
    TASK1 = "task1"
    TASK2 = "task2"
    TASK3_1 = "task3_1"
    TASK3_2 = "task3_2"

class DockingState(Enum):
    """State machine for task3 multi-stage docking."""
    IDLE = 0
    STAGE1_APPROACH = 1
    STAGE1_WAIT = 2
    STAGE2_APPROACH = 3
    STAGE2_WAIT = 4
    FINAL_EXIT = 5
    COMPLETE = 6
    FAILED = 7

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
        self.declare_parameter('use_dynamic_gate_midpoints', True)
        self.declare_parameter('run_full_sequence', False)
        self.declare_parameter('max_goal_retries', 1)
        
        # Get parameters
        self.task_type_str = self.get_parameter('task_type').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.use_dynamic_gate_midpoints = self.get_parameter('use_dynamic_gate_midpoints').value
        self.run_full_sequence = self.get_parameter('run_full_sequence').value
        self.max_goal_retries = int(self.get_parameter('max_goal_retries').value)
        
        # Validate task type
        try:
            self.task_type = TaskType(self.task_type_str)
        except ValueError:
            self.get_logger().error(f"Unknown task type: {self.task_type_str}")
            raise
        
        # Load configuration
        self.config = self._load_config()

        # TF lookup is used to replace Task3 gate waypoints with the live midpoint
        # between the corresponding red and green buoys when those frames exist.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize action client for NavigateThroughPoses
        self.nav_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        
        # Task3 state machine
        self.docking_state = DockingState.IDLE
        self.dock_wait_timer = None
        self.goal_handle = None
        self.current_task3_stage_name = None
        self.current_task3_poses = []
        self.task3_goal_retry_count = 0
        self.warned_gate_fallback_ids = set()
        
        # Subscription to navigation feedback
        self.goal_reached_subscription = None
        
        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self.first_publish = True
        
        self.get_logger().info(
            f"WaypointPublisher initialized for task: {self.task_type.value}, "
            f"run_full_sequence={self.run_full_sequence}"
        )
        
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
        """Publish the docking approach stage for the selected Task3 mode."""
        stage1 = self._build_poses_for_ids(self._task3_stage_ids('stage_1'))
        if stage1:
            self.docking_state = DockingState.STAGE1_APPROACH
            self._send_navigate_through_poses_goal(stage1, stage_name='stage_1')
            if self.task_type == TaskType.TASK3_2:
                self.get_logger().info("Task3.2: approach sent [gps9 -> gate -> berth2_approach -> berth2]")
            else:
                self.get_logger().info("Task3.1: Stage 1 sent [gps7 -> gps8 -> berth1_approach -> berth1]")
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
            poses.append(self._pose_from_waypoint(wp))
        
        return poses

    def _build_poses_for_ids(self, waypoint_ids: list) -> list:
        """Build poses for the requested waypoint ids, preserving the id order."""
        by_id = {str(wp.get('id')): wp for wp in self.config.get('waypoints', [])}
        poses = []
        missing = []
        for waypoint_id in waypoint_ids:
            wp = by_id.get(str(waypoint_id))
            if wp is None:
                missing.append(str(waypoint_id))
                continue
            poses.append(self._pose_from_waypoint(wp))
        if missing:
            self.get_logger().warn(f"Task3: missing waypoint ids: {', '.join(missing)}")
            return []
        return poses

    def _task3_stage_ids(self, stage_name: str) -> list:
        stages = self.config.get('stages', {})
        if self.run_full_sequence:
            stages = self.config.get('full_sequence_stages', stages)
        return stages.get(stage_name, [])

    def _pose_from_waypoint(self, wp: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        x = float(wp.get('x', 0.0))
        y = float(wp.get('y', 0.0))
        midpoint = self._gate_midpoint_from_tf(wp)
        if midpoint is not None:
            x, y = midpoint

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        yaw = float(wp.get('yaw', 0.0))
        pose.pose.orientation = self._yaw_to_quaternion(yaw)
        return pose

    def _gate_midpoint_from_tf(self, wp: dict):
        if not self.use_dynamic_gate_midpoints or wp.get('type') != 'gate':
            return None
        gate_pair = wp.get('gate_pair')
        if not gate_pair:
            return None
        try:
            red_tf = self.tf_buffer.lookup_transform(
                self.frame_id, gate_pair['red_frame'], rclpy.time.Time())
            green_tf = self.tf_buffer.lookup_transform(
                self.frame_id, gate_pair['green_frame'], rclpy.time.Time())
        except Exception as exc:
            waypoint_id = str(wp.get('id'))
            if waypoint_id not in self.warned_gate_fallback_ids:
                self.warned_gate_fallback_ids.add(waypoint_id)
                self.get_logger().warn(
                    f"Task3: using configured gate waypoint for {waypoint_id}; "
                    f"gate TF unavailable: {exc}"
                )
            return None

        rx = red_tf.transform.translation.x
        ry = red_tf.transform.translation.y
        gx = green_tf.transform.translation.x
        gy = green_tf.transform.translation.y
        self.get_logger().debug(
            f"Task3: using live gate midpoint for {wp.get('id')} from "
            f"{gate_pair['red_frame']} and {gate_pair['green_frame']}"
        )
        return (0.5 * (rx + gx), 0.5 * (ry + gy))
    
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
    
    def _send_navigate_through_poses_goal(self, poses: list, stage_name: str = None):
        """
        Send NavigateThroughPoses action goal
        
        Args:
            poses: list of PoseStamped messages
        """
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateThroughPoses action server not ready, retrying...")
            return

        if self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2] and stage_name:
            if stage_name != self.current_task3_stage_name:
                self.task3_goal_retry_count = 0
            self.current_task3_stage_name = stage_name
            self.current_task3_poses = poses
        
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        
        self.get_logger().debug(f"Sending goal with {len(poses)} poses")
        
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """Callback for NavigateThroughPoses goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by NavigateThroughPoses action server")
            if self.task_type in [TaskType.TASK3_1, TaskType.TASK3_2]:
                self._retry_or_fail_current_task3_stage("goal rejected")
            else:
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
            self._retry_or_fail_current_task3_stage(f"goal status={status}")
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
            constraints = self.config.get('constraints', {})
            wait_time = float(
                constraints.get('second_wait_time_s', constraints.get('wait_time_s', 5))
            )
            self.get_logger().info(f"Task3: Berth 2 reached, waiting {wait_time:.0f}s...")
            self.dock_wait_timer = self.create_timer(wait_time, self._stage2_wait_complete)

        elif self.docking_state == DockingState.FINAL_EXIT:
            self.docking_state = DockingState.COMPLETE
            finish_name = "gps10" if self._task3_stage_ids('stage_3') else "gps8"
            self.get_logger().info(f"Task3: Complete! Reached {finish_name}.")

    def _retry_or_fail_current_task3_stage(self, reason: str):
        """Retry the active Task3 stage without falling back to stage 1."""
        if (
            not self.current_task3_stage_name or
            not self.current_task3_poses or
            self.docking_state in [DockingState.COMPLETE, DockingState.FAILED]
        ):
            self.docking_state = DockingState.FAILED
            self.get_logger().error(f"Task3: failed with no retryable current stage: {reason}")
            return

        if self.task3_goal_retry_count >= self.max_goal_retries:
            self.docking_state = DockingState.FAILED
            self.get_logger().error(
                f"Task3: {self.current_task3_stage_name} failed after "
                f"{self.task3_goal_retry_count} retries: {reason}"
            )
            return

        self.task3_goal_retry_count += 1
        self.get_logger().warn(
            f"Task3: retrying {self.current_task3_stage_name} "
            f"({self.task3_goal_retry_count}/{self.max_goal_retries}) after {reason}"
        )
        self._send_navigate_through_poses_goal(
            self.current_task3_poses,
            stage_name=self.current_task3_stage_name,
        )

    def _stage1_wait_complete(self):
        """Continue according to the selected Task3 flow."""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        if self.task_type == TaskType.TASK3_2:
            self.docking_state = DockingState.FINAL_EXIT
            self._publish_task3_third_stage()
        elif self.run_full_sequence:
            self.docking_state = DockingState.STAGE2_APPROACH
            self._publish_task3_second_stage()
        else:
            self.docking_state = DockingState.FINAL_EXIT
            self._publish_task3_second_stage()

    def _publish_task3_second_stage(self):
        """Stage 2: either return to GPS8 or continue to berth2."""
        stage2 = self._build_poses_for_ids(self._task3_stage_ids('stage_2'))
        if stage2:
            self._send_navigate_through_poses_goal(stage2, stage_name='stage_2')
            if self.run_full_sequence:
                self.get_logger().info(
                    "Task3: Stage 2 sent [berth1_exit -> gps9 -> berth2_approach -> berth2]"
                )
            else:
                self.get_logger().info("Task3.1: Stage 2 sent [berth1_exit -> gps8 - finish]")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 2")

    def _stage2_wait_complete(self):
        """After berth2 wait: exit to gps10"""
        if self.dock_wait_timer:
            self.destroy_timer(self.dock_wait_timer)
            self.dock_wait_timer = None
        self.docking_state = DockingState.FINAL_EXIT
        self._publish_task3_third_stage()

    def _publish_task3_third_stage(self):
        """Stage 3: gps10 (finish)"""
        stage3 = self._build_poses_for_ids(self._task3_stage_ids('stage_3'))
        if stage3:
            self._send_navigate_through_poses_goal(stage3, stage_name='stage_3')
            self.get_logger().info("Task3: Stage 3 sent [gps10 - finish]")
        else:
            self.get_logger().warn("Task3: Insufficient waypoints for stage 3")

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
