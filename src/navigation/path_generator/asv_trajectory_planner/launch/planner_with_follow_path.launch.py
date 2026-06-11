from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planner_node = Node(
        package="asv_trajectory_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {
                "own_odom_topic": "/own_ship/odom",
                "other_ship_odom_topic": "/other_ship/odom",
                "goal_pose_topic": "/goal_pose",
                "path_topic": "/planned_path",
                "frame_id": "map",
                "planning_frequency": 2.0,
                "point_spacing": 0.5,
                "avoid_radius": 2.0,
                "avoid_offset": 3.0,
                "require_other_ship": True,
            }
        ],
    )

    follow_path_client_node = Node(
        package="asv_trajectory_planner",
        executable="follow_path_client_node",
        name="follow_path_client_node",
        output="screen",
        parameters=[
            {
                "path_topic": "/planned_path",
                "action_name": "follow_path",
                "controller_id": "FollowPath",
                "goal_checker_id": "general_goal_checker",
                "progress_checker_id": "progress_checker",
                "send_frequency": 1.0,
                "enable_replanning": True,
            }
        ],
    )

    return LaunchDescription(
        [
            planner_node,
            follow_path_client_node,
        ]
    )