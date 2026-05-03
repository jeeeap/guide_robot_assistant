from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='guide_robot_assistant',
            executable='planner_compare_node',
            name='planner_compare_node',
            output='screen',
            parameters=[{
                'occupied_threshold': 50,
                'unknown_is_obstacle': True,
                'allow_diagonal': True,
                'auto_plan_on_map': False,
                'default_start_x': 0.0,
                'default_start_y': 0.0,
                'default_goal_x': 2.0,
                'default_goal_y': 1.5,
            }],
        ),
    ])
