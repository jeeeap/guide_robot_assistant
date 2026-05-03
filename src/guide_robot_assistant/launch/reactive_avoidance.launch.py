from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='guide_robot_assistant',
            executable='reactive_avoidance_node',
            name='reactive_avoidance_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'status_topic': '/avoidance_status',
                'front_angle_deg': 50.0,
                'safe_distance': 0.65,
                'critical_distance': 0.35,
                'forward_speed': 0.14,
                'slow_speed': 0.06,
                'turn_speed': 0.55,
                'control_rate': 10.0,
                'enabled': True,
            }],
        ),
    ])
