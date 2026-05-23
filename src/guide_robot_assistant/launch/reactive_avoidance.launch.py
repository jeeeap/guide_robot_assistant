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
                'side_angle_deg': 90.0,
                # 4级预警距离（盲人场景，保守设置）
                'warning_distance': 1.2,
                'safe_distance': 0.65,
                'critical_distance': 0.35,
                'side_warning_distance': 0.45,
                # 速度
                'forward_speed': 0.14,
                'warning_speed': 0.09,
                'slow_speed': 0.05,
                'turn_speed': 0.55,
                'control_rate': 10.0,
                # TTS防重播间隔（秒）
                'tts_cooldown_sec': 3.0,
                'enabled': True,
            }],
        ),
    ])
