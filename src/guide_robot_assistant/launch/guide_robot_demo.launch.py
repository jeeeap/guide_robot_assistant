from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    locations_file = PathJoinSubstitution([
        FindPackageShare('guide_robot_assistant'),
        'config',
        'task_locations.yaml',
    ])

    return LaunchDescription([
        Node(
            package='guide_robot_assistant',
            executable='center_node',
            name='center_node',
            output='screen',
            parameters=[{
                'use_llm': False,
                'llm_model': 'glm-4-flash',
                'llm_api_key_env': 'ZHIPUAI_API_KEY',
                'llm_timeout': 8.0,
            }],
        ),
        Node(
            package='guide_robot_assistant',
            executable='nav_client_node',
            name='nav_client_node',
            output='screen',
            parameters=[{
                'locations_file': locations_file,
                'goal_frame_id': 'map',
                'action_name': 'navigate_to_pose',
                'nav2_wait_timeout_sec': 15.0,
            }],
        ),
        Node(
            package='guide_robot_assistant',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'enable_console_output': True,
            }],
        ),
        Node(
            package='guide_robot_assistant',
            executable='experiment_logger_node',
            name='experiment_logger_node',
            output='screen',
            parameters=[{
                'scenario_name': 'text_navigation_demo',
            }],
        ),
        Node(
            package='guide_robot_assistant',
            executable='text_command_node',
            name='text_command_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
