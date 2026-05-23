from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    locations_file = PathJoinSubstitution([
        FindPackageShare('guide_robot_assistant'),
        'config',
        'task_locations.yaml',
    ])

    use_llm_arg = DeclareLaunchArgument('use_llm', default_value='false')
    use_mic_arg = DeclareLaunchArgument('use_microphone', default_value='false')
    use_espeak_arg = DeclareLaunchArgument('use_espeak', default_value='true')

    return LaunchDescription([
        use_llm_arg,
        use_mic_arg,
        use_espeak_arg,

        # ── 语义解析核心（Agentic版，含对话记忆）──────────────────────────
        Node(
            package='guide_robot_assistant',
            executable='center_node',
            name='center_node',
            output='screen',
            parameters=[{
                'use_llm': LaunchConfiguration('use_llm'),
                'llm_model': 'glm-4-flash',
                'llm_api_key_env': 'ZHIPUAI_API_KEY',
                'llm_timeout': 8.0,
                'enable_memory': True,
            }],
        ),

        # ── 导航客户端 ─────────────────────────────────────────────────────
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

        # ── TTS语音合成（espeak-ng）────────────────────────────────────────
        Node(
            package='guide_robot_assistant',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'enable_console_output': True,
                'enable_espeak': LaunchConfiguration('use_espeak'),
                'espeak_voice': 'zh',
                'espeak_speed': 145,
                'espeak_amplitude': 100,
            }],
        ),

        # ── ASR语音识别（use_microphone=true 开启麦克风，否则文本键盘兜底）──
        Node(
            package='guide_robot_assistant',
            executable='asr_node',
            name='asr_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_microphone': LaunchConfiguration('use_microphone'),
                'language': 'zh-CN',
                'recognition_timeout_sec': 5.0,
                'phrase_time_limit_sec': 8.0,
                'max_retries': 2,
            }],
        ),

        # ── 实验日志记录 ───────────────────────────────────────────────────
        Node(
            package='guide_robot_assistant',
            executable='experiment_logger_node',
            name='experiment_logger_node',
            output='screen',
            parameters=[{
                'scenario_name': 'text_navigation_demo',
            }],
        ),
    ])
