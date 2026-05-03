from glob import glob
from setuptools import find_packages, setup

package_name = 'guide_robot_assistant'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='jp',
    maintainer_email='jp3274313334@163.com',
    description='ROS2 indoor guide robot assistant for text command navigation demos.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'text_command_node = guide_robot_assistant.text_command_node:main',
            'center_node = guide_robot_assistant.center_node:main',
            'nav_client_node = guide_robot_assistant.nav_client_node:main',
            'tts_node = guide_robot_assistant.tts_node:main',
            'experiment_logger_node = guide_robot_assistant.experiment_logger_node:main',
            'planner_compare_node = guide_robot_assistant.planner_compare_node:main',
            'reactive_avoidance_node = guide_robot_assistant.reactive_avoidance_node:main',
        ],
    },
)
