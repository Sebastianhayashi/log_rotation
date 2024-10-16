from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('my_ros2_logging_project'),
        'config', 'log4cxx.properties')

    return LaunchDescription([
        Node(
            package='my_ros2_logging_project',
            executable='log_test_node',
            name='log_test_node',
            output='log',
            parameters=[{'use_sim_time': False}],
            remappings=[],
            arguments=['--ros-args', '--log-config', config_dir],
        )
    ])
