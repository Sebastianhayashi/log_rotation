from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 定义启动参数
    log_file_path_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value=os.path.expanduser('~/.ros/log/log_rotation_node.log'),
        description='Path to the log file for the node'
    )

    max_size_arg = DeclareLaunchArgument(
        'max_size',
        default_value='1048576',  # 默认 1MB
        description='Maximum log file size before rotation in bytes'
    )

    max_files_arg = DeclareLaunchArgument(
        'max_files',
        default_value='5',
        description='Maximum number of backup log files'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (e.g., debug, info, warn, error, fatal)'
    )

    # 创建并返回 LaunchDescription
    return LaunchDescription([
        log_file_path_arg,
        max_size_arg,
        max_files_arg,
        log_level_arg,
        Node(
            package='log_rotation',  # 替换为您的包名称
            executable='log_test_node',  # 替换为您实际的可执行文件名称
            name='log_test_node',
            output='screen',
            parameters=[{
                'log_file_path': LaunchConfiguration('log_file_path'),
                'max_size': LaunchConfiguration('max_size'),
                'max_files': LaunchConfiguration('max_files')
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        )
    ])
