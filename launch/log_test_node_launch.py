from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 定义日志文件路径的启动参数
    log_file_path_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value=os.path.expanduser('~/.ros/log/log_rotation_node.log'),
        description='Path to the log file for the node'
    )

    # 定义日志文件大小的启动参数
    max_size_arg = DeclareLaunchArgument(
        'max_size',
        default_value='1048576',  # 1MB
        description='Maximum log file size before rotation in bytes'
    )

    # 定义最大备份文件数量的启动参数
    max_files_arg = DeclareLaunchArgument(
        'max_files',
        default_value='5',
        description='Maximum number of backup log files'
    )

    # 定义日志级别的启动参数
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
            package='log_rotation',
            executable='log_test_node',
            name='log_test_node',
            output='screen',
            parameters=[{
                'log_file_path': LaunchConfiguration('log_file_path'),
                'max_size': LaunchConfiguration('max_size'),
                'max_files': LaunchConfiguration('max_files')
            }],
            # 配置日志级别
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        )
    ])
