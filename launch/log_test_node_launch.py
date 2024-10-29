from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 定义日志文件路径和日志级别的启动参数
    log_file_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value=os.path.expanduser('~/.ros/log/log_rotation_node.log'),
        description='Path to the log file for the node'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (e.g., debug, info, warn, error, fatal)'
    )

    # 创建并返回 LaunchDescription
    return LaunchDescription([
        log_file_arg,
        log_level_arg,
        Node(
            package='log_rotation',
            executable='log_test_node',
            name='log_test_node',
            output='log',
            parameters=[{
                'log_file_path': LaunchConfiguration('log_file_path'),
            }],
            # 配置日志级别
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        )
    ])
