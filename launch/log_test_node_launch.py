from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 定义参数并设置默认值
    log_file_path_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value='/tmp/ros2_logs/log_rotation.log',
        description='Path to the log file'
    )

    max_size_arg = DeclareLaunchArgument(
        'max_size',
        default_value='1048576',  # 1MB
        description='Maximum size of log file in bytes'
    )

    max_files_arg = DeclareLaunchArgument(
        'max_files',
        default_value='5',
        description='Maximum number of log files to retain'
    )

    # 启动节点并传递参数
    return LaunchDescription([
        log_file_path_arg,
        max_size_arg,
        max_files_arg,
        Node(
            package='ros2_log_rotation',
            executable='log_rotation_node',
            name='log_rotation_node',
            output='screen',
            parameters=[{
                'log_file_path': LaunchConfiguration('log_file_path'),
                'max_size': LaunchConfiguration('max_size'),
                'max_files': LaunchConfiguration('max_files'),
            }]
        )
    ])
