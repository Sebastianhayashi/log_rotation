#!/bin/bash

# 检查 ROS 2 安装是否可用
if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS 2 is not installed or not in your PATH."
    exit 1
fi

# 询问用户日志文件路径
read -p "Enter the log file path (default: $HOME/.ros/log/default_log.log): " log_file_path
LOG_FILE_PATH="${log_file_path:-$HOME/.ros/log/default_log.log}"
export LOG_FILE_PATH

# 询问用户最大日志文件大小
read -p "Enter the max log file size in bytes (default: 1048576, i.e., 1MB): " max_log_size
MAX_LOG_SIZE="${max_log_size:-1048576}"
export MAX_LOG_SIZE

# 询问用户最大备份文件数量
read -p "Enter the max number of backup log files to retain (default: 5): " max_log_files
MAX_LOG_FILES="${max_log_files:-5}"
export MAX_LOG_FILES

# 询问用户要运行的 ROS 2 程序
read -p "Enter the ROS 2 package name to run (e.g., log_rotation): " package_name
if [ -z "$package_name" ]; then
    echo "Error: Package name cannot be empty."
    exit 1
fi

read -p "Enter the ROS 2 executable name to run (e.g., log_rotation): " executable_name
if [ -z "$executable_name" ]; then
    echo "Error: Executable name cannot be empty."
    exit 1
fi

# 输出当前配置
echo "Starting ROS 2 node with the following log configuration:"
echo "LOG_FILE_PATH=$LOG_FILE_PATH"
echo "MAX_LOG_SIZE=$MAX_LOG_SIZE"
echo "MAX_LOG_FILES=$MAX_LOG_FILES"
echo "Package: $package_name"
echo "Executable: $executable_name"

# 启动 ROS 2 节点
ros2 run "$package_name" "$executable_name" &
ROS_PID=$!

# 捕获脚本中断信号，停止 ROS 2 节点
trap "echo 'Stopping ROS 2 node...'; kill $ROS_PID; exit" SIGINT SIGTERM

# 等待 ROS 2 节点运行结束
wait $ROS_PID

echo "ROS 2 node has stopped."
