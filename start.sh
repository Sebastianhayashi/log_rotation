#!/bin/bash

# 检查是否存在 install/setup.bash 文件
if [ ! -f "install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please run 'colcon build' first."
    exit 1
fi

source install/setup.bash

# 用户输入日志文件路径、最大文件大小和文件数量
read -p "Enter the directory where you want to store logs (default: $HOME/.ros/log): " input_log_dir
LOG_DIR="${input_log_dir:-$HOME/.ros/log}"

read -p "Enter the log file name (default: log_rotation_node.log): " input_log_file
LOG_FILE="${input_log_file:-log_rotation_node.log}"
FULL_LOG_PATH="$LOG_DIR/$LOG_FILE"

read -p "Enter the max log file size in bytes before rotation (default: 1048576, i.e., 1MB): " input_max_size
MAX_SIZE="${input_max_size:-1048576}"

read -p "Enter the max number of backup log files to retain (default: 5): " input_max_files
MAX_FILES="${input_max_files:-5}"

# 用户输入 ROS 2 程序
read -p "Enter the ROS 2 package name: " package_name
read -p "Enter the ROS 2 executable name (log_rotation_node by default): " executable_name
executable_name="${executable_name:-log_rotation_node}"

# 检查并创建日志目录（如果不存在）
mkdir -p "$LOG_DIR"
if [ ! -d "$LOG_DIR" ]; then
    echo "Error: Failed to create log directory $LOG_DIR."
    exit 1
fi
echo "Log directory created or already exists at: $LOG_DIR"

# 检查日志文件路径的变量设置
echo "Checking log path settings..."
echo "Log Directory: $LOG_DIR"
echo "Log File Path: $FULL_LOG_PATH"
echo "Max Log Size: $MAX_SIZE bytes"
echo "Max Log Files: $MAX_FILES"

# 配置日志轮转环境变量
export LOG_FILE_PATH="$FULL_LOG_PATH"
export MAX_LOG_SIZE="$MAX_SIZE"
export MAX_LOG_FILES="$MAX_FILES"

# 确保日志目录具有写权限
echo "Checking directory permissions for $LOG_DIR..."
if [ ! -w "$LOG_DIR" ]; then
    echo "Error: No write permission for log directory $LOG_DIR. Attempting to set permissions..."
    chmod -R 755 "$LOG_DIR"
    if [ ! -w "$LOG_DIR" ]; then
        echo "Error: Unable to set write permissions for $LOG_DIR."
        exit 1
    fi
fi
echo "Permissions are correctly set for $LOG_DIR."

# 清理旧日志文件
echo "Do you want to delete existing log files? (y/n)"
read -r delete_logs
if [[ "$delete_logs" == "y" ]]; then
    rm -f "$LOG_DIR/$LOG_FILE" "$LOG_DIR/${LOG_FILE%.*}_"*.log
    echo "Old log files deleted."
fi

# 启动 ROS 2 程序
echo "Starting ROS2 node with log rotation settings..."
ros2 run "$package_name" "$executable_name" &
ROS_PID=$!

# 设置 trap 以便在 Ctrl+C 时终止 ROS 2 程序
trap 'kill $ROS_PID' SIGINT

# 等待 ROS 2 程序执行完成
wait $ROS_PID
echo "ROS2 node has stopped."
