#!/bin/bash

# 检查 install/setup.bash 是否存在
if [ ! -f "install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please run 'colcon build' first."
    exit 1
fi

source install/setup.bash

# 询问用户日志文件路径、大小和备份数量
read -p "Enter the directory where you want to store logs (default: $HOME/.ros/log): " input_log_dir
LOG_DIR="${input_log_dir:-$HOME/.ros/log}"

read -p "Enter the log file name (default: log_rotation_node.log): " input_log_file
LOG_FILE="${input_log_file:-log_rotation_node.log}"
FULL_LOG_PATH="$LOG_DIR/$LOG_FILE"

read -p "Enter the max log file size in bytes before rotation (default: 1048576, i.e., 1MB): " input_max_size
MAX_SIZE="${input_max_size:-1048576}"

read -p "Enter the max number of backup log files to retain (default: 5): " input_max_files
MAX_FILES="${input_max_files:-5}"

# 用户输入目标 ROS 2 程序
read -p "Enter the ROS 2 package name: " package_name
read -p "Enter the ROS 2 executable name: " executable_name

# 创建日志目录（如果不存在）
mkdir -p "$LOG_DIR"
if [ ! -d "$LOG_DIR" ]; then
    echo "Error: Failed to create log directory $LOG_DIR."
    exit 1
fi
echo "Log directory created or already exists at: $LOG_DIR"

echo "LOG_DIR is set to: $LOG_DIR"
echo "Log path is $FULL_LOG_PATH"
echo "LOG_FILE is set to: $LOG_FILE"
echo "MAX_SIZE is set to: $MAX_SIZE bytes"
echo "MAX_FILES is set to: $MAX_FILES"

# 配置日志轮转的环境变量
export LOG_FILE_PATH="$FULL_LOG_PATH"
export MAX_LOG_SIZE="$MAX_SIZE"
export MAX_LOG_FILES="$MAX_FILES"

# 清理之前的日志文件
echo "Do you want to delete existing log files? (y/n)"
read -r delete_logs
if [[ "$delete_logs" == "y" ]]; then
    echo "Cleaning up old log files..."
    rm -f "$LOG_DIR/$LOG_FILE"
    rm -f "$LOG_DIR/${LOG_FILE%.*}.*.log"
    echo "Old log files deleted."
fi

# 启动用户指定的目标 ROS 2 程序
echo "Starting ROS2 node with log rotation settings..."
ros2 run "$package_name" "$executable_name" &
ROS_PID=$!

# 设置 trap 以在脚本终止时终止 ROS2 节点
trap 'echo "Terminating..."; kill $ROS_PID; exit' SIGINT SIGTERM

# 监控日志文件大小并轮转
while kill -0 $ROS_PID 2>/dev/null; do
    LOG_SIZE=$(stat -c%s "$FULL_LOG_PATH" 2>/dev/null || echo 0)
    echo "Current log file size: $LOG_SIZE bytes"
    if [ "$LOG_SIZE" -ge "$MAX_SIZE" ]; then
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        mv "$FULL_LOG_PATH" "$LOG_DIR/${LOG_FILE%.*}_$TIMESTAMP.log"
        touch "$FULL_LOG_PATH"  # 创建新的日志文件
        echo "Log file rotated. New log file created at $FULL_LOG_PATH"

        # 保留最多 MAX_FILES 个备份日志
        NUM_LOGS=$(ls "$LOG_DIR/${LOG_FILE%.*}_"*.log 2>/dev/null | wc -l)
        if [ "$NUM_LOGS" -gt "$MAX_FILES" ]; then
            OLDEST_LOG=$(ls "$LOG_DIR/${LOG_FILE%.*}_"*.log | head -n 1)
            rm "$OLDEST_LOG"
            echo "Oldest log file $OLDEST_LOG deleted"
        fi
    fi
    sleep 5  # 每5秒检查一次文件大小
done

echo "ROS2 node has stopped."
