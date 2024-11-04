#!/bin/bash

# 检查是否已经构建项目
if [ ! -f "install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please run 'colcon build' first."
    exit 1
fi

source install/setup.bash

# 询问用户存放日志的位置
read -p "Enter the directory where you want to store logs (default: $HOME/.ros/log): " input_log_dir
LOG_DIR="${input_log_dir:-$HOME/.ros/log}"

# 询问用户日志文件的命名
read -p "Enter the log file name (default: log_rotation_node.log): " input_log_file
LOG_FILE="${input_log_file:-log_rotation_node.log}"
FULL_LOG_PATH="$LOG_DIR/$LOG_FILE"  # 组合完整路径

# 询问用户日志轮转的触发大小（单位：字节）
read -p "Enter the max log file size in bytes before rotation (default: 1048576, i.e., 1MB): " input_max_size
MAX_SIZE="${input_max_size:-1048576}"

# 询问用户保留的最大日志文件数量
read -p "Enter the max number of backup log files to retain (default: 5): " input_max_files
MAX_FILES="${input_max_files:-5}"

# 询问用户是否启用压缩和备份
echo "Do you want to enable log compression and backup? (y/n)"
read -r enable_backup
if [[ "$enable_backup" == "y" ]]; then
    ARCHIVE_DIR="$LOG_DIR/archive"
    mkdir -p "$ARCHIVE_DIR"
    BACKUP_ENABLED=true
else
    BACKUP_ENABLED=false
fi

# 清理之前的日志文件
echo "Do you want to delete existing log files? (y/n)"
read -r delete_logs
if [[ "$delete_logs" == "y" ]]; then
    echo "Cleaning up old log files..."
    rm -f "$LOG_DIR/$LOG_FILE"
    rm -f "$LOG_DIR/${LOG_FILE%.*}.*.log"
    echo "Old log files deleted."
fi

# 输出配置
echo "LOG_DIR is set to: $LOG_DIR"
echo "LOG_FILE is set to: $LOG_FILE"
echo "MAX_SIZE is set to: $MAX_SIZE bytes"
echo "MAX_FILES is set to: $MAX_FILES"
echo "Backup enabled: $BACKUP_ENABLED"

# 启动 ROS2 节点，生成日志，并将日志路径、轮转大小、文件数量传递给程序
echo "Starting ROS2 node to generate logs..."
source install/setup.bash
ros2 run log_rotation log_test_node "$FULL_LOG_PATH" "$MAX_SIZE" "$MAX_FILES" &

# 等待日志生成
sleep 10

# 检查日志文件是否生成
if [ -f "$FULL_LOG_PATH" ]; then
    echo "Log file $LOG_FILE has been created successfully in $LOG_DIR."
else
    echo "Error: Log file $LOG_FILE was not created."
    exit 1
fi

# 检查日志文件内容是否包含期望的日志级别
echo "Checking log file contents..."
if grep -q "INFO log message" "$FULL_LOG_PATH"* && \
   grep -q "WARNING log message" "$FULL_LOG_PATH"* && \
   grep -q "ERROR log message" "$FULL_LOG_PATH"*; then
    echo "Log file contains expected log messages (INFO, WARNING, ERROR)."
else
    echo "Error: Log file does not contain expected log messages."
    exit 1
fi

# 等待日志轮转并检查是否满足文件数量限制
echo "Generating more logs to trigger log rotation..."
sleep 10  # 延长等待时间以满足轮转条件

# 检查轮转文件数量是否超过限制
backup_files=($(ls "$LOG_DIR/$LOG_FILE".* 2>/dev/null))
backup_count=${#backup_files[@]}

if [ "$backup_count" -le "$MAX_FILES" ]; then
    echo "Log rotation is working correctly, no more than $MAX_FILES backup files."
else
    echo "Error: More than $MAX_FILES backup log files found."
    exit 1
fi

# 如果启用备份，则进行日志压缩和存档
if [ "$BACKUP_ENABLED" = true ]; then
    echo "Archiving and compressing old log files..."
    ARCHIVE_FILE="$ARCHIVE_DIR/log_backup_$(date +%Y-%m-%d_%H-%M-%S).tar.gz"
    tar -czf "$ARCHIVE_FILE" -C "$LOG_DIR" "${LOG_FILE%.*}.*.log"
    echo "Old logs have been archived to $ARCHIVE_FILE."
fi

# 停止 ROS2 节点
echo "Stopping ROS2 node..."
pkill -f "ros2 run log_rotation log_test_node"

echo "Test completed successfully."
