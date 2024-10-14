#!/bin/bash

# 设置目标目录
TARGET_DIR="/opt/ros/humble/include"

# 遍历目标目录下的所有一级子目录
for dir in "$TARGET_DIR"/*; do
  if [ -d "$dir" ]; then
    # 查找与父目录同名的嵌套子目录
    nested_dir=$(find "$dir" -mindepth 2 -type d -name "$(basename "$dir")")

    if [ -n "$nested_dir" ]; then
      echo "处理目录：$nested_dir -> $dir"
      # 移动嵌套子目录的内容到上一层
      sudo mv "$nested_dir"/* "$dir/"
      # 删除空的嵌套目录
      sudo rmdir "$nested_dir"
    fi
  fi
done

echo "所有嵌套文件已处理完毕。"
