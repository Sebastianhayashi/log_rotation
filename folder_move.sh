#!/bin/bash

# 设置目标目录
TARGET_DIR="/opt/ros/humble/include"

# 遍历目标目录下的所有一级子目录
for dir in "$TARGET_DIR"/*; do
  if [ -d "$dir" ]; then
    nested_dir="$dir/$(basename "$dir")"

    # 检查是否存在与父目录同名的嵌套目录
    if [ -d "$nested_dir" ]; then
      echo "移动 $nested_dir 中的内容到 $dir"
      sudo mv "$nested_dir"/* "$dir/"
      # 删除空的嵌套目录
      sudo rmdir "$nested_dir"
    fi
  fi
done

echo "嵌套目录移动完成。"
