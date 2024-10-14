#!/bin/bash

# 遍历 /usr/include 下的所有子目录
for dir in /usr/include/*; do
  if [ -d "$dir" ]; then
    nested_dir=$(find "$dir" -mindepth 2 -type d -name "$(basename "$dir")")
    
    if [ -n "$nested_dir" ]; then
      echo "处理目录：$nested_dir -> $dir"
      # 移动嵌套目录中的所有文件到上一层
      sudo mv "$nested_dir"/* "$dir/"
      # 删除空的嵌套目录
      sudo rmdir "$nested_dir"
    fi
  fi
done
