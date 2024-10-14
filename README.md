# ROS-log

## 使用

```
colcon build --packages-select log_rotation_test

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run log_rotation_test log_rotation_node
```

## 文件结构

```
log_rotation_test/
├── CMakeLists.txt       # CMake 配置文件，用于构建项目
├── package.xml          # ROS 2 包元数据文件
├── README.md            # 项目文档（本文件）
├── src/
│   └── log_rotation_node.cpp   # 主节点代码
├── log/                 # 日志目录（自动生成）
└── install/             # 安装目录（构建生成）

```