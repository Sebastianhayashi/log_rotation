# ROS 日志系统调研报告

ROS2 中有多个集成的日志系统，这些日志系统提高了日志管理的灵活性和扩展性，主要的日志系统是：`rcl_logging_spdlog` 以及 `rcl_logging_noop`。

1. `rcl_logging_spdlog`

这是基于 `spdlog` 所实现的日志系统，能够支持多线程高效的日志记录功能，能够支持输出从 `DEBUG` 到 `FATAL` 级别的日志级别输出，同时能够进行日志轮转，将日志文件输出到指定的位置。

2. `rcl_logging_noop`

这是一个空操作的日志系统，它并不会记录任何的日志记录，这是用于轻量化或者测试环境中。

## ROS1 与 ROS2 的日志系统对比

### 集中化与灵活性

ROS1 中的日志系统更加的集中化，也就是所有的日志文件都会被存储在 rosout.log 当中，使用 `rosout` 指令查看节点输出的汇总信息。
这样的日志管理方式主打的就是集中管理，相对的在大规模的系统中，使用这样的日志系统会面临不同的问题，例如对于庞大数量所产生复杂的管理问题。

ROS2 的日志管理方式更加的灵活，能够单独的为每一个节点进行独立的日志记录，并且放在不同的位置。这种方式使得日志管理的灵活度大大的增加，并且能够根据节点名称或者其他的标注方式，将日志输出到不同的路径。

### 日志轮转

ROS1 不支持日志轮转。

ROS2 中的 `rcl_logging_spdlog` 能够基于文件大小去进行日志轮转，也就是说当日志文件达到了指定的大小，就会创建新的日志文件， 

### 拓展性

## ROS2 日志系统使用

1. 设置环境变量

可以直接通过环境变量 `ROS_LOG_DIR` 去指定日志文件的输出位置：

```
export ROS_LOG_DIR=/home/user/my_ros_logs
```

1. 日志级别设置

ROS2 能够支持使用命令行参数去设置日志级别：

```
// 将日志级别设置为 DEBUG

ros2 run <package> <node> --ros-args --log-level DEBUG
```

```
// 同时也可以在代码中进行级别设置

rclcpp::Logger logger = rclcpp::get_logger("example_logger");
logger.set_level(rclcpp::Logger::Level::Debug);
```

3. 日志轮转

通过使用 `rcl_logging_spdlo`g` 和 `spdlog` 的 API，可以防止日志文件无限增长：

```
// 最大 5MB, 3 个文件

auto logger = spdlog::rotating_logger_mt("logger", "/path/to/logfile.log", 1048576 * 5, 3);  
```