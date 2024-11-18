# README

该仓库用于调研 ROS2 日志系统，在 11.16 2024 的 openEuler summit ROS SIG 上与大家分享了 ROS2 系统的一个日志系统的设计以及大致上如何去使用。

**会议大致内容**如下：

ROS2 日志系统摒弃了 ROS1 中对于 `/rosout` 的依赖，采用了更加灵活的日志管理方式，强调了多目标输出、灵活的日志级别控制以及更加丰富的 API。

简要介绍了 ROS2 中的 `rclpy` `rclcpp` `rcl` `rcutils` `rcl_logging` 子系统结构，解释他们是如何协同实现上述功能。

最后对比了 ROS1 与 ROS2 的区别，简要列出两者在架构、日志级别控制、输出方式等方面的主要差异和改进，突出 ROS 2 的灵活性和模块化设计优势。

在会上讨论中也讨论了说虽然 ROS2 的日志系统更加的灵活可以适应不同的任务场景，但对于开发者而言的学习成本也随之增加。