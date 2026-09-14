// arm_logger.h
// arm_core 库内部使用的日志器（rclcpp 的 RCLCPP_* 宏需要一个 logger 对象）。
// 库代码不属于某个节点，所以用一个模块级 logger，名字固定为 "arm_core"。
#pragma once

#include <rclcpp/logger.hpp>

namespace arm_core_logging {
rclcpp::Logger logger();
}  // namespace arm_core_logging
