#ifndef _ROS_IMPL_HPP_
#define _ROS_IMPL_HPP_

#include "ros/rclcpp/Node.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16.hpp>

#include "br_messages/msg/command.hpp"
#include "br_messages/msg/displacement_order.hpp"
#include "br_messages/msg/gains_pid.hpp"
#include "br_messages/msg/log_entry.hpp"
#include "br_messages/msg/odos_count.hpp"
#include "br_messages/msg/point.hpp"
#include "br_messages/msg/position.hpp"

namespace ros_impl {
using namespace ros_rclcpp;

using node_t = Node;

template <typename T>
using publisher_t = Publisher<T>;

template <typename T>
using subscription_t = rclcpp::Subscription<T>::SharedPtr;
} // namespace ros_impl

#define DEFINE_ROS_MESSAGE(NAME, PACKAGE, MSG)                                                                                                       \
    namespace ros_impl {                                                                                                                             \
    namespace messages {                                                                                                                             \
    using NAME = PACKAGE::msg::MSG;                                                                                                                 \
    }                                                                                                                                                \
    }

template <>
inline std::span<const br_messages::msg::Point>
message_cast<std::span<const br_messages::msg::Point>, std::vector<br_messages::msg::Point>>(const std::vector<br_messages::msg::Point> &msg) {
    return std::span(msg);
}

#endif