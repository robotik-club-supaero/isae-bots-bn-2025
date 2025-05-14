#ifndef _ROS_IMPL_RCLCPP_HPP_
#define _ROS_IMPL_RCLCPP_HPP_

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

#include <span>

namespace ros2 {

inline void init(int argc, const char *const *argv) {
    rclcpp::init(argc, argv);
}

inline void shutdown() {
    rclcpp::shutdown();
}

} // namespace ros2

namespace br_messages {
namespace __detail {
class __PathView : public std::span<const br_messages::msg::Point> {
  public:
    using span::span;
    __PathView(const std::vector<br_messages::msg::Point> &points) : __PathView(points.begin(), points.end()) {}
};
} // namespace __detail
} // namespace br_messages

#endif