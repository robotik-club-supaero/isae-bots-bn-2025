#ifndef _ROS_IMPL_NODE_HPP_
#define _ROS_IMPL_NODE_HPP_

#include "ros/qos/Reliability.hpp"
#include "ros/rclcpp/Publisher.hpp"

#include <rclcpp/rclcpp.hpp>

namespace ros2 {

template <typename T>
using Subscription = rclcpp::Subscription<T>::SharedPtr;

class Node {
  public:
    Node(const char *name) : m_node(std::make_shared<rclcpp::Node>(name)) {}

    void spin_once() { rclcpp::spin_some(m_node); }

    template <typename T>
    Subscription<T> createSubscription(const char *topic, std::function<void(const T &)> callback) {
        return m_node->template create_subscription<T>(topic, 10, [callback](const T::SharedPtr msg) -> void { callback(*msg); });
    }

    template <typename T>
    Publisher<T> createPublisher(const char *topic, QosReliability reliability = ReliableOnly) {
        std::ignore = reliability; // Always use a reliable QOS for the simulation
        return Publisher<T>(m_node->template create_publisher<T>(topic, 10));
    }

    void sendLog(LogSeverity severity, const char *message) {
        auto logger = m_node->get_logger();

        if (severity == INFO) {
            RCLCPP_INFO(logger, message);
        } else if (severity == WARN) {
            RCLCPP_WARN(logger, message);
        } else if (severity == ERROR) {
            RCLCPP_ERROR(logger, message);
        } else if (severity == FATAL) {
            RCLCPP_FATAL(logger, message);
        } else if (severity == DEBUG) {
            RCLCPP_DEBUG(logger, message);
        } else {
            RCLCPP_ERROR(logger, "Unknown log type: %d; defaulting to INFO", severity);
            sendLog(INFO, message);
        }
    }

  private:
    rclcpp::Node::SharedPtr m_node;
};
} // namespace ros2
#endif