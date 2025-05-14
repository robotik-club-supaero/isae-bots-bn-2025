#ifndef _ROS_IMPL_PUBLISHER_HPP_
#define _ROS_IMPL_PUBLISHER_HPP_

#include "ros/qos/Reliability.hpp"

#include <rclcpp/rclcpp.hpp>

namespace ros2 {

class Node;

template <typename T>
class Publisher {
  public:
    void publish(const T &msg) { m_inner->publish(msg); }

  private:
    friend class Node;
    typename rclcpp::Publisher<T>::SharedPtr m_inner;

    Publisher(rclcpp::Publisher<T>::SharedPtr &&inner) : m_inner(std::move(inner)) {}
};
} // namespace ros2
#endif