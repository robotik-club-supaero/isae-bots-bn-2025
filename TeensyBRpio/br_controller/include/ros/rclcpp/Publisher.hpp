#ifndef _ROS_IMPL_PUBLISHER_HPP_
#define _ROS_IMPL_PUBLISHER_HPP_

#include "ros/message_cast.hpp"

#include "rclcpp/rclcpp.hpp"

namespace ros_rclcpp {

class Node;

template <typename T>
class Publisher {
  public:
  template <typename M>
    void publish(const M &msg) {
        m_msg = message_cast<T>(msg);
        m_publisher->publish(m_msg);
    }

  private:
    friend class Node;
    Publisher(typename rclcpp::Publisher<T>::SharedPtr inner) : m_publisher(inner) {}

    typename rclcpp::Publisher<T>::SharedPtr m_publisher;
    T m_msg;
};
} // namespace ros_rclcpp

#endif