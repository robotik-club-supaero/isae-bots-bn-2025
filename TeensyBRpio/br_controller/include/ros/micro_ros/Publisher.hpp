#ifndef _ROS_IMPL_PUBLISHER_HPP_
#define _ROS_IMPL_PUBLISHER_HPP_

#include "ros/micro_ros/macros.hpp"
#include "ros/micro_ros/type_support.hpp"
#include "ros/qos/Reliability.hpp"

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <memory>

namespace ros2 {

class Node;

template <typename T>
class Publisher {
  public:
    Publisher(Publisher<T> &&publisher) : m_publisher(std::move(publisher.m_publisher)) {}
    Publisher<T> &operator=(Publisher<T> &&other) {
        m_publisher = std::move(other.m_publisher);
        return *this;
    }

    void publish(const T &msg) { RCCHECK_HARD(rcl_publish(m_publisher.get(), &msg, NULL)); }

  private:
    friend class Node;

    Publisher(rcl_node_t *node, const char *topic, QosReliability reliability) : m_publisher(std::make_unique<rcl_publisher_t>()) {
        if (reliability == AllowBestEffort) {
            RCCHECK_HARD(rclc_publisher_init_best_effort(m_publisher.get(), node, T::type_support(), topic));
        } else {
            RCCHECK_HARD(rclc_publisher_init_default(m_publisher.get(), node, T::type_support(), topic));
        }
    }

    std::unique_ptr<rcl_publisher_t> m_publisher;
};
} // namespace ros2

#endif