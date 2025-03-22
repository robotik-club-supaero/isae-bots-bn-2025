#ifndef _ROS_IMPL_PUBLISHER_HPP_
#define _ROS_IMPL_PUBLISHER_HPP_

#include "ros/message_cast.hpp"
#include "ros/rclc/macros.hpp"
#include "ros/rclc/type_support.hpp"

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <memory>

namespace ros_rclc {

class Node;

template <typename T>
class Publisher {
  public:
    Publisher(Publisher<T> &&publisher)
        : m_node(std::move(publisher.m_node)), m_publisher(std::move(publisher.m_publisher)), m_msg(publisher.m_msg) {}

    template <typename M>
    void publish(const M &msg) {
        m_msg = message_cast<T>(msg);
        RCCHECK_HARD(rcl_publish(m_publisher.get(), &m_msg, NULL));
    }

    ~Publisher() { RCCHECK_SOFT(rcl_publisher_fini(m_publisher.get(), m_node.get())); }

  private:
    friend class Node;

    Publisher(std::shared_ptr<rcl_node_t> node, const string_t &topic)
        : m_node(std::move(node)), m_publisher(std::make_unique<rcl_publisher_t>()), m_msg() {
        RCCHECK_HARD(rclc_publisher_init_best_effort(m_publisher.get(), m_node.get(), type_support_t<T>::get(), topic.c_str()));
    }

    std::shared_ptr<rcl_node_t> m_node;
    std::unique_ptr<rcl_publisher_t> m_publisher;
    T m_msg;
};
} // namespace ros_rclc

#endif