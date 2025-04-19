#ifndef _ROS_NODE_HPP_
#define _ROS_NODE_HPP_

#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <memory>

#include "ros2/Publisher.hpp"
#include "ros2/Subscriber.hpp"
#include "ros2/__macros.hpp"

namespace ros2 {
class Node {
   public:
    Node(const String &name) {
        RCCHECK_HARD(rclc_support_init(m_support.get(), 0, NULL, m_allocator.get()));
        RCCHECK_HARD(rclc_node_init_default(m_node.get(), name.c_str(), "", m_support.get()));
        RCCHECK_HARD(rclc_executor_init(m_executor.get(), &m_support->context, 7, m_allocator.get()));
    }
    ~Node() {
        if (m_executor) {
            RCCHECK_SOFT(rclc_executor_fini(m_executor.get()));
            RCCHECK_SOFT(rcl_node_fini(m_node.get()));
            RCCHECK_SOFT(rclc_support_fini(m_support.get()));
        }
    }

    void spin() { RCCHECK_HARD(rclc_executor_spin_some(m_executor.get(), 0)); }

    template <typename T>
    Subscriber<T> createSubscriber(const String &topic, std::function<void(const T &)> callback) {
        return Subscriber<T>(m_node, m_executor.get(), topic, callback);
    }

    template <typename T>
    Publisher<T> createPublisher(const String &topic) {
        return Publisher<T>(m_node, topic);
    }

   private:
    std::unique_ptr<rcl_allocator_t> m_allocator = std::make_unique<rcl_allocator_t>(rcl_get_default_allocator());
    std::unique_ptr<rclc_support_t> m_support = std::make_unique<rclc_support_t>();
    std::unique_ptr<rclc_executor_t> m_executor = std::make_unique<rclc_executor_t>();
    std::shared_ptr<rcl_node_t> m_node = std::make_shared<rcl_node_t>();
};

}  // namespace ros2

#endif