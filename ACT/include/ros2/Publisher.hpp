#ifndef _ROS_PUBLISHER_HPP_
#define _ROS_PUBLISHER_HPP_

#include "ros2/__macros.hpp"

namespace ros2 {

class Node;

template <typename Msg>
class Publisher {
   public:
    Publisher(Publisher<Msg> &&publisher) : m_node(std::move(publisher.m_node)), m_publisher(std::move(publisher.m_publisher)) {}

    void publish(const Msg &msg) {
        if (!m_publisher) {
            std::abort();  // Use after move
        }
        RCCHECK_HARD(rcl_publish(m_publisher.get(), &msg, NULL));
    }

    ~Publisher() {
        if (m_publisher) {
            RCCHECK_SOFT(rcl_publisher_fini(m_publisher.get(), m_node.get()));
        }
    }

   private:
    friend class Node;

    Publisher(std::shared_ptr<rcl_node_t> node, const String &topic) : m_node(std::move(node)), m_publisher(std::make_unique<rcl_publisher_t>()) {
        RCCHECK_HARD(rclc_publisher_init_default(m_publisher.get(), m_node.get(), Msg::type_support(), topic.c_str()));
    }

    std::shared_ptr<rcl_node_t> m_node;
    std::unique_ptr<rcl_publisher_t> m_publisher;
};
}  // namespace ros2

#endif