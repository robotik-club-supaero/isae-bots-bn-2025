#ifndef _ROS_IMPL_SUBSCRIBER_HPP_
#define _ROS_IMPL_SUBCRIBER_HPP_

#include "ros/rclc/macros.hpp"
#include "ros/rclc/type_support.hpp"

#include <micro_ros_utilities/type_utilities.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <functional>

namespace ros_rclc {

class Node;

template <typename T>
class Subscription;

template <typename T>
class MessageWrapper : public T {
    friend class Subscription<T>;

  public:
    MessageWrapper(std::function<void(const T &)> callback) : T(), m_callback(callback) {}

  private:
    /**
     * @param msg Must point to a valid, allocated and properly initialized `const MessageWrapper<T>` object.
     * Calling this method with an argument that does not satisfy those requirements is undefined behaviour.
     */
    static void dispatch(const void *msg) {
        const MessageWrapper<T> *msgCast = static_cast<const MessageWrapper<T> *>(msg);
        const T *rawMsg = static_cast<const T *>(msgCast);
        msgCast->m_callback(*rawMsg);
    }

    std::function<void(const T &)> m_callback;
};

template <typename T>
class Subscription {
  public:
    Subscription(Subscription<T> &&subscription)
        : m_node(std::move(subscription.m_node)), m_subscription(std::move(subscription.m_subscription)), m_msg(std::move(subscription.m_msg)) {}
    ~Subscription() {
        if (m_subscription) {
            RCCHECK_SOFT(rcl_subscription_fini(m_subscription.get(), m_node.get()));
        }
        static micro_ros_utilities_memory_conf_t MEMORY_CONF = generateMemoryConf();
        RCCHECK_SOFT(micro_ros_utilities_destroy_message_memory(type_support_t<T>::get(), m_msg.get(), MEMORY_CONF) ? RCL_RET_OK : RCL_RET_ERROR);
    }

  private:
    using MsgT = MessageWrapper<T>;

    static micro_ros_utilities_memory_conf_t generateMemoryConf() {
        micro_ros_utilities_memory_conf_t conf = {0};
        conf.max_ros2_type_sequence_capacity = 20;
        return conf;
    }

    friend class Node;
    Subscription(std::shared_ptr<rcl_node_t> node, rclc_executor_t *executor, const char *topic, std::function<void(const T &)> callback)
        : m_node(std::move(node)), m_subscription(std::make_unique<rcl_subscription_t>()), m_msg(std::make_unique<MsgT>(callback)) {
        static micro_ros_utilities_memory_conf_t MEMORY_CONF = generateMemoryConf();
        RCCHECK_HARD(micro_ros_utilities_create_message_memory(type_support_t<T>::get(), m_msg.get(), MEMORY_CONF) ? RCL_RET_OK : RCL_RET_ERROR);
        RCCHECK_HARD(rclc_subscription_init_default(m_subscription.get(), m_node.get(), type_support_t<T>::get(), topic));
        RCCHECK_HARD(rclc_executor_add_subscription(executor, m_subscription.get(), m_msg.get(),
                                                    (rclc_subscription_callback_t)MessageWrapper<T>::dispatch, ON_NEW_DATA));
    }

    std::shared_ptr<rcl_node_t> m_node;
    std::unique_ptr<rcl_subscription_t> m_subscription;
    std::unique_ptr<MsgT> m_msg;
};

} // namespace ros_rclc

#endif