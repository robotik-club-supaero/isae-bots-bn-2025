#ifndef _ROS_IMPL_SUBSCRIBER_HPP_
#define _ROS_IMPL_SUBCRIBER_HPP_

#include "ros/micro_ros/macros.hpp"
#include "ros/micro_ros/type_support.hpp"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <functional>

namespace ros2 {

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
    Subscription(Subscription<T> &&subscription) : m_subscription(std::move(subscription.m_subscription)), m_msg(std::move(subscription.m_msg)) {}

  private:
    using MsgT = MessageWrapper<T>;

    friend class Node;
    Subscription(rcl_node_t *node, rclc_executor_t *executor, const char *topic, std::function<void(const T &)> callback)
        : m_subscription(std::make_unique<rcl_subscription_t>()), m_msg(std::make_unique<MsgT>(callback)) {
        RCCHECK_HARD(rclc_subscription_init_default(m_subscription.get(), node, T::type_support(), topic));
        RCCHECK_HARD(rclc_executor_add_subscription(executor, m_subscription.get(), m_msg.get(),
                                                    (rclc_subscription_callback_t)MessageWrapper<T>::dispatch, ON_NEW_DATA));
    }

    std::unique_ptr<rcl_subscription_t> m_subscription;
    std::unique_ptr<MsgT> m_msg;
};

} // namespace ros2

#endif