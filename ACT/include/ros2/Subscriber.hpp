#ifndef _ROS_SUBSCRIBER_HPP_
#define _ROS_SUBCRIBER_HPP_

#include "ros2/__macros.hpp"

namespace ros2 {

class Node;

template <typename T>
class Subscriber;

template <typename T>
class MessageWrapper : public T {
    friend class Subscriber<T>;

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

template <typename Msg>
class Subscriber {
   public:
    Subscriber(Subscriber<Msg> &&subscriber)
        : m_node(std::move(subscriber.m_node)), m_subscriber(std::move(subscriber.m_subscriber)), m_msg(std::move(subscriber.m_msg)) {}
    ~Subscriber() {
        if (m_subscriber) {
            RCCHECK_SOFT(rcl_subscription_fini(m_subscriber.get(), m_node.get()));
        }
    }

   private:
    using MsgT = MessageWrapper<Msg>;

    friend class Node;
    Subscriber(std::shared_ptr<rcl_node_t> node, rclc_executor_t *executor, const String &topic, std::function<void(const Msg &)> callback)
        : m_node(std::move(node)), m_subscriber(std::make_unique<rcl_subscription_t>()), m_msg(std::make_unique<MsgT>(callback)) {
        RCCHECK_HARD(rclc_subscription_init_default(m_subscriber.get(), m_node.get(), Msg::type_support(), topic.c_str()));
        RCCHECK_HARD(rclc_executor_add_subscription(executor, m_subscriber.get(), m_msg.get(),
                                                    (rclc_subscription_callback_t)MessageWrapper<Msg>::dispatch, ON_NEW_DATA));
    }

    std::shared_ptr<rcl_node_t> m_node;
    std::unique_ptr<rcl_subscription_t> m_subscriber;
    std::unique_ptr<MsgT> m_msg;
};

}  // namespace ros2

#endif