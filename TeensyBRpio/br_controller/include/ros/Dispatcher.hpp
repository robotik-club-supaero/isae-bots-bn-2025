#ifndef _ROS_DISPATCHER_HPP_
#define _ROS_DISPATCHER_HPP_

#include "manager/ControllerManager.hpp"
#include "ros/ros_impl.hpp"

#include <memory>

/**
 * Implementation of ROS subscriptions
 */
template <typename TManager>
class Dispatcher {
  public:
    Dispatcher(ros_impl::Node &node, std::weak_ptr<TManager> manager);

  private:
    template <typename T>
    class DispatcherSubscription : public ros_impl::subscription_t<T> {
      public:
        template <std::invocable<TManager &, const T &> Fun>
        DispatcherSubscription(ros_impl::Node &node, const char *topic, const std::weak_ptr<TManager> &manager, Fun callback)
            : ros_impl::subscription_t<T>(node.createSubscription<T>(topic, [callback, manager](const T &msg) {
                  if (auto lock = manager.lock()) {
                      callback(*lock, msg);
                  }
              })) {}
    };

    /// /br/goTo
    DispatcherSubscription<ros_impl::messages::displacement_order_t> m_subGoTo;
    /// /br/stop
    DispatcherSubscription<ros_impl::messages::empty_t> m_subStop;
    /// /br/command
    DispatcherSubscription<ros_impl::messages::command_t> m_subCommand;
    /// /br/idle
    DispatcherSubscription<ros_impl::messages::bool_t> m_subIdle;
    /// /br/resetPosition
    DispatcherSubscription<ros_impl::messages::position_t> m_subReset;
    /// /br/gains
    DispatcherSubscription<ros_impl::messages::gains_t> m_subGains;
    /// /br/setSpeed
    DispatcherSubscription<ros_impl::messages::msg_int16_t> m_subSpeed;
};

#endif