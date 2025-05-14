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
    Dispatcher(ros2::Node &node, std::weak_ptr<TManager> manager);

  private:
    template <typename T>
    class DispatcherSubscription : public ros2::Subscription<T> {
      public:
        template <std::invocable<TManager &, const T &> Fun>
        DispatcherSubscription(ros2::Node &node, const char *topic, const std::weak_ptr<TManager> &manager, Fun &&callback)
            : ros2::Subscription<T>(node.createSubscription<T>(topic, [callback = std::forward<Fun>(callback), manager](const T &msg) {
                  if (auto lock = manager.lock()) {
                      callback(*lock, msg);
                  }
              })) {}
    };

    /// /br/goTo
    DispatcherSubscription<br_messages::msg::DisplacementOrder> m_subGoTo;
    /// /br/stop
    DispatcherSubscription<std_msgs::msg::Empty> m_subStop;
    /// /br/command
    DispatcherSubscription<br_messages::msg::Command> m_subCommand;
    /// /br/idle
    DispatcherSubscription<std_msgs::msg::Bool> m_subIdle;
    /// /br/resetPosition
    DispatcherSubscription<br_messages::msg::Position> m_subReset;
    /// /br/gains
    DispatcherSubscription<br_messages::msg::GainsPid> m_subGains;
    /// /br/setSpeed
    DispatcherSubscription<std_msgs::msg::Int16> m_subSpeed;
};

#endif