#ifndef _ACTUATOR_STATE_HPP_
#define _ACTUATOR_STATE_HPP_

#include <memory>
#include <optional>

#include "ros2/ros2.hpp"

/// Shared state holder. It is updated in ROS subscription callback, and read by the actuator.
/// It allows to transmit the orders without putting the actuators' logic directly in the callback.
class ActuatorStateHolder {
   public:
    ActuatorStateHolder();

    void requestState(uint16_t state);
    /// Is empty if no order has been received or if the last order was already processed.
    std::optional<uint16_t> getRequestedState() const;
    void clearRequestedState();

   private:
    std::optional<uint16_t> m_state;
};

// TODO: the name of this class could be improved
/// Creates and manages the publisher and subscriber for a specific actuator.
class ActuatorStateManager {
   public:
    ActuatorStateManager(ros2::Node &node, const char *order_topic, const char *callback_topic);

    std::optional<uint16_t> getRequestedState() const;
    void clearRequestedState();

    void sendCallback(uint16_t data);

   private:
    std::shared_ptr<ActuatorStateHolder> m_state;
    ros2::Subscriber<std_msgs::Int16> m_sub;
    ros2::Publisher<std_msgs::Int16> m_pub;
    std_msgs::Int16 m_msg;
};

#endif