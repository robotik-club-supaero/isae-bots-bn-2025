#ifndef _ACTUATOR_STATE_HPP_
#define _ACTUATOR_STATE_HPP_

#include <memory>
#include <optional>

#include "ros2/ros2.hpp"

class ActuatorStateHolder {
   public:
    ActuatorStateHolder();

    void requestState(uint16_t state);
    std::optional<uint16_t> getRequestedState() const;
    void clearRequestedState();

   private:
    std::optional<uint16_t> m_state;
};

class ActuatorStateManager {
   public:
    ActuatorStateManager(ros2::Node &node, const char *order_topic, const char *callback_topic);

    std::optional<uint16_t> getRequestedState() const;
    void clearRequestedState();

    void sendCallback(uint16_t data);

   private:
    std::shared_ptr<ActuatorStateHolder> m_state;
    ros2::Subscriber<std_msgs::Int16>m_sub;
    ros2::Publisher<std_msgs::Int16> m_pub;
    std_msgs::Int16 m_msg;
};

#endif