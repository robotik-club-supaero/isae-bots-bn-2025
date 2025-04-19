#include "ActuatorState.hpp"

#include <Arduino.h>

ActuatorStateHolder::ActuatorStateHolder() : m_state() {}

void ActuatorStateHolder::requestState(uint16_t state) { m_state = state; }

std::optional<uint16_t> ActuatorStateHolder::getRequestedState() const { return m_state; }

void ActuatorStateHolder::clearRequestedState() { m_state.reset(); }

ActuatorStateManager::ActuatorStateManager(ros2::Node &node, const char *order_topic, const char *callback_topic)
    : m_state(std::make_shared<ActuatorStateHolder>()),
      m_sub(node.createSubscriber<std_msgs::Int16>(order_topic, ActuatorStateManager::orderCallback, m_state.get())),
      m_pub(node.createPublisher<std_msgs::Int16>(callback_topic)),
      m_msg() {}

void ActuatorStateManager::sendCallback(uint16_t data) {
    m_msg.data = data;
    m_pub->publish(&m_msg);
}

std::optional<uint16_t> ActuatorStateManager::getRequestedState() const { return m_state->getRequestedState(); }
void ActuatorStateManager::clearRequestedState() { m_state->clearRequestedState(); }

void ActuatorStateManager::orderCallback(void *msg, void *arg) {
    std_msgs::Int16 *msg_casted = (std_msgs::Int16 *)msg;
    ActuatorStateHolder *state = (ActuatorStateHolder *)arg;

    state->requestState(msg_casted->data);
}