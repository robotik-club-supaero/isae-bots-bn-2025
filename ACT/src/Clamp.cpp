#include "Clamp.hpp"

#include "configuration.hpp"

// TODO what should be the initial state?

ClampHandle::ClampHandle(int servo_pin, const char *name)
    : ActuatorHandle(servo_pin, name, std::array{CLAMP_CLOSED_POS, CLAMP_OPEN_POS}, /* initial_state = */ OPEN) {}

Clamp::Clamp(int servo_pin, ros2::Node &node, const char *name, const char *order_topic, const char *callback_topic)
    : m_handle(std::make_shared<ClampHandle>(servo_pin, name)),
      m_ros(node, order_topic, callback_topic, [handle_weak = std::weak_ptr<ClampHandle>(m_handle)](uint16_t order) {
          if (auto handle = handle_weak.lock()) {
              handle->setState(order);
          }
      }) {}

void Clamp::loop() {
    if (m_handle->needsNotify() && micros() - m_handle->lastChangeTime() > CALLBACK_INTERVAL) {
        m_ros.sendCallback(m_handle->getState());
        m_handle->markNotified();
    }
}

Clamp1::Clamp1(ros2::Node &node) : Clamp(CLAMP_1_PIN, node, "CLAMP1", "/act/order/clamp_1", "/act/callback/clamp_1") {}

Clamp2::Clamp2(ros2::Node &node) : Clamp(CLAMP_2_PIN, node, "CLAMP2", "/act/order/clamp_2", "/act/callback/clamp_2") {}