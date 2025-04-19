#include "Clamp.hpp"

#include "configuration.hpp"
#include "logging.hpp"

// TODO what should be the initial state?

ClampServo::ClampServo(int servo_pin) : ActuatorServo(servo_pin, std::array{CLAMP_CLOSED_POS, CLAMP_OPEN_POS}, /* initial_state = */ OPEN) {}

Clamp::Clamp(int servo_pin1, int servo_pin2, ros2::Node &node, int level, const char *order_topic, const char *callback_topic)
    : m_level(level), m_ros(node, order_topic, callback_topic), m_clamp_1(servo_pin1), m_clamp_2(servo_pin2) {}

void Clamp::loop() {
    if (auto state = m_ros.getRequestedState()) {
        if (*state < 2) {
            log(INFO, String("Moving clamps ").concat(m_level).concat(" to position ").concat(*state));
            m_clamp_1.setState(*state);
            m_clamp_2.setState(*state);
        } else {
            log(WARN, String("Invalid order received for clamps ").concat(m_level));
        }
        m_ros.clearRequestedState();
    } else if (m_clamp_1.needsNotify() && micros() - m_clamp_1.getLastChangeTime() > CALLBACK_INTERVAL) {
        m_ros.sendCallback(m_clamp_1.getState());
        m_clamp_1.markNotified();
    }
}

Clamp1::Clamp1(ros2::Node &node) : Clamp(CLAMP_1_1_PIN, CLAMP_1_2_PIN, node, 1, "/act/order/clamp_1", "/act/callback/clamp_1") {}

Clamp2::Clamp2(ros2::Node &node) : Clamp(CLAMP_2_1_PIN, CLAMP_2_2_PIN, node, 2, "/act/order/clamp_2", "/act/callback/clamp_2") {}