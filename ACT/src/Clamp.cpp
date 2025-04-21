#include "Clamp.hpp"

#include "configuration.hpp"
#include "logging.hpp"

// TODO what should be the initial state?

ClampServo::ClampServo(int servo_pin, std::array<int, 2> positions) : ActuatorServo(servo_pin, positions, /* initial_state = */ OPEN) {}

Clamps::Clamps(ClampServo servo1, ClampServo servo2, ros2::Node &node, int level, const char *order_topic, const char *callback_topic)
    : m_level(level), m_ros(node, order_topic, callback_topic), m_clamp_1(servo1), m_clamp_2(servo2) {}

void Clamps::loop() {
    if (auto state = m_ros.getRequestedState()) {
        if (*state < 2) {
            log(INFO, String("Moving clamps ").concat(m_level).concat(" to position ").concat(*state));
            m_clamp_1.setState(*state);
            m_clamp_2.setState(*state);
        } else {
            log(WARN, String("Invalid order received for clamps ").concat(m_level));
        }
        m_ros.clearRequestedState();
    } else if (m_clamp_1.needsNotify() && millis() - m_clamp_1.getLastChangeTime() > CALLBACK_INTERVAL) {
        m_ros.sendCallback(m_clamp_1.getState());
        m_clamp_1.markNotified();
    }
}

ClampServo1_1::ClampServo1_1() : ClampServo(CLAMP_1_1_PIN, std::array{CLAMP_1_1_CLOSED_POS, CLAMP_1_1_OPEN_POS}) {}

ClampServo1_2::ClampServo1_2() : ClampServo(CLAMP_1_2_PIN, std::array{CLAMP_1_2_CLOSED_POS, CLAMP_1_2_OPEN_POS}) {}

ClampServo2_1::ClampServo2_1() : ClampServo(CLAMP_2_1_PIN, std::array{CLAMP_2_1_CLOSED_POS, CLAMP_2_1_OPEN_POS}) {}

ClampServo2_2::ClampServo2_2() : ClampServo(CLAMP_2_2_PIN, std::array{CLAMP_2_2_CLOSED_POS, CLAMP_2_2_OPEN_POS}) {}

Clamps1::Clamps1(ros2::Node &node) : Clamps(ClampServo1_1(), ClampServo1_2(), node, 1, "/act/order/clamp_1", "/act/callback/clamp_1") {}

Clamps2::Clamps2(ros2::Node &node) : Clamps(ClampServo2_1(), ClampServo2_2(), node, 2, "/act/order/clamp_2", "/act/callback/clamp_2") {}