#include "Clamp.hpp"

#include "configuration.hpp"

// TODO what should be the initial state?

Clamp::Clamp(int servo_pin, ros2::Node &node, const char *order_topic, const char *callback_topic)
    : Actuator(servo_pin, node, order_topic, callback_topic, std::array{CLAMP_CLOSED_POS, CLAMP_OPEN_POS}, /* initial_state = */ OPEN) {}

Clamp1::Clamp1(ros2::Node &node) : Clamp(CLAMP_1_PIN, node, "/act/order/clamp_1", "/act/callback/clamp_1") {}

Clamp2::Clamp2(ros2::Node &node) : Clamp(CLAMP_2_PIN, node, "/act/order/clamp_2", "/act/callback/clamp_2") {}