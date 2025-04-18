#include "Elevator.hpp"

#include "configuration.hpp"

// TODO what should be the initial state?

Elevator::Elevator(int servo_pin, ros2::Node &node, const char *order_topic, const char *callback_topic)
    : Actuator(servo_pin, node, order_topic, callback_topic, std::array{ELEVATOR_DOWN_POS, ELEVATOR_UP_POS}, /* initial_state = */ DOWN) {}

Elevator1::Elevator1(ros2::Node &node) : Elevator(ELEVATOR_1_PIN, node, "/act/order/elevator_1", "/act/callback/elevator_1") {}

Elevator2::Elevator2(ros2::Node &node) : Elevator(ELEVATOR_2_PIN, node, "/act/order/elevator_2", "/act/callback/elevator_2") {}