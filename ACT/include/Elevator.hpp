#ifndef _ELEVATOR_HPP_
#define _ELEVATOR_HPP_

#include "Actuator.hpp"

enum ElevatorOrder : uint16_t { MOVE_DOWN = 0, MOVE_UP = 1 };

enum ElevatorCallback : uint16_t { DOWN = 0, UP = 1 };

class Elevator : public Actuator<2> {
   public:
    Elevator(int servo_pin, ros2::Node &node, const char *order_topic, const char *callback_topic);
};

class Elevator1 : public Elevator {
   public:
    Elevator1(ros2::Node &noe);
};

class Elevator2 : public Elevator {
   public:
    Elevator2(ros2::Node &node);
};

#endif