#ifndef _CLAMP_HPP_
#define _CLAMP_HPP_

#include "Actuator.hpp"

enum ClampOrder : uint16_t { ORDER_OPEN = 0, CLOSE = 1 };

enum ClampCallback : uint16_t { OPEN = 0, CLOSED = 1 };

class Clamp : public Actuator<2> {
   public:
    Clamp(int servo_pin, ros2::Node &node, const char *order_topic, const char *callback_topic);
};

class Clamp1 : public Clamp {
   public:
    Clamp1(ros2::Node &noe);
};

class Clamp2 : public Clamp {
   public:
    Clamp2(ros2::Node &node);
};

#endif