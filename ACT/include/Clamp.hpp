#ifndef _CLAMP_HPP_
#define _CLAMP_HPP_

#include "ActuatorServo.hpp"
#include "ActuatorState.hpp"

enum ClampOrder : uint16_t { ORDER_OPEN = 0, CLOSE = 1 };

enum ClampCallback : uint16_t { OPEN = 0, CLOSED = 1 };

class ClampServo : public ActuatorServo<2> {
   public:
    ClampServo(int servo_pin);
};

class Clamps {
   public:
    Clamps(int servo_pin1, int servo_pin2, ros2::Node &node, int level, const char *order_topic, const char *callback_topic);

    void loop();

   private:
    int m_level;
    ActuatorStateManager m_ros;
    ClampServo m_clamp_1;
    ClampServo m_clamp_2;
};

class Clamps1 : public Clamps {
   public:
    Clamps1(ros2::Node &node);
};

class Clamps2 : public Clamps {
   public:
    Clamps2(ros2::Node &node);
};

#endif