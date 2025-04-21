#ifndef _CLAMP_HPP_
#define _CLAMP_HPP_

#include "ActuatorServo.hpp"
#include "ActuatorState.hpp"

enum ClampOrder : uint16_t { ORDER_OPEN = 0, CLOSE = 1 };

enum ClampCallback : uint16_t { OPEN = 0, CLOSED = 1 };

class ClampServo : public ActuatorServo<2> {
   public:
    ClampServo(int servo_pin, std::array<int, 2> positions);
};

class Clamps {
   public:
    Clamps(ClampServo servo1, ClampServo servo2, ros2::Node &node, int level, const char *order_topic, const char *callback_topic);

    void loop();

   private:
    int m_level;
    ActuatorStateManager m_ros;
    ClampServo m_clamp_1;
    ClampServo m_clamp_2;
};

class ClampServo1_1 : public ClampServo {
   public:
    ClampServo1_1();
};
class ClampServo1_2 : public ClampServo {
   public:
    ClampServo1_2();
};
class ClampServo2_1 : public ClampServo {
   public:
    ClampServo2_1();
};
class ClampServo2_2 : public ClampServo {
   public:
    ClampServo2_2();
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