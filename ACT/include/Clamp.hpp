#ifndef _CLAMP_HPP_
#define _CLAMP_HPP_

#include "Actuator.hpp"

enum ClampOrder : uint16_t { ORDER_OPEN = 0, CLOSE = 1 };

enum ClampCallback : uint16_t { OPEN = 0, CLOSED = 1 };

class ClampHandle : public ActuatorHandle<2> {
   public:
    ClampHandle(int servo_pin, const char *name);
};

class Clamp {
   public:
    Clamp(int servo_pin, ros2::Node &node, const char *name, const char *order_topic, const char *callback_topic);

    void loop();

   private:
    std::shared_ptr<ClampHandle> m_handle;
    ActuatorTopics m_ros;
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