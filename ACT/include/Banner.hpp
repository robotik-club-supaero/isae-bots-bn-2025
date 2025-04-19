#ifndef _BANNER_HPP_
#define _BANNER_HPP_

#include "ActuatorServo.hpp"
#include "ActuatorState.hpp"

enum BannerOrder : uint16_t { DEPLOY = 1 };

enum BannerCallback : uint16_t { RETRACTED = 0, DEPLOYED = 1 };

class BannerServo : public ActuatorServo<2> {
   public:
    BannerServo(int servo_pin, std::array<int, 2> positions);
};

class Banner {
   public:
    Banner(ros2::Node &node);

    void loop();

   private:
    BannerServo m_servo_1;
    BannerServo m_servo_2;
    ActuatorStateManager m_ros;
};

#endif