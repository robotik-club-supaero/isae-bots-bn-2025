#ifndef _BANNER_HPP_
#define _BANNER_HPP_

#include "ActuatorServo.hpp"
#include "ActuatorState.hpp"

enum BannerOrder : uint16_t { DEPLOY = 1 };

enum BannerCallback : uint16_t { RETRACTED = 0, DEPLOYED = 1 };

/// An `ActuatorServo` specialized for the banner. See `ActuatorServo`. 
class BannerServo : public ActuatorServo<2> {
   public:
    BannerServo(int servo_pin, std::array<int, 2> positions);
};

/// A `BannerServo` that uses the pin and positions of BANNER_1 (see `BannerServo` and configuration.hpp).
class BannerServo1 : public BannerServo {
   public:
    BannerServo1();
};

/// A `BannerServo` that uses the pin and positions of BANNER_2 (see `BannerServo` and configuration.hpp).
class BannerServo2 : public BannerServo {
   public:
    BannerServo2();
};

/// Connects ROS and the servos of the banner. Don't forget to call `loop`.
class Banner {
   public:
    Banner(ros2::Node &node);

    void loop();
    void reset();

   private:
    BannerServo1 m_servo_1;
    BannerServo2 m_servo_2;
    ActuatorStateManager m_ros;
};

#endif