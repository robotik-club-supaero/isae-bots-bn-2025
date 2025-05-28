#ifndef _CLAMP_HPP_
#define _CLAMP_HPP_

#include "ActuatorServo.hpp"
#include "ActuatorState.hpp"

enum ClampOrder : uint16_t { ORDER_OPEN = 0, CLOSE = 1 };

enum ClampCallback : uint16_t { OPEN = 0, CLOSED = 1 };

/// An `ActuatorServo` specialized for the clamp. See `ActuatorServo`.
class ClampServo : public ActuatorServo<2> {
   public:
    ClampServo(int servo_pin, std::array<int, 2> positions);
};

/// Connects ROS the the servos of the clamps for one level. Don't forget to call `loop`.
class Clamps {
   public:
    /// Instantiate class `Clamps1` or `Clamps2` instead.
    Clamps(ClampServo servo1, ClampServo servo2, ros2::Node &node, int level, const char *order_topic, const char *callback_topic);

    void loop();
    void reset();

   private:
    int m_level;
    ActuatorStateManager m_ros;
    ClampServo m_clamp_1;
    ClampServo m_clamp_2;
};

/// A `ClampServo` that uses the pin and positions of CLAMP_1_1 (clamp 1 of level 1). See `configuration.hpp`
class ClampServo1_1 : public ClampServo {
   public:
    ClampServo1_1();
};

/// A `ClampServo` that uses the pin and positions of CLAMP_1_2 (clamp 2 of level 1). See `configuration.hpp`
class ClampServo1_2 : public ClampServo {
   public:
    ClampServo1_2();
};

/// A `ClampServo` that uses the pin and positions of CLAMP_2_1 (clamp 1 of level 2). See `configuration.hpp`
class ClampServo2_1 : public ClampServo {
   public:
    ClampServo2_1();
};

/// A `ClampServo` that uses the pin and positions of CLAMP_2_2 (clamp 2 of level 2). See `configuration.hpp`
class ClampServo2_2 : public ClampServo {
   public:
    ClampServo2_2();
};

/// Connects ROS to the servos of the first level of clamps. Don't forget to call `loop`.
class Clamps1 : public Clamps {
   public:
    Clamps1(ros2::Node &node);
};

/// Connects ROS to the servos of the second level of clamps. Don't forget to call `loop`.
class Clamps2 : public Clamps {
   public:
    Clamps2(ros2::Node &node);
};

#endif