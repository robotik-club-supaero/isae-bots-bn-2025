#ifndef _WHEEL_SPEEDS_HPP_
#define _WHEEL_SPEEDS_HPP_

#include "defines/math.hpp"

class Speeds;

class WheelSpeeds {
  public:
    WheelSpeeds(number_t left, number_t right);

    bool operator==(const WheelSpeeds &other) const = default;
    WheelSpeeds operator+(WheelSpeeds other) const;
    WheelSpeeds operator-(WheelSpeeds other) const;
    WheelSpeeds operator*(number_t factor) const;
    WheelSpeeds operator/(number_t factor) const;

    WheelSpeeds operator-() const;

    /// Convert from (left wheel, right wheel) to (linear, angular)
    Speeds toUnicycleSpeeds(number_t wheelRadius, number_t wheelDistance) const;

    /// (Maybe signed) angular speed of the left wheel (rad/s)
    number_t left;
    /// (Maybe signed) angular speed of the right wheel (rad/s)
    number_t right;
};

inline WheelSpeeds operator*(number_t factor, WheelSpeeds speeds) {
    return speeds * factor;
}

#endif