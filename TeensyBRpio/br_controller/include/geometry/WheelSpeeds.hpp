#ifndef _WHEEL_SPEEDS_HPP_
#define _WHEEL_SPEEDS_HPP_

#include "defines/math.hpp"

class Speeds;

class WheelSpeeds {
  public:
    WheelSpeeds(double_t left, double_t right);

    bool operator==(const WheelSpeeds &other) const = default;
    WheelSpeeds operator+(WheelSpeeds other) const;
    WheelSpeeds operator-(WheelSpeeds other) const;
    WheelSpeeds operator*(double_t factor) const;
    WheelSpeeds operator/(double_t factor) const;

    WheelSpeeds operator-() const;

    /// Convert from (left wheel, right wheel) to (linear, angular)
    Speeds toUnicycleSpeeds(double_t wheelRadius, double_t wheelDistance) const;

    /// (Maybe signed) angular speed of the left wheel (rad/s)
    double_t left;
    /// (Maybe signed) angular speed of the right wheel (rad/s)
    double_t right;
};

inline WheelSpeeds operator*(double_t factor, WheelSpeeds speeds) {
    return speeds * factor;
}

#endif