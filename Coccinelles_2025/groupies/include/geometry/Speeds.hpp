#ifndef _TRAJECTORY_SPEEDS_HPP_
#define _TRAJECTORY_SPEEDS_HPP_

#include "defines/math.hpp"

// TODO: rename class Speeds to a name that can apply to Accelerations and Tolerances as well, and
// create an alias for Speeds.

class WheelSpeeds;

/**
 * A pair of a linear speed (or acceleration or distance) and an angular speed (or acceleration or distance). Whether signed or absolute
 * values are expected must be specified by each use case.
 */
class Speeds {
  public:
    Speeds() = default;
    Speeds(double_t linear, double_t angular);

    Speeds operator+(Speeds other) const;
    Speeds operator-(Speeds other) const;
    Speeds operator*(double_t factor) const;
    Speeds operator/(double_t factor) const;

    /// Convert from (linear, angular) to (left wheel, right wheel)
    WheelSpeeds toWheelSpeeds(double_t wheelRadius, double_t wheelDistance) const;

    /// (Maybe signed) linear speed or acceleration (m/s or m/s²)
    double_t linear;
    /// (Maybe signed) angular speed or acceleration (rad/s or rad/s²)
    double_t angular;
};

class WheelSpeeds {
  public:
    WheelSpeeds(double_t left, double_t right);

    WheelSpeeds operator+(WheelSpeeds other) const;
    WheelSpeeds operator-(WheelSpeeds other) const;
    WheelSpeeds operator*(double_t factor) const;
    WheelSpeeds operator/(double_t factor) const;

    /// Convert from (left wheel, right wheel) to (linear, angular)
    Speeds toUnicycleSpeeds(double_t wheelRadius, double_t wheelDistance) const;

    /// (Maybe signed) angular speed of the left wheel (rad/s)
    double_t left;
    /// (Maybe signed) angular speed of the right wheel (rad/s)
    double_t right;
};

WheelSpeeds clamp(WheelSpeeds speeds, double_t minBound, double_t maxBound);

/// See class Speeds
using Accelerations = Speeds;

/// See class Speeds
using Tolerances = Speeds;

#endif