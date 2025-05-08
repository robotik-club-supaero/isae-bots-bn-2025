#ifndef _SPEEDS_HPP_
#define _SPEEDS_HPP_

#include "defines/math.hpp"
#include "defines/string.hpp"

// TODO: rename class Speeds to a name that can apply to Accelerations and Tolerances as well, and
// create an alias for Speeds.

class WheelSpeeds;

/**
 * A pair of a linear speed (or acceleration) and an angular speed (or acceleration). Whether signed or absolute values are
 * expected must be specified by each use case.
 */
class Speeds {
  public:
    Speeds() = default;
    Speeds(double_t linear, double_t angular);

    bool operator==(const Speeds &other) const = default;
    Speeds operator+(Speeds other) const;
    Speeds operator-(Speeds other) const;
    Speeds operator*(double_t factor) const;
    Speeds operator/(double_t factor) const;

    Speeds operator-() const;

    /// Convert from (linear, angular) to (left wheel, right wheel)
    WheelSpeeds toWheelSpeeds(double_t wheelRadius, double_t wheelDistance) const;

    /// (Maybe signed) linear speed or acceleration (m/s or m/s²)
    double_t linear;
    /// (Maybe signed) angular speed or acceleration (rad/s or rad/s²)
    double_t angular;

#ifdef _STRING_EXT_
    operator std::string() const { return "(" + to_string(linear) + ", " + to_string(angular) + ")"; }
#endif
};

using Accelerations = Speeds;

#endif