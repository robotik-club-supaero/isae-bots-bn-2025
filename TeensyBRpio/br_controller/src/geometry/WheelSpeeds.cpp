#include "geometry/WheelSpeeds.hpp"
#include "geometry/Speeds.hpp"

WheelSpeeds::WheelSpeeds(double_t left, double_t right) : left(left), right(right) {}

WheelSpeeds WheelSpeeds::operator+(WheelSpeeds other) const {
    return WheelSpeeds(left + other.left, right + other.right);
}
WheelSpeeds WheelSpeeds::operator-(WheelSpeeds other) const {
    return WheelSpeeds(left - other.left, right - other.right);
}
WheelSpeeds WheelSpeeds::operator*(double_t factor) const {
    return WheelSpeeds(left * factor, right * factor);
}
WheelSpeeds WheelSpeeds::operator/(double_t factor) const {
    return WheelSpeeds(left / factor, right / factor);
}

WheelSpeeds WheelSpeeds::operator-() const {
    return WheelSpeeds(-left, -right);
}

Speeds WheelSpeeds::toUnicycleSpeeds(double_t wheelRadius, double_t wheelDistance) const {
    double_t angular = (right - left) * wheelRadius / wheelDistance;
    double_t linear = (left + right) * wheelRadius / 2;
    return Speeds(linear, angular);
}

template <>
WheelSpeeds clamp<WheelSpeeds>(WheelSpeeds speeds, double_t minBound, double_t maxBound) {
    return WheelSpeeds(std::clamp(speeds.left, minBound, maxBound), std::clamp(speeds.right, minBound, maxBound));
}