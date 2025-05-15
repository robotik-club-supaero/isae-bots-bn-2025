#include "geometry/WheelSpeeds.hpp"
#include "geometry/Speeds.hpp"

WheelSpeeds::WheelSpeeds(number_t left, number_t right) : left(left), right(right) {}

WheelSpeeds WheelSpeeds::operator+(WheelSpeeds other) const {
    return WheelSpeeds(left + other.left, right + other.right);
}
WheelSpeeds WheelSpeeds::operator-(WheelSpeeds other) const {
    return WheelSpeeds(left - other.left, right - other.right);
}
WheelSpeeds WheelSpeeds::operator*(number_t factor) const {
    return WheelSpeeds(left * factor, right * factor);
}
WheelSpeeds WheelSpeeds::operator/(number_t factor) const {
    return WheelSpeeds(left / factor, right / factor);
}

WheelSpeeds WheelSpeeds::operator-() const {
    return WheelSpeeds(-left, -right);
}

Speeds WheelSpeeds::toUnicycleSpeeds(number_t wheelRadius, number_t wheelDistance) const {
    number_t angular = (right - left) * wheelRadius / wheelDistance;
    number_t linear = (left + right) * wheelRadius / 2;
    return Speeds(linear, angular);
}

template <>
WheelSpeeds clamp<WheelSpeeds>(WheelSpeeds speeds, number_t minBound, number_t maxBound) {
    return WheelSpeeds(std::clamp(speeds.left, minBound, maxBound), std::clamp(speeds.right, minBound, maxBound));
}