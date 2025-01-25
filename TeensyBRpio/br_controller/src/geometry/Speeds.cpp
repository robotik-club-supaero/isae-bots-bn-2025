#include "geometry/Speeds.hpp"

Speeds::Speeds(double_t linear, double_t angular) : linear(linear), angular(angular) {}

Speeds Speeds::operator+(Speeds other) const {
    return Speeds(linear + other.linear, angular + other.angular);
}
Speeds Speeds::operator-(Speeds other) const {
    return Speeds(linear - other.linear, angular - other.angular);
}
Speeds Speeds::operator*(double_t factor) const {
    return Speeds(linear * factor, angular * factor);
}
Speeds Speeds::operator/(double_t factor) const {
    return Speeds(linear / factor, angular / factor);
}

Speeds Speeds::operator-() const {
    return Speeds(-linear, -angular);
}

WheelSpeeds Speeds::toWheelSpeeds(double_t wheelRadius, double_t wheelDistance) const {
    double_t rightWheelSpeed = linear + angular * wheelDistance / 2;
    double_t leftWheelSpeed = linear - angular * wheelDistance / 2;
    return WheelSpeeds(leftWheelSpeed, rightWheelSpeed) / wheelRadius;
}

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

Speeds WheelSpeeds::toUnicycleSpeeds(double_t wheelRadius, double_t wheelDistance) const {
    double angular = (right - left) * wheelRadius / wheelDistance;
    double linear = (left + right) * wheelRadius / 2;
    return Speeds(linear, angular);
}

WheelSpeeds clamp(WheelSpeeds speeds, double_t minBound, double_t maxBound) {
    return WheelSpeeds(std::clamp(speeds.left, minBound, maxBound), std::clamp(speeds.right, minBound, maxBound));
}