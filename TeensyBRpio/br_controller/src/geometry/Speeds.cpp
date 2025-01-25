#include "geometry/Speeds.hpp"
#include "geometry/WheelSpeeds.hpp"

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

template <>
Speeds clamp<Speeds>(Speeds speeds, double_t minBound, double_t maxBound) {
    return Speeds(std::clamp(speeds.linear, minBound, maxBound), speeds.angular);
}