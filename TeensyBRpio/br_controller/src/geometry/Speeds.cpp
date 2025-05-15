#include "geometry/Speeds.hpp"
#include "geometry/WheelSpeeds.hpp"

Speeds::Speeds(number_t linear, number_t angular) : linear(linear), angular(angular) {}

Speeds Speeds::operator+(Speeds other) const {
    return Speeds(linear + other.linear, angular + other.angular);
}
Speeds Speeds::operator-(Speeds other) const {
    return Speeds(linear - other.linear, angular - other.angular);
}
Speeds Speeds::operator*(number_t factor) const {
    return Speeds(linear * factor, angular * factor);
}
Speeds Speeds::operator/(number_t factor) const {
    return Speeds(linear / factor, angular / factor);
}

Speeds Speeds::operator-() const {
    return Speeds(-linear, -angular);
}

WheelSpeeds Speeds::toWheelSpeeds(number_t wheelRadius, number_t wheelDistance) const {
    number_t rightWheelSpeed = linear + angular * wheelDistance / 2;
    number_t leftWheelSpeed = linear - angular * wheelDistance / 2;
    return WheelSpeeds(leftWheelSpeed, rightWheelSpeed) / wheelRadius;
}

template <>
Speeds clamp<Speeds>(Speeds speeds, number_t minBound, number_t maxBound) {
    return Speeds(std::clamp(speeds.linear, minBound, maxBound), speeds.angular);
}