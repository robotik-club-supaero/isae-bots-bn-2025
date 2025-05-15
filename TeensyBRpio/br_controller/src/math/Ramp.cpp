#include "math/Ramp.hpp"
#include "defines/math.hpp"
#include <cmath>

Ramp::Ramp(number_t targetSpeed, number_t maximalAcceleration, number_t currentSpeed)
    : m_isComplete(targetSpeed == currentSpeed), m_currentSpeed(currentSpeed), m_targetSpeed(targetSpeed), m_maxAcceleration(maximalAcceleration) {}

number_t Ramp::getCurrentSpeed() const {
    return m_currentSpeed;
}

number_t Ramp::getTargetSpeed() const {
    return m_targetSpeed;
}

number_t Ramp::getMaximalAcceleration() const {
    return m_maxAcceleration;
}

void Ramp::overwriteCurrentSpeed(number_t speed) {
    if (m_currentSpeed == speed) {
        return;
    }
    m_currentSpeed = speed;
    m_isComplete = false;
}

void Ramp::setTargetSpeed(number_t speed) {
    if (m_targetSpeed == speed) {
        return;
    }
    m_targetSpeed = speed;
    m_isComplete = false;
}

void Ramp::setMaximalAcceleration(number_t acceleration) {
    m_maxAcceleration = acceleration;
}

void Ramp::update(number_t interval) {
    if (!m_isComplete) {

        number_t maxDiff = m_maxAcceleration * interval;
        number_t requiredDiff = m_targetSpeed - m_currentSpeed;

        if (std::abs(requiredDiff) < maxDiff) {
            m_currentSpeed = m_targetSpeed;
            m_isComplete = true;
        } else {
            m_currentSpeed += maxDiff * sign(requiredDiff);
        }
    }
}

number_t Ramp::getBrakingDistance(number_t targetSpeed) const {
    number_t delta_v = targetSpeed - m_currentSpeed;
    return std::abs(delta_v * (m_currentSpeed + delta_v / 2) / m_maxAcceleration);
}

number_t Ramp::computeSpeedFromBrakingDistance(number_t distance) const {
    // Braking distance = v**2 / (2a)
    // It must be no greater than d = the remaining distance on the trajectory: v**2 / (2a) <= d
    // v <= sqrt(2ad)

    return std::sqrt(2 * m_maxAcceleration * distance);
}

void Ramp::ensureCanBrake(number_t distance) {
    setTargetSpeed(std::min(m_targetSpeed, computeSpeedFromBrakingDistance(distance)));
}
