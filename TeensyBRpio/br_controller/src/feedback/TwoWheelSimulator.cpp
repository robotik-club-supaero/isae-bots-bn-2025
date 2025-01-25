#include "feedback/TwoWheelSimulator.hpp"

#include "Clock.hpp"

TwoWheelSimulator::TwoWheelSimulator(double_t wheelDiameter, double_t wheelDistance, double_t minWheelSpeed, double_t maxWheelSpeed,
                                     double_t maxWheelAcceleration, double_t noise_stddev)
    : m_wheelRadius(wheelDiameter / 2), m_wheelDistance(wheelDistance), m_minWheelSpeed(minWheelSpeed), m_maxWheelSpeed(maxWheelSpeed), m_noise(),
      m_requestedSpeeds(std::make_shared<Speeds>()), m_leftWheelSpeed(0, maxWheelAcceleration), m_rightWheelSpeed(0, maxWheelAcceleration),
      m_position() {
    if (noise_stddev > 0) {
        m_noise = std::normal_distribution<double_t>(0, noise_stddev);
    }
}

void TwoWheelSimulator::update(double_t interval) {
    static std::default_random_engine generator(SystemClock().micros());

    // Apply max acceleration to wheel speed (simulates damping)
    WheelSpeeds wheelSpeeds = m_requestedSpeeds->toWheelSpeeds(m_wheelRadius, m_wheelDistance);
    m_leftWheelSpeed.setTargetSpeed(wheelSpeeds.left);
    m_rightWheelSpeed.setTargetSpeed(wheelSpeeds.right);

    m_leftWheelSpeed.update(interval);
    m_rightWheelSpeed.update(interval);

    wheelSpeeds = WheelSpeeds(m_leftWheelSpeed.getCurrentSpeed(), m_rightWheelSpeed.getCurrentSpeed());

    // Apply noise
    if (m_noise) {
        wheelSpeeds = WheelSpeeds(wheelSpeeds.left * (1 + m_noise->operator()(generator)), wheelSpeeds.right * (1 + m_noise->operator()(generator)));
    }

    // Apply min and max speeds
    wheelSpeeds = clamp(wheelSpeeds, -m_maxWheelSpeed, m_maxWheelSpeed);
    if (abs(m_leftWheelSpeed.getTargetSpeed()) < m_minWheelSpeed) {
        wheelSpeeds.left = 0;
    }
    if (abs(m_rightWheelSpeed.getTargetSpeed()) < m_minWheelSpeed) {
        wheelSpeeds.right = 0;
    }

    // Update position
    Speeds speeds = wheelSpeeds.toUnicycleSpeeds(m_wheelRadius, m_wheelDistance);

    double_t linOffset = speeds.linear * interval;
    m_position.x += std::cos(m_position.theta) * linOffset;
    m_position.y += std::sin(m_position.theta) * linOffset;
    m_position.theta += Angle(speeds.angular * interval);
}

void TwoWheelSimulator::resetPosition(Position2D<Meter> position) {
    m_position = position;
    m_leftWheelSpeed.overwriteCurrentSpeed(0);
    m_rightWheelSpeed.overwriteCurrentSpeed(0);
    *m_requestedSpeeds = {0, 0};
}
Position2D<Meter> TwoWheelSimulator::getRobotPosition() const {
    return m_position;
}

MotorStub TwoWheelSimulator::createMotorStub() const {
    return MotorStub(m_requestedSpeeds);
}
