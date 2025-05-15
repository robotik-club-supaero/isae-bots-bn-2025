#include "motors/MotorsOdrive2.hpp"
#include "geometry/WheelSpeeds.hpp"
#include <ODriveEnums.h>

#include <algorithm>
#include <numbers>

// defines to avoid wondering if 0 is left or right
#define BR_RIGHT 0
#define BR_LEFT 1

MotorsOdrive2::MotorsOdrive2(uint8_t odriveRxPin, uint8_t odriveTxPin, number_t transmissionRatio, number_t wheelDiameter, number_t wheelDistance,
                             number_t maxMotorSpeed)
    : m_serial(std::make_unique<SoftwareSerial>(odriveRxPin, odriveTxPin)), m_odrive(*m_serial),
      m_conversionFactor(transmissionRatio / (2 * std::numbers::pi_v<number_t>)), m_wheelRadius(wheelDiameter / 2), m_wheelDistance(wheelDistance),
      m_maxMotorSpeed(maxMotorSpeed) {
    m_serial->begin(115200);
}

void MotorsOdrive2::sendCommand(Speeds speeds) {
    WheelSpeeds wheelSpeeds = speeds.toWheelSpeeds(m_wheelRadius, m_wheelDistance) * m_conversionFactor;

#ifdef _BR_DEBUG
    m_lastLeftSpeed = -wheelSpeeds.left;
    m_lastRightSpeed = wheelSpeeds.right;
#endif

    sendCommand(BR_LEFT, -wheelSpeeds.left);
    sendCommand(BR_RIGHT, wheelSpeeds.right);
}

void MotorsOdrive2::sendCommand(int motor_number, number_t velCmd) {
    // constrain the motor command for safety
    m_odrive.SetVelocity(motor_number, std::clamp(velCmd, -m_maxMotorSpeed, m_maxMotorSpeed));
}

void MotorsOdrive2::switchOn() {
    // NOTE on n'attend pas le retour de l'Odrive
    m_odrive.run_state(BR_RIGHT, AXIS_STATE_CLOSED_LOOP_CONTROL, false, 0.0);
    m_odrive.run_state(BR_LEFT, AXIS_STATE_CLOSED_LOOP_CONTROL, false, 0.0);
}

void MotorsOdrive2::switchOff() {
    m_odrive.run_state(BR_RIGHT, AXIS_STATE_IDLE, false, 0.0);
    m_odrive.run_state(BR_LEFT, AXIS_STATE_IDLE, false, 0.0);
}

void MotorsOdrive2::update(number_t interval) {
    // Nothing to do
}

bool MotorsOdrive2::isReady() {
    return m_odrive.getCurrentAxisState(BR_LEFT) == AXIS_STATE_CLOSED_LOOP_CONTROL &&
           m_odrive.getCurrentAxisState(BR_RIGHT) == AXIS_STATE_CLOSED_LOOP_CONTROL;
}
bool MotorsOdrive2::isIdle() {
    return m_odrive.getCurrentAxisState(BR_LEFT) == AXIS_STATE_IDLE && m_odrive.getCurrentAxisState(BR_RIGHT) == AXIS_STATE_IDLE;
}

#ifdef _BR_DEBUG
number_t MotorsOdrive2::getLastLeftSpeed() const {
    return m_lastLeftSpeed;
}
number_t MotorsOdrive2::getLastRightSpeed() const {
    return m_lastRightSpeed;
}
#endif