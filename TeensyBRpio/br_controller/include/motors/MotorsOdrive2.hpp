#ifndef _MOTORS_ODRIVE_2_HPP_
#define _MOTORS_ODRIVE_2_HPP_

#include "configuration.hpp"
#include "defines/constraint.hpp"
#include "geometry/Speeds.hpp"

#include <ODriveArduino.h>
#include <SoftwareSerial.h>
#include <memory>

/**
 * Interface with two differential Odrive motors. Arduino only.
 * Satisfies concept Actuators.
 */
class MotorsOdrive2 {
  public:
    MotorsOdrive2(uint8_t odriveRxPin, uint8_t odriveTxPin, number_t transmissionRatio, number_t wheelDiameter, number_t wheelDistance,
                  number_t maxMotorSpeed);

    /// Initializes the motors with the default values from the configuration file.
    MotorsOdrive2() : MotorsOdrive2(ODRIVE_RX_PIN, ODRIVE_TX_PIN, TRANSMISSION_RATIO, WHEEL_DIAMETER, WHEEL_DISTANCE, MAX_MOTOR_SPEED) {}

    /// This exists to provide a common constructor with `MotorStub` (where the motor stub needs to be connected to a simulator), but here the
    /// argument is just ignored.
    template <typename T>
    MotorsOdrive2([[maybe_unused]] const T &feedback) : MotorsOdrive2() {}

    // See Actuators.hpp

    void switchOn();
    void switchOff();
    void update(number_t interval);
    bool isReady();
    bool isIdle();
    void sendCommand(Speeds speeds);

#ifdef _BR_DEBUG
    number_t getLastLeftSpeed() const;
    number_t getLastRightSpeed() const;
#endif

  private:
    void sendCommand(int motor_number, number_t velCmd);

#ifdef _BR_DEBUG
    number_t m_lastLeftSpeed;
    number_t m_lastRightSpeed;
#endif

    std::unique_ptr<SoftwareSerial> m_serial;
    ODriveArduino m_odrive;

    number_t m_conversionFactor;
    number_t m_wheelRadius;
    number_t m_wheelDistance;
    number_t m_maxMotorSpeed;
};

#endif