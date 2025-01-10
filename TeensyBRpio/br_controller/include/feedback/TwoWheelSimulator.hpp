#ifndef _TWO_WHEEL_SIMULATOR_HPP_
#define _TWO_WHEEL_SIMULATOR_HPP_

#include "configuration.hpp"
#include "geometry/Position2D.hpp"
#include "geometry/Speeds.hpp"
#include "math/Ramp.hpp"
#include "motors/MotorStub.hpp"

#include <memory>
#include <optional>
#include <random>

/**
 * State simulator for a differential wheeled robot
 * See concept PositionFeedback.
 */
class TwoWheelSimulator {
  public:
    /**
     * @param wheelDiameter
     * @param wheelDistance
     * @param minWheelSpeed Simulates static friction
     * @param maxWheelSpeed Simulates motor saturation
     * @param maxWheelAcceleration Simulates damping
     * @param noise_stddev Standard deviation of the noise added to the angular speed of the virtual wheels.
     */
    TwoWheelSimulator(double_t wheelDiameter, double_t wheelDistance, double_t minWheelSpeed, double_t maxWheelSpeed, double_t maxWheelAcceleration,
                      double_t noise_stddev = 0);

    TwoWheelSimulator()
        : TwoWheelSimulator(WHEEL_DIAMETER, WHEEL_DISTANCE, SIM_MIN_WHEEL_SPEED, SIM_MAX_WHEEL_SPEED, SIM_MAX_WHEEL_ACCEL, NOISE_STD_DEV) {}

    void update(double_t interval);
    void resetPosition(Position2D<Millimeter> position);
    Position2D<Meter> getRobotPosition() const;

    MotorStub createMotorStub() const;

  private:
    double_t m_wheelRadius;
    double_t m_wheelDistance;

    double_t m_minWheelSpeed;
    double_t m_maxWheelSpeed;
    std::optional<std::normal_distribution<double_t>> m_noise;

    std::shared_ptr<Speeds> m_requestedSpeeds;
    Ramp m_leftWheelSpeed;
    Ramp m_rightWheelSpeed;
    Position2D<Meter> m_position;
};

#endif