#ifndef _UNICYCLE_STATE_SIMULATOR_HPP_
#define _UNICYCLE_STATE_SIMULATOR_HPP_

#include "configuration.hpp"
#include "geometry/Position2D.hpp"
#include "geometry/Speeds.hpp"
#include "motors/MotorStub.hpp"

#include <memory>
#include <optional>
#include <random>

/// See concept PositionFeedback.
class UnicycleStateSimulator {
  public:
    /**
     * This simulator has been deprecated in favor of `TwoWheelSimulator`.
     * 
     * @param noise_stddev Standard deviation for the simulation noise. Must be positive, or zero to disable the noise.
     * Currently, the noise follows a centered normal distribution and is added to both the linear and the angular speeds.
     * The standard deviation is the same for both Speeds (which may not be physically homogeneous).
     * TODO: improve how the noise is implemented
     */
    UnicycleStateSimulator(number_t noise_stddev);
    UnicycleStateSimulator() : UnicycleStateSimulator(NOISE_STD_DEV) {}

    void setSpeeds(Speeds speeds);
    void resetPosition(Position2D<Meter> pos);
    void update(number_t interval);

    Position2D<Meter> getRobotPosition() const;
    MotorStub createMotorStub() const;

  private:
    class Noise {
      public:
        Noise(number_t standardDeviation);
        number_t operator()();

      private:
        std::optional<std::normal_distribution<number_t>> m_noise;
    };

    std::shared_ptr<Speeds> m_speeds;
    Position2D<Meter> m_position;
    Noise m_noise;
};

#endif