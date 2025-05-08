#ifndef _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_
#define _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "controller/DisplacementKind.hpp"
#include "math/Ramp.hpp"
#include "trajectories/Trajectory.hpp"

#include <memory> //FIXME

class Trajectory;

namespace controller {

/**
 * Makes the robot follow a smooth trajectory. This expects the robot to be heading in the initial direction of the trajectory, and
 * stops at points of discontinuity.
 *
 * @see Trajectory
 */
class StateSmoothTrajectory final : public ControllerState {
  public:
    /// @param trajectory must not be null
    StateSmoothTrajectory(DisplacementKind kind, std::shared_ptr<Trajectory> trajectory, Speeds maxSpeeds, double_t maxLinAcceleration);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    void setMaxSpeeds(Speeds maxSpeeds);

  private:
    DisplacementKind m_kind;
    std::shared_ptr<Trajectory> m_trajectory;
    Angle m_lastDirection;

    Speeds m_maxSpeeds;
    Ramp m_ramp;
};
} // namespace controller

#endif