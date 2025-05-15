#ifndef _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_
#define _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "controller/DisplacementKind.hpp"
#include "math/Ramp.hpp"
#include "trajectories/Trajectory.hpp"

class Trajectory;

namespace controller {

/**
 * Makes the robot follow a smooth trajectory. This expects the robot to be heading in the initial direction of the trajectory, and
 * stops at points of discontinuity.
 *
 * @see Trajectory
 */
class StateSmoothTrajectory {
  public:
    /// @param trajectory must not be null and must be valid for the lifetime of this state
    StateSmoothTrajectory(DisplacementKind kind, Trajectory *trajectory, Speeds maxSpeeds, number_t maxLinAcceleration);
    ControllerStatus getStatus() const;
    StateUpdateResult update(number_t interval);
    void setMaxSpeeds(Speeds maxSpeeds);

  private:
    DisplacementKind m_kind;
    Trajectory *m_trajectory;
    Angle m_lastDirection;

    Speeds m_maxSpeeds;
    Ramp m_ramp;
};
} // namespace controller

#endif