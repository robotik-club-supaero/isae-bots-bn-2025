#ifndef _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_
#define _CONTROLLER_STATE_DISPLACEMENT_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "controller/DisplacementKind.hpp"
#include "math/Ramp.hpp"
#include "trajectories/Trajectory.hpp"

#include <memory>

class Trajectory;

namespace controller {

class StateTrajectory : public ControllerState {
  public:
    /// @param trajectory must not be null
    StateTrajectory(DisplacementKind kind, std::shared_ptr<Trajectory> trajectory, Speeds maxSpeeds, double_t maxLinAcceleration);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    void notify(ControllerEvent event) override;
    
  private:
    DisplacementKind m_kind;
    std::shared_ptr<Trajectory> m_trajectory;
    Angle m_lastDirection;

    Speeds m_maxSpeeds;
    Ramp m_ramp;
};
} // namespace controller

#endif