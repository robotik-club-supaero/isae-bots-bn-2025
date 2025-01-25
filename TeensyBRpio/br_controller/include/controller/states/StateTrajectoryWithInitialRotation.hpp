#ifndef _CONTROLLER_STATE_TRAJECTORY_HPP_
#define _CONTROLLER_STATE_TRAJECTORY_HPP_

#include "controller/ControllerState.hpp"
#include "controller/DisplacementKind.hpp"
#include "fsm/StateMachine.hpp"
#include "trajectories/Trajectory.hpp"

namespace controller {

/// Manages the transition between StateInitialRotation and StateTrajectory
class StateTrajectoryWithInitialRotation : public ControllerState, private fsm::StateMachine<ControllerState> {
  public:
    /// @param trajectory must not be null
    StateTrajectoryWithInitialRotation(DisplacementKind kind, std::unique_ptr<Trajectory> trajectory, Angle initialOrientation, Speeds maxSpeeds,
                                       Accelerations maxAccelerations, std::optional<Angle> finalOrientation = std::nullopt);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    void notify(ControllerEvent event) override;
    bool resumeState(Position2D<Meter> robotPosition) override;

  private:
    void startInitialRotation(Angle initialRobotOrientation);

    DisplacementKind m_kind;
    std::shared_ptr<Trajectory> m_trajectory;
    std::optional<Angle> m_finalOrientation;

    std::shared_ptr<Trajectory> m_suspendedTrajectory;

    Speeds m_maxSpeeds;
    Accelerations m_maxAccelerations;
};

}; // namespace controller

#endif