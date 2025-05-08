#ifndef _CONTROLLER_STATE_TRAJECTORY_HPP_
#define _CONTROLLER_STATE_TRAJECTORY_HPP_

#include "controller/states/StateRotation.hpp"
#include "controller/states/StateSmoothTrajectory.hpp"
#include "fsm/StateMachine.hpp"
#include "trajectories/Trajectory.hpp"

#include <memory> //FIXME

namespace controller {

/// Makes the robot follow a trajectory. This manages the initial rotation and the rotations at points of discontinuity.
class StateFullTrajectory final : public ControllerState,
                                  private fsm::StateMachine<fsm::StateUninit, StateInitialRotation, StateSmoothTrajectory> {
  public:
    /// @param trajectory must not be null
    StateFullTrajectory(DisplacementKind kind, std::shared_ptr<Trajectory> trajectory, Angle initialOrientation, Speeds maxSpeeds,
                        Accelerations maxAccelerations);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    bool resumeState(Position2D<Meter> robotPosition) override;
    void setMaxSpeeds(Speeds maxSpeeds);

  private:
    void startInitialRotation(Angle initialRobotOrientation);

    DisplacementKind m_kind;
    std::shared_ptr<Trajectory> m_trajectory;

    Speeds m_maxSpeeds;
    Accelerations m_maxAccelerations;
};

}; // namespace controller

#endif