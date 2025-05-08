#ifndef _CONTROLLER_STATE_TRAJECTORY_HPP_
#define _CONTROLLER_STATE_TRAJECTORY_HPP_

#include "controller/states/StateRotation.hpp"
#include "controller/states/StateSmoothTrajectory.hpp"
#include "defines/stl.hpp"
#include "fsm/StateMachine.hpp"
#include "rotations/OrientationProfile.hpp"
#include "trajectories/Trajectory.hpp"

namespace controller {

/// Makes the robot follow a trajectory. This manages the initial rotation and the rotations at points of discontinuity.
class StateFullTrajectory : private fsm::StateMachine<fsm::StateUninit, StateInitialRotation, StateSmoothTrajectory> {
  public:
    /// @param trajectory must not be null
    StateFullTrajectory(DisplacementKind kind, Trajectory *trajectory, rotation_ptr &rotation_buffer, Angle initialOrientation, Speeds maxSpeeds,
                        Accelerations maxAccelerations);
    ControllerStatus getStatus() const;
    StateUpdateResult update(double_t interval);
    void setMaxSpeeds(Speeds maxSpeeds);

  private:
    void startInitialRotation(Angle initialRobotOrientation);

    DisplacementKind m_kind;
    Trajectory *m_trajectory;
    rotation_ptr &m_rotation_buffer;

    Speeds m_maxSpeeds;
    Accelerations m_maxAccelerations;
};

}; // namespace controller

#endif