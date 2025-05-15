#ifndef _CONTROLLER_STATE_TRAJECTORY_HPP_
#define _CONTROLLER_STATE_TRAJECTORY_HPP_

#include "controller/Arena.hpp"
#include "controller/states/StateRotation.hpp"
#include "controller/states/StateSmoothTrajectory.hpp"
#include "rotations/OrientationProfile.hpp"
#include "stl/StateMachine.hpp"
#include "trajectories/Trajectory.hpp"

namespace controller {

/// Makes the robot follow a trajectory. This manages the initial rotation and the rotations at points of discontinuity.
class StateFullTrajectory : private fsm::StateMachine<fsm::StateUninit, StateInitialRotation, StateSmoothTrajectory> {
  public:
    /// @param trajectory must not be null and must be valid for the lifetime of this state
    /// @param rotation_buffer must be valid for the lifetime of this state. Concurrent modifications to the buffer while this state is active are
    /// undefined behavior. When this state is destroyed, the buffer is left in a valid but unspecified state.
    StateFullTrajectory(DisplacementKind kind, Trajectory *trajectory, Arena::rotation_ptr &rotation_buffer, Angle initialOrientation,
                        Speeds maxSpeeds, Accelerations maxAccelerations);
    ControllerStatus getStatus() const;
    StateUpdateResult update(number_t interval);
    void setMaxSpeeds(Speeds maxSpeeds);

  private:
    void startInitialRotation(Angle initialRobotOrientation);

    DisplacementKind m_kind;
    Trajectory *m_trajectory;
    Arena::rotation_ptr &m_rotation_buffer;

    Speeds m_maxSpeeds;
    Accelerations m_maxAccelerations;
};

}; // namespace controller

#endif