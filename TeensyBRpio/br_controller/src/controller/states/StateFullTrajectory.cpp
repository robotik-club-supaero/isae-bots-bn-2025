#include "controller/states/StateFullTrajectory.hpp"
#include "controller/states/StateRotation.hpp"
#include "controller/states/StateSmoothTrajectory.hpp"
#include "defines/func.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/LinearTrajectory.hpp"

namespace controller {
StateFullTrajectory::StateFullTrajectory(DisplacementKind kind, Trajectory *trajectory, Arena::rotation_ptr &rotation_buffer,
                                         Angle initialOrientation, Speeds maxSpeeds, Accelerations maxAccelerations)
    : m_kind(kind), m_trajectory(trajectory), m_rotation_buffer(rotation_buffer), m_maxSpeeds(maxSpeeds), m_maxAccelerations(maxAccelerations) {
    startInitialRotation(initialOrientation);
}

ControllerStatus StateFullTrajectory::getStatus() const {
    return getStateStatus<ControllerStatus>() | ControllerStatus::TRAJECTORY;
}

StateUpdateResult StateFullTrajectory::update(double_t interval) {
    StateUpdateResult result = updateState<StateUpdateResult>(interval);
    if (std::holds_alternative<RotationComplete>(result)) {
        setCurrentState<StateSmoothTrajectory>(m_kind, m_trajectory, m_maxSpeeds, m_maxAccelerations.linear);
        return PositionControl();
    } else if (std::holds_alternative<TrajectoryComplete>(result)) {
        Angle currentOrientation = m_trajectory->getCurrentPosition().theta;
        if (m_trajectory->advance(0)) {
            startInitialRotation(currentOrientation);
            return PositionControl();
        } else {
            return TrajectoryComplete();
        }
    } else {
        return result;
    }
}

void StateFullTrajectory::startInitialRotation(Angle initialRobotOrientation) {
    m_rotation_buffer.emplace<SetHeadingProfile>(initialRobotOrientation, m_trajectory->getCurrentPosition().theta + m_kind.getAlignmentOffset());
    setCurrentState<StateInitialRotation>(m_rotation_buffer.get(), m_maxSpeeds.angular, m_maxAccelerations.angular);
}
void StateFullTrajectory::setMaxSpeeds(Speeds maxSpeeds) {
    m_maxSpeeds = maxSpeeds;
    visitCurrentState<void>(overload{[&](StateInitialRotation &state) { state.setMaxSpeed(maxSpeeds.angular); },
                                     [&](StateSmoothTrajectory &state) { state.setMaxSpeeds(maxSpeeds); }, [](auto &state) {}});
}

} // namespace controller