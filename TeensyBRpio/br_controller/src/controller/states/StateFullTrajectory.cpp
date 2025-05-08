#include "controller/states/StateFullTrajectory.hpp"
#include "controller/states/StateRotation.hpp"
#include "controller/states/StateSmoothTrajectory.hpp"
#include "defines/func.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/LinearTrajectory.hpp"

namespace controller {
StateFullTrajectory::StateFullTrajectory(DisplacementKind kind, std::shared_ptr<Trajectory> trajectory, Angle initialOrientation, Speeds maxSpeeds,
                                         Accelerations maxAccelerations)
    : m_kind(kind), m_trajectory(std::move(trajectory)), m_maxSpeeds(maxSpeeds), m_maxAccelerations(maxAccelerations) {
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
    std::shared_ptr<OrientationProfile> profile =
        std::make_shared<SetHeadingProfile>(initialRobotOrientation, m_trajectory->getCurrentPosition().theta + m_kind.getAlignmentOffset());
    setCurrentState<StateInitialRotation>(std::move(profile), m_maxSpeeds.angular, m_maxAccelerations.angular);
}
void StateFullTrajectory::setMaxSpeeds(Speeds maxSpeeds) {
    m_maxSpeeds = maxSpeeds;
    visitCurrentState<void>(overload{[&](StateInitialRotation &state) { state.setMaxSpeed(maxSpeeds.angular); },
                                     [&](StateSmoothTrajectory &state) { state.setMaxSpeeds(maxSpeeds); }, [](auto &state) {}});
}

bool StateFullTrajectory::resumeState(Position2D<Meter> robotPosition) {
    if (m_trajectory->recompute(robotPosition)) {
        log(INFO, "Trajectory recomputed.");
        startInitialRotation(robotPosition.theta);
        return true;
    } else {
        log(WARN, "Failed to recompute the trajectory. Aborting trajectory...");
        return false;
    }
}

} // namespace controller