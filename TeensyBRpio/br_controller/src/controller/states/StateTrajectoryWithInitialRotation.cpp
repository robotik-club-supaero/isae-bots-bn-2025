#include "controller/states/StateTrajectoryWithInitialRotation.hpp"
#include "controller/states/StateRotation.hpp"
#include "controller/states/StateTrajectory.hpp"
#include "defines/func.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/LinearTrajectory.hpp"

namespace controller {
StateTrajectoryWithInitialRotation::StateTrajectoryWithInitialRotation(DisplacementKind kind, std::unique_ptr<Trajectory> trajectory,
                                                                       Angle initialOrientation, Speeds maxSpeeds, Accelerations maxAccelerations,
                                                                       std::optional<Angle> finalOrientation)
    : m_kind(kind), m_trajectory(std::move(trajectory)), m_finalOrientation(finalOrientation), m_suspendedTrajectory(), m_maxSpeeds(maxSpeeds),
      m_maxAccelerations(maxAccelerations) {
    startInitialRotation(initialOrientation);
}

ControllerStatus StateTrajectoryWithInitialRotation::getStatus() const {
    return getCurrentState().getStatus() | ControllerStatus::TRAJECTORY;
}

StateUpdateResult StateTrajectoryWithInitialRotation::update(double_t interval) {
    StateUpdateResult result = this->getCurrentState().update(interval);
    if (std::holds_alternative<RotationComplete>(result)) {
        setCurrentState<StateTrajectory>(m_kind, m_trajectory, m_maxSpeeds, m_maxAccelerations.linear);
        return PositionControl();
    } else if (std::holds_alternative<TrajectoryComplete>(result)) {
        if (m_suspendedTrajectory) {
            log(INFO, "Resuming interrupted trajectory.");
            Angle initialRobotOrientation = m_trajectory->getCurrentPosition().theta + m_kind.getAlignmentOffset();
            m_trajectory = std::move(m_suspendedTrajectory);
            startInitialRotation(initialRobotOrientation);
            return PositionControl();
        } else {
            return TrajectoryComplete(m_finalOrientation);
        }
    } else {
        return result;
    }
}

void StateTrajectoryWithInitialRotation::startInitialRotation(Angle initialRobotOrientation) {
    log(INFO, "Entering controller state: Initial rotation");
    setCurrentState<StateRotation>(
        std::make_unique<SetHeadingProfile>(initialRobotOrientation, m_trajectory->getCurrentPosition().theta + m_kind.getAlignmentOffset()),
        m_maxSpeeds.angular, m_maxAccelerations.angular);
}

void StateTrajectoryWithInitialRotation::notify(ControllerEvent event) {
    std::visit(overload{[&](const MaxSpeedsChanged &event) { m_maxSpeeds = event.newSpeeds; }, [](auto) {}}, event);
    getCurrentState().notify(event);
}

bool StateTrajectoryWithInitialRotation::resumeState(Position2D<Meter> robotPosition) {
    if (m_trajectory->recompute(robotPosition)) {
        log(INFO, "Trajectory recomputed.");
    } else {
        log(WARN, "Failed to recompute the trajectory. Catching up using a straight line instead.");
        m_suspendedTrajectory = std::move(m_trajectory);
        m_trajectory = std::make_shared<LinearTrajectory>(robotPosition, m_suspendedTrajectory->getCurrentPosition());
    }
    startInitialRotation(robotPosition.theta);
    return true;
}

} // namespace controller