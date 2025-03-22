#include "controller/states/StateSmoothTrajectory.hpp"
#include "defines/func.hpp"
#include "logging.hpp"

namespace controller {

StateSmoothTrajectory::StateSmoothTrajectory(DisplacementKind kind, std::shared_ptr<Trajectory> trajectory, Speeds maxSpeeds, double_t maxLinAcceleration)
    : m_kind(kind), m_trajectory(std::move(trajectory)), m_lastDirection(m_trajectory->getCurrentPosition().theta), m_maxSpeeds(maxSpeeds),
      m_ramp(maxSpeeds.linear, maxLinAcceleration) {
    log(INFO, "Entering controller state: Moving");
}

ControllerStatus StateSmoothTrajectory::getStatus() const {
    if (m_kind == FORWARD) {
        return ControllerStatus::MOVING;
    } else {
        return ControllerStatus::REVERSE;
    }
}

StateUpdateResult StateSmoothTrajectory::update(double_t interval) {
    if (m_trajectory) {
        m_ramp.update(interval);
        double_t currentRampSpeed = m_ramp.getCurrentSpeed();

        if (m_trajectory->advanceSmooth(currentRampSpeed * interval)) {           
            m_ramp.setTargetSpeed(m_maxSpeeds.linear);
            std::optional<double_t> remainingDistance = m_trajectory->getRemainingDistance();
            if (remainingDistance) {
                m_ramp.ensureCanBrake(*remainingDistance);
            }

            Position2D<Meter> position = m_trajectory->getCurrentPosition();
            double_t angularSpeed = abs(position.theta - m_lastDirection) / interval;

            // V_lin = R V_ang; curvature = 1 / R
            double_t actualCurvature = currentRampSpeed == 0 ? 0 : angularSpeed / currentRampSpeed;

            double_t distanceToCheck = m_ramp.getBrakingDistance();
            while (distanceToCheck) {
                double_t maxCurvature = std::max(actualCurvature, m_trajectory->getMaxCurvature(distanceToCheck));
                if (maxCurvature == 0) {
                    break;
                }
                double_t maxSpeedForCurvature = m_maxSpeeds.angular / maxCurvature;
                if (maxSpeedForCurvature >= m_ramp.getTargetSpeed()) {
                    break;
                }
                double_t distanceToAdaptSpeed = m_ramp.getBrakingDistance(maxSpeedForCurvature);
                if (distanceToCheck > distanceToAdaptSpeed) {
                    distanceToCheck = distanceToAdaptSpeed;
                } else {
                    m_ramp.setTargetSpeed(maxSpeedForCurvature);
                    break;
                }
            }

            m_lastDirection = position.theta;
            position.theta += m_kind.getAlignmentOffset();
            return PositionControl(position);
        } else {
            m_trajectory.reset();
        }
    }
    return TrajectoryComplete();
}

void StateSmoothTrajectory::notify(ControllerEvent event) {
    std::visit(overload{[&](const MaxSpeedsChanged &event) { m_maxSpeeds = event.newSpeeds; }, [](auto) {}}, event);
}

} // namespace controller