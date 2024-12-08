#include "controller/states/StateTrajectoryWithRamp.hpp"
#include "defines/func.hpp"
#include "logging.hpp"

namespace controller {

StateTrajectoryWithRamp::StateTrajectoryWithRamp(std::unique_ptr<Trajectory> trajectory, Speeds maxSpeeds, double_t maxLinAcceleration,
                                                 DisplacementKind kind, std::optional<Angle> finalOrientation)
    : m_trajectory(std::move(trajectory)), m_maxSpeeds(maxSpeeds), m_ramp(maxSpeeds.linear, maxLinAcceleration), m_kind(kind),
      m_finalOrientation(finalOrientation) {
    log(INFO, "Entering controller state: Forward");
}

ControllerStatus StateTrajectoryWithRamp::getStatus() const {
    if (m_kind == FORWARD) {
        return Forward;
    } else {
        return Reversing;
    }
}

StateUpdateResult StateTrajectoryWithRamp::update(double_t interval, Position2D<Meter> &setpoint, Position2D<Meter> actualRobotPosition) {
    if (m_trajectory) {
        m_ramp.update(interval);
        double_t currentRampSpeed = m_ramp.getCurrentSpeed();

        if (m_trajectory->advance(currentRampSpeed * interval)) {
            Angle oldTheta = setpoint.theta;

            Position2D<Meter> position = m_trajectory->getCurrentPosition();
            setpoint.x = position.x;
            setpoint.y = position.y;
            setpoint.theta = position.theta + m_kind.getAlignmentOffset();

            m_ramp.setTargetSpeed(m_maxSpeeds.linear);
            std::optional<double_t> remainingDistance = m_trajectory->getRemainingDistance();
            if (remainingDistance) {
                m_ramp.ensureCanBrake(*remainingDistance);
            }

            double_t angularSpeed = abs(setpoint.theta - oldTheta) / interval;
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

        } else {
            m_trajectory.reset();
        }
    }
    if (m_trajectory) {
        if (abs(setpoint.theta - actualRobotPosition.theta) > std::numbers::pi_v<double_t> / 2.0) {
            return BadRobotOrientation(std::move(m_trajectory), m_kind, m_finalOrientation);
        }
        return Ongoing();
    } else {
        return TrajectoryComplete(m_kind, m_finalOrientation);
    }
}

void StateTrajectoryWithRamp::notify(ControllerEvent event) {
    std::visit(overload{[&](const MaxSpeedsChanged &event) { m_maxSpeeds = event.newSpeeds; }, [](auto) {}}, event);
}

} // namespace controller