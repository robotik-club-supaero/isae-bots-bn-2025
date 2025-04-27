#include "controller/states/StateRotation.hpp"
#include "defines/func.hpp"
#include "logging.hpp"
#include "rotations/OrientationProfile.hpp"

namespace controller {

StateRotation::StateRotation(std::unique_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration)
    : m_profile(std::move(profile)), m_maxSpeed(maxAngSpeed), m_ramp(maxAngSpeed, maxAngAcceleration) {}

ControllerStatus StateRotation::getStatus() const {
    return ControllerStatus::ROTATING;
}

StateUpdateResult StateRotation::update(double_t interval) {
    if (m_profile) {
        m_ramp.update(interval);
        if (m_profile->advance(m_ramp.getCurrentSpeed() * interval)) {
            m_ramp.setTargetSpeed(m_maxSpeed);
            std::optional<double_t> remainingAngle = m_profile->getRemainingAngle();
            if (remainingAngle) {
                m_ramp.ensureCanBrake(*remainingAngle);
            }
        } else {
            m_profile.reset();
        }
    }
    if (m_profile) {
        return OrientationControl(m_profile->getCurrentOrientation());
    } else {
        return RotationComplete();
    }
}

void StateRotation::notify(ControllerEvent event) {
    std::visit(overload{[&](const MaxSpeedsChanged &event) { m_maxSpeed = event.newSpeeds.angular; }, [](auto) {}}, event);
}

bool StateRotation::resumeState(Position2D<Meter> robotPosition) {
    if (m_profile->recompute(robotPosition.theta)) {
        log(INFO, "Rotation profile recomputed.");

        m_ramp.overwriteCurrentSpeed(0.);
        return true;
    } else {
        log(WARN, "Failed to recompute the rotation profile. Aborting rotation...");
        return false;
    }
}

} // namespace controller