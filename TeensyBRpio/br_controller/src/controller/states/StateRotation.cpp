#include "controller/states/StateRotation.hpp"
#include "defines/func.hpp"
#include "logging.hpp"
#include "rotations/OrientationProfile.hpp"

namespace controller {

StateRotation::StateRotation(OrientationProfile *profile, number_t maxAngSpeed, number_t maxAngAcceleration)
    : m_profile(profile), m_maxSpeed(maxAngSpeed), m_ramp(maxAngSpeed, maxAngAcceleration) {}

StateInitialRotation::StateInitialRotation(OrientationProfile *profile, number_t maxAngSpeed, number_t maxAngAcceleration)
    : StateRotation(profile, maxAngSpeed, maxAngAcceleration) {
    log(INFO, "Entering controller state: Initial rotation");
}

StateFinalRotation::StateFinalRotation(OrientationProfile *profile, number_t maxAngSpeed, number_t maxAngAcceleration)
    : StateRotation(profile, maxAngSpeed, maxAngAcceleration) {
    log(INFO, "Entering controller state: Final rotation");
}

ControllerStatus StateRotation::getStatus() const {
    return ControllerStatus::ROTATING;
}

StateUpdateResult StateRotation::update(number_t interval) {
    if (m_profile) {
        m_ramp.update(interval);
        if (m_profile->advance(m_ramp.getCurrentSpeed() * interval)) {
            m_ramp.setTargetSpeed(m_maxSpeed);
            std::optional<number_t> remainingAngle = m_profile->getRemainingAngle();
            if (remainingAngle) {
                m_ramp.ensureCanBrake(*remainingAngle);
            }
        } else {
            m_profile = nullptr;
        }
    }
    if (m_profile) {
        return OrientationControl(m_profile->getCurrentOrientation());
    } else {
        return RotationComplete();
    }
}

void StateRotation::setMaxSpeed(number_t maxAngSpeed) {
    m_maxSpeed = maxAngSpeed;
}

} // namespace controller