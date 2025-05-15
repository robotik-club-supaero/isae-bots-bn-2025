#include "controller/states/StateBraking.hpp"
#include "logging.hpp"

namespace controller {

StateBraking::StateBraking(Speeds initialSpeeds, Accelerations brakeAccelerations)
    : m_linRamp(0, brakeAccelerations.linear, initialSpeeds.linear), m_angRamp(0, brakeAccelerations.angular, initialSpeeds.angular) {
    log(INFO, "Entering controller state: Braking");
}

ControllerStatus StateBraking::getStatus() const {
    return ControllerStatus::Braking;
}

StateUpdateResult StateBraking::update(number_t interval) {
    m_linRamp.update(interval);
    m_angRamp.update(interval);

    Speeds speeds(m_linRamp.getCurrentSpeed(), m_angRamp.getCurrentSpeed());
    if (speeds == Speeds(0, 0)) {
        return BrakingComplete();
    }
    return SpeedControl(speeds);
}

} // namespace controller
