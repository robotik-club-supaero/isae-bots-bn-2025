#include "controller/states/StateManualControl.hpp"
#include "defines/func.hpp"
#include "logging.hpp"

constexpr bool FORCE_POSITION_CONTROL = false;

namespace controller {

StateManualControl::StateManualControl(Speeds speedRequest, Accelerations maxAcceleration)
    : m_linearRamp(speedRequest.linear, maxAcceleration.linear), m_angularRamp(speedRequest.angular, maxAcceleration.angular) {
    log(INFO, "Entering controller state: Manual control");
}

ControllerStatus StateManualControl::getStatus() const {
    return ControllerStatus::ManualControl;
}

StateUpdateResult StateManualControl::update(double_t interval) {
    m_linearRamp.update(interval);
    m_angularRamp.update(interval);

    if constexpr (FORCE_POSITION_CONTROL) {
        Speeds speeds(m_linearRamp.getCurrentSpeed(), m_angularRamp.getCurrentSpeed());
        return PositionControl(Position2D<Meter>() + speeds * interval, /* relative = */ true);
    } else {
        return SpeedControl(Speeds(m_linearRamp.getCurrentSpeed(), m_angularRamp.getCurrentSpeed()));
    }
}

void StateManualControl::setSpeed(Speeds speeds, bool enforceMaxAcceleration) {
    m_linearRamp.setTargetSpeed(speeds.linear);
    m_angularRamp.setTargetSpeed(speeds.angular);
    if (!enforceMaxAcceleration) {
        m_linearRamp.overwriteCurrentSpeed(speeds.linear);
        m_angularRamp.overwriteCurrentSpeed(speeds.angular);
    }
}

bool StateManualControl::resumeState(Position2D<Meter> robotPosition) {
    m_linearRamp.overwriteCurrentSpeed(0.);
    m_angularRamp.overwriteCurrentSpeed(0.);

    return true;
}

} // namespace controller
