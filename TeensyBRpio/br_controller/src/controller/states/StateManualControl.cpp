#include "controller/states/StateManualControl.hpp"
#include "defines/func.hpp"
#include "logging.hpp"

constexpr bool FORCE_POSITION_CONTROL = false;

namespace controller {

StateManualControl::StateManualControl(Accelerations maxAcceleration)
    : m_linearRamp(0, maxAcceleration.linear), m_angularRamp(0, maxAcceleration.angular) {
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

void StateManualControl::notify(ControllerEvent event) {
    std::visit(overload{[&](const ManualSpeedCommand &event) {
                            m_linearRamp.setTargetSpeed(event.speeds.linear);
                            m_angularRamp.setTargetSpeed(event.speeds.angular);
                            if (!event.enforceMaxAcceleration) {
                                m_linearRamp.overwriteCurrentSpeed(event.speeds.linear);
                                m_angularRamp.overwriteCurrentSpeed(event.speeds.angular);
                            }
                        },
                        [](auto) {}},
               event);
}

} // namespace controller
