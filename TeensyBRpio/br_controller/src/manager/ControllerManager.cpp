#include "manager/ControllerManager.hpp"
#include "logging.hpp"

#include <memory>
#include <string>

#define TEMPLATE template <Actuators TActuators, CanControl<TActuators> TController, PositionFeedback TFeedback, Clock TClock>
#define MANAGER ControllerManager<TActuators, TController, TFeedback, TClock>

namespace manager {

TEMPLATE
MANAGER::ControllerManager(duration_t updateInterval, TClock clock, TController controller, TActuators actuators, TFeedback feedback)
    : ControllerManager(updateInterval, updateInterval, std::move(clock), std::move(controller), std::move(actuators), std::move(feedback)) {}

TEMPLATE
MANAGER::ControllerManager(duration_t minUpdateInterval, duration_t maxUpdateInterval, TClock clock, TController controller, TActuators actuators,
                           TFeedback feedback)
    : m_clock(std::move(clock)), m_minUpdateInterval(minUpdateInterval), m_maxUpdateInterval(maxUpdateInterval), m_lastUpdate(),
      m_controller(std::move(controller)), m_actuators(std::move(actuators)), m_feedback(std::move(feedback)) {
    this->template setCurrentState<StateIdle>();
}

TEMPLATE
ManagerStatus MANAGER::getStatus() const {
    return this->template getStateStatus<ManagerStatus>();
}

TEMPLATE
void MANAGER::setActive(bool active) {
    if (active) {
        if (this->getStatus() == Active || this->getStatus() == Activating) {
            log(WARN, "The controller is already active.");
        } else {
            this->template setCurrentState<StateActivating>(m_actuators);
        }
    } else {
        if (this->getStatus() == Idle || this->getStatus() == Deactivating) {
            log(WARN, "The controller is already inactive.");
        } else {
            m_controller.reset();
            this->template setCurrentState<StateDeactivating>(m_actuators);
        }
    }
}

TEMPLATE
bool MANAGER::update() {
    instant_t nowMicros = m_clock.micros();
    if (!m_lastUpdate) {
        m_lastUpdate.emplace(nowMicros);
        return false;
    }

    duration_t intervalMicros = getDurationMicros(*m_lastUpdate, nowMicros);
    if (intervalMicros >= m_minUpdateInterval) {
        duration_t tickInterval = std::min(m_maxUpdateInterval, intervalMicros);
        *m_lastUpdate += tickInterval;
        update((double_t)tickInterval / 1e6);
        return true;
    } else {
        return false;
    }
}

TEMPLATE
void MANAGER::update(double_t interval) {
    ManagerStatus status = updateState<ManagerStatus, TActuators&>(m_actuators);
    if (status != this->getStatus()) {
        switch (status) {
            case Idle:
                this->template setCurrentState<StateIdle>();
                break;
            case Active:
                this->template setCurrentState<StateActive>();
                m_controller.reset(m_feedback.getRobotPosition());
                break;

            case Activating:
                // Only returned by StateActivating
            case Deactivating:
                // Only returned by StateDeactivating
            default:
                log(ERROR, "Invalid response from manager state!" + std::to_string(status));
        }
    }

    m_feedback.update(interval);
    if (isActive()) {
        auto command = m_controller.updateCommand(interval, m_feedback.getRobotPosition());
        m_actuators.sendCommand(command);
    };
    m_actuators.update(interval);
}

TEMPLATE
void MANAGER::resyncClock() {
    m_lastUpdate.reset();
}

TEMPLATE
duration_t MANAGER::getMinUpdateInterval() const {
    return m_minUpdateInterval;
}
TEMPLATE
duration_t MANAGER::getMaxUpdateInterval() const {
    return m_maxUpdateInterval;
}

TEMPLATE
void MANAGER::setMinUpdateInterval(duration_t updateInterval) {
    m_minUpdateInterval = updateInterval;
}

TEMPLATE
void MANAGER::setMaxUpdateInterval(duration_t updateInterval) {
    m_maxUpdateInterval = updateInterval;
}

TEMPLATE
void MANAGER::setUpdateInterval(duration_t updateInterval) {
    setMinUpdateInterval(updateInterval);
    setMaxUpdateInterval(updateInterval);
}

TEMPLATE
void MANAGER::resetPosition(Position2D<Meter> newPosition) {
    m_feedback.resetPosition(newPosition);
    if (isActive()) {
        m_controller.reset(newPosition);
    }
}

TEMPLATE
const TController &MANAGER::getController() const {
    return m_controller;
}
TEMPLATE
TController &MANAGER::getController() {
    return m_controller;
}
TEMPLATE
const TActuators &MANAGER::getActuators() const {
    return m_actuators;
}
TEMPLATE
const TFeedback &MANAGER::getPositionFeedback() const {
    return m_feedback;
}
TEMPLATE
const TClock &MANAGER::getClock() const {
    return m_clock;
}

} // namespace manager

// Explicit instantiation of the manager
// Template classes need either to have all their implementation in the .hpp file or to be explicitly instantiated for the particular types they are
// used with.
#include "specializations/manager.hpp"
template class manager::ControllerManager<actuators_t, controller_t, feedback_t, _clock_t>;
