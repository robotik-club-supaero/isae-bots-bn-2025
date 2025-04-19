#if 0
#include "Elevator.hpp"

#include "configuration.hpp"
#include "logging.hpp"

// TODO what should be the initial state?

ElevatorHandle::ElevatorHandle(int servo_pin, const char *name, std::array<int, 2> positions)
    : ActuatorHandle<2>(servo_pin, name, positions, /* initial_state = */ DOWN), m_was_refused(false) {}

bool ElevatorHandle::hasError() const { return m_was_refused; }
void ElevatorHandle::setHasError(bool error) { m_was_refused = error; }

Elevators::Elevators(ros2::Node &node)
    : m_handle_1(std::make_shared<ElevatorHandle>(ELEVATOR_1_PIN, "ELEVATOR1", std::array<int, 2>{ELEVATOR_1_DOWN_POS, ELEVATOR_1_UP_POS})),
      m_handle_2(std::make_shared<ElevatorHandle>(ELEVATOR_2_PIN, "ELEVATOR2", std::array<int, 2>{ELEVATOR_2_DOWN_POS, ELEVATOR_2_UP_POS})),
      m_elevator_1(node, "/act/order/elevator_1", "/act/callback/elevator_1", createCallback(m_handle_1, m_handle_2, 1)),
      m_elevator_2(node, "/act/order/elevator_2", "/act/callback/elevator_2", createCallback(m_handle_1, m_handle_2, 2)) {}

void Elevators::loop() {
    loop(*m_handle_1, m_elevator_1);
    loop(*m_handle_2, m_elevator_2);
}

bool Elevators::checkStates(const ElevatorHandle &handle1, const ElevatorHandle &handle2, int level, uint16_t new_state) {
    if (handle1.needsNotify() || handle2.needsNotify()) {
        return false;
    }
    if (level == 1 && new_state == UP && handle2.getState() == DOWN) {
        return false;
    }
    if (level == 2 && new_state == DOWN && handle1.getState() == UP) {
        return false;
    }
    return true;
}
void Elevators::loop(ElevatorHandle &handle, ActuatorTopics &topics) {
    if (handle.hasError() || (handle.needsNotify() && micros() - handle.lastChangeTime() > CALLBACK_INTERVAL)) {
        topics.sendCallback(handle.getState());
        handle.markNotified();
        handle.setHasError(false);
    }
}

std::function<void(uint16_t)> Elevators::createCallback(const std::shared_ptr<ElevatorHandle> &handle1,
                                                        const std::shared_ptr<ElevatorHandle> &handle2, int level) {
    return [level, handle_1 = std::weak_ptr<ElevatorHandle>(handle1), handle_2 = std::weak_ptr<ElevatorHandle>(handle2)](uint16_t msg) {
        if (auto handle1 = handle_1.lock()) {
            if (auto handle2 = handle_2.lock()) {
                ElevatorHandle &handle = level == 1 ? *handle1 : *handle2;
                if (Elevators::checkStates(*handle1, *handle2, level, msg)) {
                    handle.setState(msg);
                } else {
                    handle.setHasError(true);
                    log(WARN, String("Refusing to move elevator ").concat(level).concat(" - move the other elevator first"));
                }
            }
        }
    };
}
#endif