#include "manager/states/StateIdle.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators>
StateIdle<TActuators>::StateIdle() {
    log(INFO, "Entering manager state: Idle");
}

template <Actuators TActuators>
ManagerStatus StateIdle<TActuators>::getStatus() const {
    return Idle;
}

template <Actuators TActuators>
ManagerStatus StateIdle<TActuators>::update(TActuators &actuators) {
    // TODO: check actuators state?
    return Idle;
}

} // namespace manager

#include "specializations/actuators.hpp"
template class manager::StateIdle<actuators_t>;
