#include "manager/states/StateActive.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators>
StateActive<TActuators>::StateActive() {
    log(INFO, "Entering manager state: Active");
}

template <Actuators TActuators>
ManagerStatus StateActive<TActuators>::getStatus() const {
    return Active;
}

template <Actuators TActuators>
ManagerStatus StateActive<TActuators>::update(TActuators &actuators) {
    // TODO: check actuators state?
    return Active;
}

} // namespace manager

#include "specializations/actuators.hpp"
template class manager::StateActive<actuators_t>;
