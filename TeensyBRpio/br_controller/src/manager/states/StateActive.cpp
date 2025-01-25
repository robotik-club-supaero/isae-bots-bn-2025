#include "manager/states/StateActive.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators, typename TController>
StateActive<TActuators, TController>::StateActive() {
    log(INFO, "Entering manager state: Active");
}

template <Actuators TActuators, typename TController>
ManagerStatus StateActive<TActuators, TController>::getStatus() const {
    return Active;
}

template <Actuators TActuators, typename TController>
ManagerStatus StateActive<TActuators, TController>::update(TActuators &actuators) {
    // TODO: check actuators state?
    return Active;
}

} // namespace manager

#include "specializations/actuators.hpp"
#include "specializations/controller.hpp"
template class manager::StateActive<actuators_t, controller_t>;
