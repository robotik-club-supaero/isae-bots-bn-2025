#include "manager/states/StateDeactivating.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators>
StateDeactivating<TActuators>::StateDeactivating(TActuators &actuators) {
    log(INFO, "Entering manager state: Deactivating");
    actuators.switchOff();
}

template <Actuators TActuators>
ManagerStatus StateDeactivating<TActuators>::getStatus() const {
    return Deactivating;
}

template <Actuators TActuators>
ManagerStatus StateDeactivating<TActuators>::update(TActuators &actuators) {
    // TODO: check `actuators.isReady()`?
    if (actuators.isIdle()) {
        return Idle;
    } else {
        return Deactivating;
    }
}

} // namespace manager

#include "specializations/actuators.hpp"
template class manager::StateDeactivating<actuators_t>;
