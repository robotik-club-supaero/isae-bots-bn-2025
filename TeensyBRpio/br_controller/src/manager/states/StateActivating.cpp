#include "manager/states/StateActivating.hpp"
#include "logging.hpp"

namespace manager {

template <Actuators TActuators>
StateActivating<TActuators>::StateActivating(TActuators &actuators) {
    log(INFO, "Entering manager state: Activating");
    actuators.switchOn();
}

template <Actuators TActuators>
ManagerStatus StateActivating<TActuators>::getStatus() const {
    return Activating;
}

template <Actuators TActuators>
ManagerStatus StateActivating<TActuators>::update(TActuators &actuators) {
    // TODO: check `actuators.isIdle()`?
    if (actuators.isReady()) {
        return Active;
    } else {
        return Activating;
    }
}

} // namespace manager

#include "specializations/actuators.hpp"
template class manager::StateActivating<actuators_t>;
