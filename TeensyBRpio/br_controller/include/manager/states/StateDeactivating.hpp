#ifndef _MANAGER_STATE_DEACTIVATING_HPP_
#define _MANAGER_STATE_DEACTIVATING_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

/// Closed loop -> Idle
template <Actuators TActuators>
class StateDeactivating {
  public:
    /// @param actuators the reference must not escape the function
    StateDeactivating(TActuators &actuators);
    ManagerStatus getStatus() const;
    ManagerStatus update(TActuators &actuators);
};

} // namespace manager

#endif