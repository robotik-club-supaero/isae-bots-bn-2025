#ifndef _MANAGER_STATE_ACTIVATING_HPP_
#define _MANAGER_STATE_ACTIVATING_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

/// Idle -> Closed loop
template <Actuators TActuators>
class StateActivating {
  public:
    /// @param actuators the reference must not escape the function
    StateActivating(TActuators &actuators);
    ManagerStatus getStatus() const;
    ManagerStatus update(TActuators &actuators);
};

} // namespace manager

#endif