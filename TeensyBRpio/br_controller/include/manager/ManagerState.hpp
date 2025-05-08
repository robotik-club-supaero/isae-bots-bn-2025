#ifndef _MANAGER_STATE_HPP_
#define _MANAGER_STATE_HPP_

#include "Actuators.hpp"

namespace manager {

enum ManagerStatus {
    /// Controller inactive
    Idle,
    /// setActive(false) called, waiting for the motors to get idle
    Deactivating,
    /// setActive(true) called, waiting for the motors to get ready
    Activating,
    /// Controller active
    Active
};

/// State of the manager
template <Actuators TActuators>
class ManagerState {
  public:
    virtual ManagerStatus getStatus() const = 0;
    /// @param actuators The reference must not escape the function
    virtual ManagerStatus update(TActuators &actuators) = 0;

  protected:
    ManagerState();
};

} // namespace manager

#endif