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

} // namespace manager

#endif