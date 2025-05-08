#ifndef _MANAGER_STATE_ACTIVE_HPP_
#define _MANAGER_STATE_ACTIVE_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

template <Actuators TActuators>
class StateActive {
  public:
    StateActive();
    ManagerStatus getStatus() const;
    ManagerStatus update(TActuators &actuators);
};

} // namespace manager

#endif