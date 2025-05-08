#ifndef _MANAGER_STATE_IDLE_HPP_
#define _MANAGER_STATE_IDLE_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

template <Actuators TActuators>
class StateIdle {
  public:
    StateIdle();
    ManagerStatus getStatus() const;
    ManagerStatus update(TActuators &actuators);
};

} // namespace manager

#endif