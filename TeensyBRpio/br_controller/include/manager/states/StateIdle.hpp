#ifndef _MANAGER_STATE_IDLE_HPP_
#define _MANAGER_STATE_IDLE_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

template <Actuators TActuators>
class StateIdle final : public ManagerState<TActuators> {
  public:
    StateIdle();
    ManagerStatus getStatus() const override;
    ManagerStatus update(TActuators &actuators) override;
};

} // namespace manager

#endif