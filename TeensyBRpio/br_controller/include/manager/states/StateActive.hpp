#ifndef _MANAGER_STATE_ACTIVE_HPP_
#define _MANAGER_STATE_ACTIVE_HPP_

#include "manager/ManagerState.hpp"

namespace manager {

template <Actuators TActuators>
class StateActive final : public ManagerState<TActuators> {
  public:
    StateActive();
    ManagerStatus getStatus() const override;
    ManagerStatus update(TActuators &actuators) override;
};

} // namespace manager

#endif