#ifndef _CONTROLLER_STATE_STAND_STILL_HPP_
#define _CONTROLLER_STATE_STAND_STILL_HPP_

#include "controller/ControllerState.hpp"

namespace controller {

class StateStandStill : public ControllerState {
  public:
    StateStandStill(Position2D<Meter> restPoint, bool advertize = true);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;

  private:
    Position2D<Meter> m_restPosition;
};
} // namespace controller

#endif