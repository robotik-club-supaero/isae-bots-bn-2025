#ifndef _CONTROLLER_STATE_STAND_STILL_HPP_
#define _CONTROLLER_STATE_STAND_STILL_HPP_

#include "controller/ControllerState.hpp"

namespace controller {

class StateStandStill {
  public:
    StateStandStill() = default;
    StateStandStill(Position2D<Meter> restPoint);
    ControllerStatus getStatus() const;
    StateUpdateResult update(number_t interval);

  private:
    Position2D<Meter> m_restPosition;
};
} // namespace controller

#endif