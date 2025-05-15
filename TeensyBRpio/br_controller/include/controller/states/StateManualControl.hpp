#ifndef _CONTROLLER_STATE_MANUAL_CONTROL_HPP_
#define _CONTROLLER_STATE_MANUAL_CONTROL_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

namespace controller {

class StateManualControl {
  public:
    StateManualControl(Speeds speedRequest, Accelerations maxAcceleration);
    ControllerStatus getStatus() const;
    StateUpdateResult update(number_t interval);

    void setSpeed(Speeds speeds, bool enforceMaxAccelerations = true);

  private:
    Ramp m_linearRamp;
    Ramp m_angularRamp;
};
} // namespace controller

#endif