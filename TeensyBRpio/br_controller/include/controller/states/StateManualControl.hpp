#ifndef _CONTROLLER_STATE_MANUAL_CONTROL_HPP_
#define _CONTROLLER_STATE_MANUAL_CONTROL_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

namespace controller {

class StateManualControl : public ControllerState {
  public:
    StateManualControl(Accelerations maxAcceleration);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    void notify(ControllerEvent event) override;

  private:
    Ramp m_linearRamp;
    Ramp m_angularRamp;
};
} // namespace controller

#endif