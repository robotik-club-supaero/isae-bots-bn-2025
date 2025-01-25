#ifndef _CONTROLLER_STATE_BRAKING_HPP_
#define _CONTROLLER_STATE_BRAKING_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

namespace controller {

class StateBraking : public ControllerState {
  public:
    StateBraking(Speeds initialSpeeds, Accelerations brakeAccelerations);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;

  private:
    Ramp m_linRamp;
    Ramp m_angRamp;
};

} // namespace controller
#endif