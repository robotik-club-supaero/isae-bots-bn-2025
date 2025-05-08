#ifndef _CONTROLLER_STATE_MANUAL_CONTROL_HPP_
#define _CONTROLLER_STATE_MANUAL_CONTROL_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

namespace controller {

class StateManualControl final : public ControllerState {
  public:
    StateManualControl(Speeds speedRequest, Accelerations maxAcceleration);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    bool resumeState(Position2D<Meter> robotPosition) override;

    void setSpeed(Speeds speeds, bool enforceMaxAccelerations = true);

  private:
    Ramp m_linearRamp;
    Ramp m_angularRamp;
};
} // namespace controller

#endif