#ifndef _CONTROLLER_STATE_ROTATION_RAMP_HPP_
#define _CONTROLLER_STATE_ROTATION_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

#include <memory>

class OrientationProfile;

namespace controller {

class StateRotation : public ControllerState {
  public:
    /// @param profile must not be null
    StateRotation(std::unique_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration);
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    void notify(ControllerEvent event) override;
    bool resumeState(Position2D<Meter> robotPosition) override;

  private:
    std::unique_ptr<OrientationProfile> m_profile;
    double_t m_maxSpeed;
    Ramp m_ramp;
};

} // namespace controller
#endif