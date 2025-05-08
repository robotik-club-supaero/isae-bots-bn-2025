#ifndef _CONTROLLER_STATE_ROTATION_RAMP_HPP_
#define _CONTROLLER_STATE_ROTATION_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

#include <memory> //FIXME

class OrientationProfile;

namespace controller {

class StateRotation : public ControllerState {
  public:
    ControllerStatus getStatus() const override;
    StateUpdateResult update(double_t interval) override;
    bool resumeState(Position2D<Meter> robotPosition) override;
    void setMaxSpeed(double_t maxAngSpeed);

  protected:
    /// @param profile must not be null
    StateRotation(std::shared_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration);

  private:
    std::shared_ptr<OrientationProfile> m_profile;
    double_t m_maxSpeed;
    Ramp m_ramp;
};

class StateInitialRotation final : public StateRotation {
  public:
    /// @param profile must not be null
    StateInitialRotation(std::shared_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration);
};

class StateFinalRotation final : public StateRotation {
  public:
    /// @param profile must not be null
    StateFinalRotation(std::shared_ptr<OrientationProfile> profile, double_t maxAngSpeed, double_t maxAngAcceleration);
};

} // namespace controller
#endif