#ifndef _CONTROLLER_STATE_ROTATION_RAMP_HPP_
#define _CONTROLLER_STATE_ROTATION_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

class OrientationProfile;

namespace controller {

class StateRotation {
  public:
    ControllerStatus getStatus() const;
    StateUpdateResult update(number_t interval);
    void setMaxSpeed(number_t maxAngSpeed);

  protected:
    /// @param profile must not be null and must be valid for the lifetime of this state
    StateRotation(OrientationProfile *profile, number_t maxAngSpeed, number_t maxAngAcceleration);

  private:
    OrientationProfile *m_profile;
    number_t m_maxSpeed;
    Ramp m_ramp;
};

class StateInitialRotation final : public StateRotation {
  public:
    /// @param profile must not be null and must be valid for the lifetime of this state
    StateInitialRotation(OrientationProfile *profile, number_t maxAngSpeed, number_t maxAngAcceleration);
};

class StateFinalRotation final : public StateRotation {
  public:
    /// @param profile must not be null and must be valid for the lifetime of this state
    StateFinalRotation(OrientationProfile *profile, number_t maxAngSpeed, number_t maxAngAcceleration);
};

} // namespace controller
#endif