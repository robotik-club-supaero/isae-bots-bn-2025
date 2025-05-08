#ifndef _CONTROLLER_STATE_ROTATION_RAMP_HPP_
#define _CONTROLLER_STATE_ROTATION_RAMP_HPP_

#include "controller/ControllerState.hpp"
#include "math/Ramp.hpp"

class OrientationProfile;

namespace controller {

class StateRotation {
  public:
    ControllerStatus getStatus() const;
    StateUpdateResult update(double_t interval);
    void setMaxSpeed(double_t maxAngSpeed);

  protected:
    /// @param profile must not be null
    StateRotation(OrientationProfile *profile, double_t maxAngSpeed, double_t maxAngAcceleration);

  private:
    OrientationProfile *m_profile;
    double_t m_maxSpeed;
    Ramp m_ramp;
};

class StateInitialRotation final : public StateRotation {
  public:
    /// @param profile must not be null
    StateInitialRotation(OrientationProfile *profile, double_t maxAngSpeed, double_t maxAngAcceleration);
};

class StateFinalRotation final : public StateRotation {
  public:
    /// @param profile must not be null
    StateFinalRotation(OrientationProfile *profile, double_t maxAngSpeed, double_t maxAngAcceleration);
};

} // namespace controller
#endif