#ifndef _SET_HEADING_PROFILE_HPP_
#define _SET_HEADING_PROFILE_HPP_

#include "rotations/OrientationProfile.hpp"

/**
 * An OrientationProfile that orientates the robot in the given heading (i.e. direction).
 */
class SetHeadingProfile : public OrientationProfile {
  public:
    SetHeadingProfile(Angle initialHeading, Angle targetHeading);

    /**
     * @copydoc OrientationProfile::advance()
     */
    bool advance(double_t diffAngle) override;

    /**
     * @copydoc OrientationProfile::getCurrentOrientation()
     */
    Angle getCurrentOrientation() const override;

    /**
     * @copydoc OrientationProfile::getRemainingAngle()
     */
    std::optional<double_t> getRemainingAngle() const override;

    /**
     * @copydoc OrientationProfile::recompute
     */
    bool recompute(Angle newStartOrientation) override;

  private:
    Angle m_currentHeading;
    Angle m_targetHeading;
};

#endif