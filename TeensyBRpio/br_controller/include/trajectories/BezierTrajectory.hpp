#ifndef _BEZIER_TRAJECTORY_HPP_
#define _BEZIER_TRAJECTORY_HPP_

#include "trajectories/Trajectory.hpp"
#include "geometry/Bezier.hpp"
#include <vector>

/// A Bézier trajectory between the origin and the destination, with additional control points in-between.
class [[deprecated("not tested")]] BezierTrajectory : public Trajectory {

  public:
    /**
     * @param points There must be at least two points. The first point is the origin and the last point is
     * the destination. Other elements, if any, are the control points (the curve may NOT pass through them).
     */
    BezierTrajectory(std::vector<Point2D<Meter>> points);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override;

  private:
    BezierCurve m_curve;
    /// Between 0 and 1
    double_t m_position;
    /// In meters
    double_t m_totalLength;
};

#endif