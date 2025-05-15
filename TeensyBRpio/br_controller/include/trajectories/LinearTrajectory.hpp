#ifndef _LINEAR_TRAJECTORY_HPP
#define _LINEAR_TRAJECTORY_HPP

#include "trajectories/Trajectory.hpp"

/// A straight line between the origin and the destination.
class LinearTrajectory final : public SmoothTrajectory {
  public:
    LinearTrajectory(Point2D<Meter> origin, Point2D<Meter> destination);

    /// @copydoc Trajectory::advance()
    bool advance(number_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<number_t> getRemainingDistance() const override;

    /// @copydoc Trajectory::getMaxCurvature()
    number_t getMaxCurvature(number_t distance) override;

    /// @copydoc Trajectory::recompute()
    bool recompute(Position2D<Meter> newStartPosition) override;

  private:
    Point2D<Meter> m_origin;
    Point2D<Meter> m_destination;
    Vector2D<Meter> m_direction;
    number_t m_totalLength;
    number_t m_position;
};

#endif