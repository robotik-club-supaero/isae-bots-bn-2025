#ifndef _POLYGONAL_TRAJECTORY_HPP
#define _POLYGONAL_TRAJECTORY_HPP

#include "stl/SmallDeque.hpp"
#include "trajectories/LinearTrajectory.hpp"

/**
 * A (non-smooth) polygonal trajectory. In other words, this connects crosspoints with straight lines.
 */
class PolygonalTrajectory final : public Trajectory {
  public:
    PolygonalTrajectory(SmallDeque<Point2D<Meter>> path);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::advanceSmooth()
    bool advanceSmooth(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override;

    /// @copydoc Trajectory::getMaxCurvature()
    double_t getMaxCurvature(double_t distance) override;

    /// @copydoc Trajectory::recompute()
    bool recompute(Position2D<Meter> newStartPosition) override;

  private:
    bool setupNextLine();

    SmallDeque<Point2D<Meter>> m_path;
    LinearTrajectory m_currentLine;
};

#endif