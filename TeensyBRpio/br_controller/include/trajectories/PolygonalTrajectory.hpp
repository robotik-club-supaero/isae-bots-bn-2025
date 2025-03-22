#ifndef _POLYGONAL_TRAJECTORY_HPP
#define _POLYGONAL_TRAJECTORY_HPP

#include "trajectories/LinearTrajectory.hpp"

#include <vector>

/**
 * A (non-smooth) polygonal trajectory. In other words, this connects crosspoints with straight lines.
 */
class PolygonalTrajectory : public Trajectory {
  public:
    PolygonalTrajectory(std::vector<Point2D<Meter>> path);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::advanceSmooth()
    bool advanceSmooth(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override;

    /// @copydoc Trajectory::getMaxCurvature()
    double_t getMaxCurvature(double_t distance) const override;

    /// @copydoc Trajectory::recompute()
    bool recompute(Position2D<Meter> newStartPosition) override;

  private:
    bool setupNextLine();

    std::vector<Point2D<Meter>> m_path;
    size_t m_index;
    LinearTrajectory m_currentLine;
};

#endif