#ifndef _PATH_TRAJECTORY_HPP_
#define _PATH_TRAJECTORY_HPP_

#include "geometry/Bezier.hpp"
#include "trajectories/MultiCurveTrajectory.hpp"

/**
 * Generates Bézier curves between the crosspoints.
 * How the curves are generated is unspecified and subject to change.
 * See implementation for details.
 *
 * See concept CurveGenerator (in file MultiCurveTrajectory.hpp).
 */
class BezierCurvesGenerator {
  public:
    using size_type = std::vector<Point2D<Meter>>::size_type;

    BezierCurvesGenerator(std::optional<Angle> initialDirection, std::optional<Angle> finalDirection, std::vector<Point2D<Meter>> points);

    BezierCurve<4> next();
    bool hasNext() const;

    const std::vector<Point2D<Meter>> &getPoints() const;
    size_type getCurrentCurveIndex() const;
    const std::optional<Angle> &getPreferredFinalDirection() const;

  private:
    std::vector<Point2D<Meter>> m_points;
    Vector2D<Meter> m_initialDirection;
    std::optional<Angle> m_finalDirection;
    size_type m_currentCurveIndex;
};

/**
 * A smooth trajectory that passes through crosspoints.
 *
 * The trajectory follows a Bézier curve instead of a straight line between two successive crosspoints in order to enforce
 * the continuity of the direction.
 */
class PathTrajectory : public MultiCurveTrajectory<BezierCurve<4>, BezierCurvesGenerator> {

  public:
    using size_type = BezierCurvesGenerator::size_type;

    /// @param initialDirection If it is not empty, the trajectory will try to start in this direction except
    /// if this would cause a big detour. How the trajectory decides whether or not to take the initial direction
    /// into account is unspecified.
    /// @param finalDirection If it is not empty, the trajectory will try to end in this direction except if this
    /// would cause a big detour. How the trajectory decides whether or not to take the final direction into account
    /// is unspecified.
    /// @param points Ordered list of crosspoints. The first point must be the start of the trajectory.
    /// There must be at least two points.
    PathTrajectory(std::optional<Angle> initialDirection, std::optional<Angle> finalDirection, std::vector<Point2D<Meter>> points);

    /// The crosspoints the trajectory must pass through.
    const std::vector<Point2D<Meter>> &getPathPoints() const;

    /// @copydoc Trajectory::recompute()
    bool recompute(Position2D<Meter> newStartPosition) override;
};

#endif