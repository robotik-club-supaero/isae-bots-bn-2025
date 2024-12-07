#ifndef _PATH_TRAJECTORY_HPP_
#define _PATH_TRAJECTORY_HPP_

#include "geometry/Bezier.hpp"
#include "trajectories/Trajectory.hpp"
#include <vector>

/**
 * A smooth trajectory that passes through crosspoints.
 *
 * The trajectory follows a Bézier curve instead of the straight line between two successive crosspoints
 * in order to enforce the continuity of the direction.
 */
class PathTrajectory : public Trajectory {

  public:
    using size_type = std::vector<Point2D<Meter>>::size_type;

    /// @param points Ordered list of crosspoints. The first point must be the start of the trajectory.
    /// There must be at least two points.
    PathTrajectory(Angle initialDirection, std::vector<Point2D<Meter>> points);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    /// May be under-estimated since the Bézier curves are lazily computed.
    std::optional<double_t> getRemainingDistance() const override;

    /// The crosspoints the trajectory must pass through.
    // Only used in tests
    const std::vector<Point2D<Meter>> &getPathPoints() const;
    /// The current Bézier curve followed by this trajectory until the next crosspoint is reached.
    // Only used in tests
    const BezierCurve &getCurrentBezierArc() const;
    /**
     * Returns the index of the current Bézier curve. If the trajectory is not complete, this is the index
     * of the last crosspoint that was reached. If the trajectory is complete, this is the index of the penultimate
     * crosspoint.
     */
    // Only used in tests
    size_type getCurrentArcIndex() const;

    /**
     * Advances the trajectory to the next Bézier curve (i.e. to the next crosspoint).
     * If the current Bézier curve is already the last one, this advances to the end of the trajectory and returns false.
     * Otherwise, this returns true.
     */
    // Only used in tests
    bool skipToNextArc();

  private:
    inline constexpr size_type numberOfArcs() const { return m_points.size() - 1; }
    void setupArc(Angle initialDirection);
    void updateRemainingLength();

    std::vector<Point2D<Meter>> m_points;

    size_type m_currentArcIndex;
    /// Never null except during object initialization
    std::optional<BezierCurve> m_currentArc;
    /// In meters
    double_t m_currentArcLength;
    /// Between 0 and 1
    double_t m_currentArcPosition;
    Vector2D<Meter> m_currentDerivative;

    /// A lower bound of the cumulative length of the remaining arcs (excluding the current arc).
    double_t m_remainingLength;
};

#endif