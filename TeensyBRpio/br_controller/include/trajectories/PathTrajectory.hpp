#ifndef _PATH_TRAJECTORY_HPP_
#define _PATH_TRAJECTORY_HPP_

#include "geometry/Bezier.hpp"
#include "math/RollingMax.hpp"
#include "trajectories/Trajectory.hpp"
#include <deque>
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

    /// @param initialDirection If it is not empty, the trajectory will try to start in this direction except
    /// if this would cause a big detour. How the trajectory decides whether or not to take the initial direction
    /// into account is unspecified.
    /// @param points Ordered list of crosspoints. The first point must be the start of the trajectory.
    /// There must be at least two points.
    PathTrajectory(std::optional<Angle> initialDirection, std::vector<Point2D<Meter>> points);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    /// May be under-estimated since the Bézier curves are lazily computed.
    std::optional<double_t> getRemainingDistance() const override;

    /// @copydoc Trajectory::getMaxCurvature()
    /// For performance reasons, this is a rough estimation and may not be accurate, but this should be sufficient
    /// for what the result is used for.
    double_t getMaxCurvature(double_t distance) const override;

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
    struct BezierArc {
        BezierArc(BezierCurve curve)
            : curve(std::move(curve)), length(), lengthSamples(),
              curvature(Samples([&](double_t t) { return abs(this->curve.curvature(t)); }, 0, 1, 100)) {
            auto samples = this->curve.sampleLength();
            length = samples.totalLength;
            lengthSamples = std::move(samples.samples);
        }

        BezierCurve curve;
        /// In meters
        double_t length;
        /// Maps distance from start (in meters) to the curve parameter (t between 0 and 1)
        Samples<> lengthSamples;
        RollingMax curvature;

        /**
         * Interpolates the curve parameter `t` such that `B(t)` is the position on the curve after walking `distance` meters.
         *
         * If `distance` is greater than the length of the curve, this returns 1.
         * If `distance` is negative, the behavior is undefined.
         */
        double_t interpolatePosition(double_t distance) const;
    };

    inline size_type numberOfArcs() const { return m_points.size() - 1; }

    std::optional<BezierArc> generateNextArc() const;
    BezierArc generateArc(size_type index, Vector2D<Meter> initialDirection) const;
    void setupNextArc();
    void updateRemainingLength();

    std::vector<Point2D<Meter>> m_points;

    size_type m_currentArcIndex;
    /// Never null except during object initialization
    std::optional<BezierArc> m_currentArc;
    /// Between 0 and 1
    double_t m_currentArcPosition;
    /// In meters
    double_t m_currentArcDistance;
    Vector2D<Meter> m_currentDerivative;

    /// A lower bound of the cumulative length of the remaining arcs (excluding the current arc).
    double_t m_remainingLength;

    /// Pre-generated Bézier arcs (getMaxCurvature may need to generate arcs in advance).
    /// Generating Bézier curves is not costless, therefore they are stored so they don't need to be computed again later.
    mutable std::deque<BezierArc> m_generatedArcs;
};

#endif