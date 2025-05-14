#ifndef _CURVE_TRAJECTORY_HPP_
#define _CURVE_TRAJECTORY_HPP_

#include "geometry/CurveSampling.hpp"
#include "logging.hpp"
#include "math/RollingMax.hpp"
#include "trajectories/Trajectory.hpp"

#include <concepts>

/// A trajectory that follows a differentiable parametric curve.
/// @tparam TCurve The type of the trajectory. Must be a complete type.
template <Curve TCurve>
class CurveTrajectory final : public SmoothTrajectory {
  public:
    /**
     * @param curve
     * @param samplingStep The step (in meter) used to estimate the length of the trajectory and sample the curvature
     * @param advanceMaxStep The maximum integration step (in meter) used in `advance()`. If the requested distance is greater,
     * it will be split in multiple steps less than or equal to the maximum step.
     */
    CurveTrajectory(TCurve curve, double_t samplingStep = 0.005, double_t samplingTolerance = 0.005)
        : m_curve(std::move(curve)), m_sampling(samplingStep, samplingTolerance), m_curvature(samplingStep), m_distance(0), m_t(0) {}

    template <typename... Args>
        requires std::is_constructible<TCurve, Args...>::value
    CurveTrajectory(Args &&...args) : CurveTrajectory(TCurve(std::forward<Args>(args)...)) {}

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override {
        if (m_sampling.lengthEstimationComplete() && m_distance >= m_sampling.length()) {
            return false;
        }

        m_distance += distance;
        // If `m_sampling` enters in "degraded" mode due to insufficient memory, `solveInverseArcLength` could produce
        // inconsistent results. We ensure `m_t` is not decreasing (the robot would not like it at all).
        m_t = std::max(m_t, m_sampling.solveInverseArcLength(m_curve, m_distance));

        return true;
    }

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override { return Position2D<Meter>(m_curve(m_t), m_curve.derivative(m_t).argument()); }

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override {
        if (m_sampling.lengthEstimationComplete()) {
            return std::max(static_cast<double_t>(0.0), m_sampling.length() - m_distance);
        } else {
            return std::nullopt;
        }
    }
    /// @copydoc Trajectory::getMaxCurvature()
    double_t getMaxCurvature(double_t distance) override {
        forceGenerate(distance);

        return m_curvature.getMaximum(m_distance, m_distance + distance, [&](double_t dist) {
            double_t t = m_sampling.solveInverseArcLength(m_curve, dist);
            return std::abs(m_curve.curvature(t));
        });
    }

    const TCurve &getCurve() const { return m_curve; }

    void forceGenerate(double_t distance = std::numeric_limits<double_t>::max()) {
        m_sampling.sample(m_curve, m_distance + distance);
        if (m_sampling.isDegraded()) {
            m_sampling.discardSamples(m_distance);
        }
    }

#ifdef _BR_DEBUG
    double_t sampledDistance() const { return m_sampling.sampledLength(); }
    std::size_t numberOfSamplePoints() const { return m_sampling.numberOfSamplePoints(); }
    std::size_t numberOfCurvatureExtrema() const { return m_curvature.getExtrema().size(); }
    bool isCurvatureIncreasing() const { return m_curvature.isStartIncreasing(); }
#endif

  private:
    TCurve m_curve;

    CurveSampling m_sampling;
    RollingMax m_curvature;

    double_t m_distance;
    double_t m_t;
};

#endif